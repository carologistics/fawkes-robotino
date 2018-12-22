/***************************************************************************
 *  conveyor_pose_thread.cpp - conveyor_pose thread
 *
 *  Created: Thr 12. April 16:28:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>


#include <tf/types.h>
#include <utils/math/angle.h>

#include <core/exceptions/system.h>

#include <interfaces/SwitchInterface.h>
#include <interfaces/LaserLineInterface.h>
#include <interfaces/ConveyorPoseInterface.h>

#include <cmath>
#include <cstdio>

#include "conveyor_pose_thread.h"
#include "recognition_thread.h"

using namespace fawkes;

using Cloud = ConveyorPoseThread::Cloud;
using CloudPtr = ConveyorPoseThread::CloudPtr;

/** @class ConveyorPoseThread "conveyor_pose_thread.cpp"
 * Plugin to detect the conveyor belt in a pointcloud (captured from Intel RealSense)
 * @author Tobias Neumann
 */

/** Constructor. */
ConveyorPoseThread::ConveyorPoseThread()
  : Thread("ConveyorPoseThread", Thread::OPMODE_WAITFORWAKEUP)
  , BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
  , ConfigurationChangeHandler(CFG_PREFIX)
  , fawkes::TransformAspect(fawkes::TransformAspect::BOTH,"conveyor_pose")
  , result_fitness_(std::numeric_limits<double>::min())
  , syncpoint_clouds_ready_name("/perception/conveyor_pose/clouds_ready")
  , cloud_out_raw_name_("raw")
  , cloud_out_trimmed_name_("trimmed")
  , current_mps_type_(ConveyorPoseInterface::NO_STATION)
  , current_mps_target_(ConveyorPoseInterface::NO_LOCATION)
  , recognition_thread_(nullptr)
  , realsense_switch_(nullptr)
{}

void
ConveyorPoseThread::init()
{
  config->add_change_handler(this);

  syncpoint_clouds_ready = syncpoint_manager->get_syncpoint(name(), syncpoint_clouds_ready_name);
  syncpoint_clouds_ready->register_emitter(name());

  cfg_debug_mode_ = config->get_bool( CFG_PREFIX "/debug" );
  cloud_in_name_ = config->get_string( CFG_PREFIX "/cloud_in" );

  cfg_if_prefix_ = config->get_string( CFG_PREFIX "/if/prefix" );
  if (cfg_if_prefix_.back() != '/')
    cfg_if_prefix_.append("/");


  laserlines_names_       = config->get_strings( CFG_PREFIX "/if/laser_lines" );

  conveyor_frame_id_          = config->get_string( CFG_PREFIX "/conveyor_frame_id" );

  recognition_thread_->cfg_icp_max_corr_dist_     = config->get_float( CFG_PREFIX "/icp/max_correspondence_dist" );
  recognition_thread_->cfg_icp_max_iterations_    = config->get_int( CFG_PREFIX "/icp/max_iterations" );
  recognition_thread_->cfg_icp_refinement_factor_ = double(config->get_float( CFG_PREFIX "/icp/refinement_factor" ));
  recognition_thread_->cfg_icp_tf_epsilon_        = double(config->get_float( CFG_PREFIX "/icp/transformation_epsilon" ));
  recognition_thread_->cfg_icp_min_loops_         = config->get_uint( CFG_PREFIX "/icp/min_loops" );
  recognition_thread_->cfg_icp_max_loops_         = config->get_uint( CFG_PREFIX "/icp/max_loops" );
  recognition_thread_->cfg_icp_auto_restart_      = config->get_bool( CFG_PREFIX "/icp/auto_restart" );

  recognition_thread_->cfg_icp_hv_inlier_thresh_  = config->get_float( CFG_PREFIX "/icp/hv_inlier_threshold" );
  recognition_thread_->cfg_icp_hv_penalty_thresh_ = config->get_float( CFG_PREFIX "/icp/hv_penalty_threshold" );
  recognition_thread_->cfg_icp_hv_support_thresh_ = config->get_float( CFG_PREFIX "/icp/hv_support_threshold" );

  // Init of station target hints
  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR];
  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR];
  cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT];
  cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT];
  cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE];
  cfg_target_hint_[ConveyorPoseInterface::SLIDE];
  cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION];

  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][0]    = config->get_float( CFG_PREFIX "/icp/hint/conveyor/x" );
  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][1]    = config->get_float( CFG_PREFIX "/icp/hint/conveyor/y" );
  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][2]    = config->get_float( CFG_PREFIX "/icp/hint/conveyor/z" );

  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][0]   = config->get_float( CFG_PREFIX "/icp/hint/conveyor/x");
  // Y * -1, because it should be the opposite of the input conveyor
  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][1]   = -config->get_float( CFG_PREFIX "/icp/hint/conveyor/y");
  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][2]   = config->get_float( CFG_PREFIX "/icp/hint/conveyor/z");

  cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][0]  = config->get_float( CFG_PREFIX "/icp/hint/left_shelf/x" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][1]  = config->get_float( CFG_PREFIX "/icp/hint/left_shelf/y" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][2]  = config->get_float( CFG_PREFIX "/icp/hint/left_shelf/z" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][0]= config->get_float( CFG_PREFIX "/icp/hint/middle_shelf/x" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][1]= config->get_float( CFG_PREFIX "/icp/hint/middle_shelf/y" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][2]= config->get_float( CFG_PREFIX "/icp/hint/middle_shelf/z" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][0] = config->get_float( CFG_PREFIX "/icp/hint/right_shelf/x" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][1] = config->get_float( CFG_PREFIX "/icp/hint/right_shelf/y" );
  cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][2] = config->get_float( CFG_PREFIX "/icp/hint/right_shelf/z" );
  cfg_target_hint_[ConveyorPoseInterface::SLIDE][0]       = config->get_float( CFG_PREFIX "/icp/hint/slide/x" );
  cfg_target_hint_[ConveyorPoseInterface::SLIDE][1]       = config->get_float( CFG_PREFIX "/icp/hint/slide/y" );
  cfg_target_hint_[ConveyorPoseInterface::SLIDE][2]       = config->get_float( CFG_PREFIX "/icp/hint/slide/z" );

  cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION][0]  = config->get_float( CFG_PREFIX "/icp/hint/default/x" );
  cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION][1]  = config->get_float( CFG_PREFIX "/icp/hint/default/y" );
  cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION][2]  = config->get_float( CFG_PREFIX "/icp/hint/default/z" );

  // Init of station type hints

  cfg_type_hint_[ConveyorPoseInterface::BASE_STATION];
  cfg_type_hint_[ConveyorPoseInterface::CAP_STATION];
  cfg_type_hint_[ConveyorPoseInterface::RING_STATION];
  cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION];
  cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION];

  cfg_type_hint_[ConveyorPoseInterface::BASE_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/base_station/x", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::BASE_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/base_station/y", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::BASE_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/base_station/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::CAP_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/cap_station/x", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::CAP_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/cap_station/y", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::CAP_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/cap_station/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::RING_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/ring_station/x", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::RING_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/ring_station/y", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::RING_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/ring_station/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/delivery_station/x", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/delivery_station/y", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/delivery_station/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/storage_station/x", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/storage_station/y", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/storage_station/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::NO_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/default/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::NO_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/default/z", 0.f );
  cfg_type_hint_[ConveyorPoseInterface::NO_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/mps_type_offset/default/z", 0.f );


  cfg_enable_switch_          = config->get_bool( CFG_PREFIX "/switch_default" );

  //cut information

  cfg_left_cut_no_ll_         = config->get_float( CFG_PREFIX "/without_ll/left_cut" );
  cfg_right_cut_no_ll_        = config->get_float( CFG_PREFIX "/without_ll/right_cut" );
  cfg_top_cut_no_ll_          = config->get_float( CFG_PREFIX "/without_ll/top_cut" );
  cfg_bottom_cut_no_ll_       = config->get_float( CFG_PREFIX "/without_ll/bottom_cut" );
  cfg_front_cut_no_ll_        = config->get_float( CFG_PREFIX "/without_ll/front_cut" );
  cfg_back_cut_no_ll_         = config->get_float( CFG_PREFIX "/without_ll/back_cut" );

  cfg_left_cut_               = config->get_float( CFG_PREFIX "/with_ll/left_cut" );
  cfg_right_cut_              = config->get_float( CFG_PREFIX "/with_ll/right_cut" );
  cfg_top_cut_                = config->get_float( CFG_PREFIX "/with_ll/top_cut" );
  cfg_bottom_cut_             = config->get_float( CFG_PREFIX "/with_ll/bottom_cut" );
  cfg_front_cut_              = config->get_float( CFG_PREFIX "/with_ll/front_cut" );
  cfg_back_cut_               = config->get_float( CFG_PREFIX "/with_ll/back_cut" );

  cfg_gripper_bottom_         = config->get_float( CFG_PREFIX "/gripper/bottom" );  

  cfg_voxel_grid_leaf_size_  = config->get_float( CFG_PREFIX "/voxel_grid/leaf_size" );

  cfg_bb_realsense_switch_name_ = config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");
  wait_time_ = Time(double(config->get_float_or_default(CFG_PREFIX "/realsense_wait_time", 1.0f)));

  cfg_default_model_path_ = config->get_string(CFG_PREFIX "/default_model");
  if (cfg_default_model_path_.substr(0, 1) != "/")
    cfg_default_model_path_ = CONFDIR "/" + cfg_default_model_path_;

  // Load default reference pcl for shelf, input belt (with cone), output belt (without cone) and slide

  for ( int i = ConveyorPoseInterface::NO_STATION; i != ConveyorPoseInterface::LAST_MPS_TYPE_ELEMENT; i++ )
  {
    ConveyorPoseInterface::MPS_TYPE mps_type = static_cast<ConveyorPoseInterface::MPS_TYPE>(i);
    for (int j = ConveyorPoseInterface::NO_LOCATION; j != ConveyorPoseInterface::LAST_MPS_TARGET_ELEMENT; j++)
    {
      ConveyorPoseInterface::MPS_TARGET mps_target = static_cast<ConveyorPoseInterface::MPS_TARGET>(j);
      switch(mps_target)
      {
      case ConveyorPoseInterface::INPUT_CONVEYOR:
        type_target_to_path_[{mps_type,mps_target}] = CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_default_models/with_cone");
        break;
      case ConveyorPoseInterface::OUTPUT_CONVEYOR:
        type_target_to_path_[{mps_type,mps_target}] =CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_default_models/no_cone");
        break;
      case ConveyorPoseInterface::SHELF_LEFT:
        type_target_to_path_[{mps_type,mps_target}] = CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_default_models/shelf");
        break;
      case ConveyorPoseInterface::SHELF_MIDDLE:
        type_target_to_path_[{mps_type,mps_target}] = CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_default_models/shelf");
        break;
      case ConveyorPoseInterface::SHELF_RIGHT:
        type_target_to_path_[{mps_type,mps_target}] = CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_default_models/shelf");
        break;
      case ConveyorPoseInterface::SLIDE:
        type_target_to_path_[{mps_type,mps_target}] =CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_default_models/slide");
        break;
      case ConveyorPoseInterface::NO_LOCATION:
        break;
      case ConveyorPoseInterface::LAST_MPS_TARGET_ELEMENT:
        break;
      }
    }
  }

  trimmed_scene_.reset(new Cloud());

  cfg_model_origin_frame_ = config->get_string(CFG_PREFIX "/model_origin_frame");
  cfg_record_model_ = config->get_bool_or_default(CFG_PREFIX "/record_model", false);
  default_model_.reset(new Cloud());

  // if recording is set, a pointcloud will be written to the the cfg_record_path_
  // else load pcd file for every station and calculate model with normals

  if (cfg_record_model_) {

    std::string reference_station = config->get_string_or_default(CFG_PREFIX "/record_path","recorded_file.pcd");
    cfg_record_path_ = CONFDIR   "/" + reference_station;
    std::string new_record_path = cfg_record_path_;

    bool exists;
    FILE *tmp;
    size_t count = 0;

    do {
      exists = false;
      tmp = ::fopen(new_record_path.c_str(),"r");
      if(tmp) {
        exists = true;
        ::fclose(tmp);
        if(cfg_record_path_.substr(cfg_record_path_.length() - 4) == ".pcd")
          new_record_path = cfg_record_path_.substr(0,cfg_record_path_.length()-4)
              + std::to_string(count++) + ".pcd";
        else
          new_record_path = cfg_record_path_ + std::to_string(count++);

      }
    } while(exists);

    cfg_record_path_= new_record_path;

    logger->log_info(name(), "Writing point cloud to %s", cfg_record_path_.c_str());

  }
  else {

    //Load default PCD file from model path and calculate model with normals for it

    int errnum;
    if ((errnum = pcl::io::loadPCDFile(cfg_default_model_path_, *default_model_)) < 0)
      throw fawkes::CouldNotOpenFileException(cfg_default_model_path_.c_str(), errnum,
                                              "Set from " CFG_PREFIX "/default_model");

    model_ = default_model_;

    // Loading PCD file and calculation of model with normals for ALL! stations
    for (const auto &pair : type_target_to_path_) {
      CloudPtr model(new Cloud());
      if ((errnum = pcl::io::loadPCDFile(pair.second, *model)) < 0)
        throw fawkes::CouldNotOpenFileException(pair.second.c_str(), errnum,
                                                ("For station " )   ); //TODO:: print

      type_target_to_model_.insert({pair.first, model});
    }

  }

  cloud_in_registered_ = false;

  cloud_out_raw_ = new Cloud();
  cloud_out_trimmed_ = new Cloud();
  cloud_out_model_ = new Cloud();
  pcl_manager->add_pointcloud(cloud_out_raw_name_.c_str(), cloud_out_raw_);
  pcl_manager->add_pointcloud(cloud_out_trimmed_name_.c_str(), cloud_out_trimmed_);
  pcl_manager->add_pointcloud("model", cloud_out_model_);

  for (std::string ll : laserlines_names_) {
    laserlines_.push_back( blackboard->open_for_reading<fawkes::LaserLineInterface>(ll.c_str()) );
  }

  bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>((cfg_if_prefix_ + "switch").c_str());
  bb_enable_switch_->set_enabled( cfg_debug_mode_ || cfg_enable_switch_); // ignore cfg_enable_switch_ and set to true if debug mode is used
  bb_enable_switch_->write();

  bb_pose_ = blackboard->open_for_writing<ConveyorPoseInterface>((cfg_if_prefix_ + "status").c_str());
  bb_pose_->set_current_mps_type(bb_pose_->NO_STATION);
  bb_pose_->set_current_mps_target(bb_pose_->NO_LOCATION);

  bb_pose_->write();

  realsense_switch_ = blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());

  if (cfg_debug_mode_)
    recognition_thread_->enable();

}


void
ConveyorPoseThread::finalize()
{
  pcl_manager->remove_pointcloud(cloud_out_raw_name_.c_str());
  pcl_manager->remove_pointcloud(cloud_out_trimmed_name_.c_str());
  pcl_manager->remove_pointcloud("model");
  blackboard->close(bb_enable_switch_);
  logger->log_info(name(), "Unloading, disabling %s",
                   cfg_bb_realsense_switch_name_.c_str());
  realsense_switch_->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
  blackboard->close(realsense_switch_);
  blackboard->close(bb_pose_);
}


void
ConveyorPoseThread::loop()
{
  // Check for Messages in ConveyorPoseInterface and update information if needed
  while ( !bb_pose_->msgq_empty() ) {
    if (bb_pose_->msgq_first_is<ConveyorPoseInterface::SetStationMessage>() ) {

      //Update station related information
      logger->log_info(name(), "Received SetStationMessage");
      ConveyorPoseInterface::SetStationMessage *msg =
          bb_pose_->msgq_first<ConveyorPoseInterface::SetStationMessage>();
      update_station_information(*msg);
      bb_pose_->write();

      // Schedule restart of recognition thread
      recognition_thread_->restart();
    }
    else {
      logger->log_warn(name(), "Unknown message received");
    }
    bb_pose_->msgq_pop();
  }

  bb_update_switch();
  realsense_switch_->read();

  if (bb_enable_switch_->is_enabled()) {
    if (realsense_switch_->has_writer()) {
      if (!realsense_switch_->is_enabled()) {
        logger->log_info(name(), "Camera %s is disabled, enabling",
          cfg_bb_realsense_switch_name_.c_str());
        realsense_switch_->msgq_enqueue(
          new SwitchInterface::EnableSwitchMessage());
        start_waiting();
        return;
      }
    } else {
      logger->log_error(name(), "No writer for camera %s",
        cfg_bb_realsense_switch_name_.c_str());
      return;
    }
  }
  else if (realsense_switch_->has_writer()
      && realsense_switch_->is_enabled())
  {
    logger->log_info(name(), "Disabling %s",
      cfg_bb_realsense_switch_name_.c_str());
    realsense_switch_->msgq_enqueue(
      new SwitchInterface::DisableSwitchMessage());
  }

  if (need_to_wait()) {
    logger->log_debug(name(),
      "Waiting for %s for %f sec, still %f sec remaining",
      cfg_bb_realsense_switch_name_.c_str(),
      wait_time_.in_sec(), (wait_start_ + wait_time_ - Time()).in_sec() );
    return;
  }

  if (!bb_enable_switch_->is_enabled()) {
    recognition_thread_->disable();
  }
  else if (update_input_cloud()) {
    fawkes::LaserLineInterface * ll = nullptr;
    have_laser_line_ = laserline_get_best_fit( ll );

    // No point in recording a model when there's no laser line
    if (cfg_record_model_ && !have_laser_line_)
      return;

    if (cloud_mutex_.try_lock()) {
        try {
            if (have_laser_line_) {
                set_initial_tf_from_laserline(ll,current_mps_type_,current_mps_target_);
            }
        } catch (std::exception &e) {
            logger->log_error(name(), "Exception generating initial transform: %s", e.what());
        }
        cloud_mutex_.unlock();
    } // cloud_mutex_.try_lock()

    CloudPtr cloud_in(new Cloud(**cloud_in_));

    size_t in_size = cloud_in->points.size();
    CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
    size_t out_size = cloud_vg->points.size();
    if (in_size == out_size) {
      logger->log_error(name(), "Voxel Grid failed, skipping loop!");
      return;
    }

    trimmed_scene_ = cloud_trim(cloud_vg, ll, have_laser_line_);

    cloud_publish(cloud_in, cloud_out_raw_);
    cloud_publish(trimmed_scene_, cloud_out_trimmed_);


    if (cfg_record_model_) {
      record_model();
      cloud_publish(default_model_, cloud_out_model_);
    }
    else {
      if (bb_mutex_.try_lock()) {
        try {
          if (result_pose_) {
            pose_write();
            pose_publish_tf(*result_pose_);
            result_pose_.release();
          }
        } catch (std::exception &e) {
          logger->log_error(name(), "Unexpected exception: %s", e.what());
        }
        bb_mutex_.unlock();
      }
      if (cloud_mutex_.try_lock()) {
        try {
          scene_ = trimmed_scene_;

          syncpoint_clouds_ready->emit(name());

        } catch (std::exception &e) {
          logger->log_error(name(), "Exception preprocessing point clouds: %s", e.what());
        }
        cloud_mutex_.unlock();
      } // cloud_mutex_.try_lock()
    } // ! cfg_record_model_
  } // update_input_cloud()
}

void
ConveyorPoseThread::set_initial_tf_from_laserline(fawkes::LaserLineInterface *line, ConveyorPoseInterface::MPS_TYPE mps_type, ConveyorPoseInterface::MPS_TARGET mps_target)
{
  tf::Stamped<tf::Pose> initial_guess;

  // Vector from end point 1 to end point 2
  if (!tf_listener->transform_origin(line->end_point_frame_2(), line->end_point_frame_1(),
                                     initial_guess, Time(0,0)))
    throw fawkes::Exception("Failed to transform from %s to %s",
                            line->end_point_frame_2(), line->end_point_frame_1());

  // Halve that to get line center
  initial_guess.setOrigin(initial_guess.getOrigin() / 2);

  // Add distance from center to mps_target
  initial_guess.setOrigin(initial_guess.getOrigin() + tf::Vector3 {
                            double(cfg_target_hint_[mps_target][0]),
                            double(cfg_target_hint_[mps_target][1]),
                            double(cfg_target_hint_[mps_target][2])});


  // Add distance offset for station type
  initial_guess.setOrigin(initial_guess.getOrigin() + tf::Vector3 {
                            double(cfg_type_hint_[mps_type][0]),
                            double(cfg_type_hint_[mps_type][1]),
                            double(cfg_type_hint_[mps_type][2])});


  initial_guess.setRotation(
        tf::Quaternion(tf::Vector3(0,1,0), -M_PI/2)
      * tf::Quaternion(tf::Vector3(0,0,1), M_PI/2));

  // Transform with Time(0,0) to just get the latest transform
  tf_listener->transform_pose(
        "odom",
        tf::Stamped<tf::Pose>(initial_guess, Time(0,0), initial_guess.frame_id),
        initial_guess_laser_odom_);
}


void
ConveyorPoseThread::record_model()
{
  // Transform model
  tf::Stamped<tf::Pose> pose_cam;
  if (!tf_listener->can_transform(cloud_in_->header.frame_id, cfg_model_origin_frame_,
                                  fawkes::Time(0,0))) {
    logger->log_error(name(), "Cannot transform from %s to %s",
                      cloud_in_->header.frame_id.c_str(), cfg_model_origin_frame_.c_str());
    return;
  }
  tf_listener->transform_origin(cloud_in_->header.frame_id, cfg_model_origin_frame_, pose_cam);
  Eigen::Matrix4f tf_to_cam = pose_to_eigen(pose_cam);
  pcl::transformPointCloud(*trimmed_scene_, *default_model_, tf_to_cam);

  // Overwrite and atomically rename model so it can be copied at any time
  try {
    int rv = pcl::io::savePCDFileASCII(cfg_record_path_, *default_model_);
    if (rv)
      logger->log_error(name(), "Error %d saving point cloud to %s", rv, cfg_record_path_.c_str());
    else
      ::rename((cfg_record_path_ + ".pcd").c_str(), cfg_record_path_.c_str());
  } catch (pcl::IOException &e) {
    logger->log_error(name(), "Exception saving point cloud to %s: %s", cfg_default_model_path_.c_str(), e.what());
  }
}


void
ConveyorPoseThread::update_station_information(ConveyorPoseInterface::SetStationMessage &msg)
{
  ConveyorPoseInterface::MPS_TYPE mps_type = msg.mps_type_to_set();
  ConveyorPoseInterface::MPS_TARGET mps_target = msg.mps_target_to_set();

  auto map_it = type_target_to_model_.find({mps_type,mps_target});
  if ( map_it == type_target_to_model_.end())
    logger->log_error(name(), "Invalid station type or target: %i,%i", mps_type, mps_target);
  else {
    logger->log_info(name(), "Setting Station to %s, %s",
                     bb_pose_->enum_tostring("MPS_TYPE", mps_type),
                     bb_pose_->enum_tostring("MPS_TARGET", mps_target));

    MutexLocker locked2(&bb_mutex_);

    model_ = map_it->second;
    current_mps_type_ = mps_type;
    current_mps_target_ = mps_target;

    bb_pose_->set_current_mps_type(mps_type);
    bb_pose_->set_current_mps_target(mps_target);
    result_fitness_ = std::numeric_limits<double>::min();
    bb_pose_->set_euclidean_fitness(result_fitness_);
    bb_pose_->set_msgid(msg.id());
    bb_pose_->write();

    result_fitness_ = std::numeric_limits<double>::min();
  }
}

bool
ConveyorPoseThread::update_input_cloud()
{
  if (pcl_manager->exists_pointcloud(cloud_in_name_.c_str())) {
    if ( ! cloud_in_registered_) {                                             // do I already have this pc
      cloud_in_ = pcl_manager->get_pointcloud<Point>(cloud_in_name_.c_str());
      if (cloud_in_->points.size() > 0) {
        cloud_in_registered_ = true;
      }
    }

    unsigned long time_old = header_.stamp;
    header_ = cloud_in_->header;

    return time_old != header_.stamp; // true, if there is a new cloud, false otherwise

  } else {
    logger->log_debug(name(), "can't get pointcloud %s", cloud_in_name_.c_str());
    cloud_in_registered_ = false;
    return false;
  }
}


void
ConveyorPoseThread::bb_update_switch()
{
  // enable switch
  bb_enable_switch_->read();

  bool rv = bb_enable_switch_->is_enabled();
  while ( ! bb_enable_switch_->msgq_empty() ) {
    if (bb_enable_switch_->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
      logger->log_info(name(),"Received DisableSwitchMessage");
      rv = false;
      { MutexLocker locked(&bb_mutex_);
        result_fitness_ = std::numeric_limits<double>::min();
        bb_pose_->set_euclidean_fitness(result_fitness_);
        bb_pose_->write();
      }
    } else if (bb_enable_switch_->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
      logger->log_info(name(),"Received EnableSwitchMessage");
      rv = true;
    }

    bb_enable_switch_->msgq_pop();
  }
  if ( rv != bb_enable_switch_->is_enabled() ) {
    if ( ! cfg_debug_mode_ ) {
      logger->log_info(name(), "*** enabled: %s", rv ? "yes" : "no");
      bb_enable_switch_->set_enabled(rv);
    } else {
      logger->log_warn(name(), "*** enabled: %s, ignored because of DEBUG MODE, if will be ENABLED", rv ? "yes" : "no");
      bb_enable_switch_->set_enabled(true);
    }
    bb_enable_switch_->write();
  }

  // laser lines
  for (fawkes::LaserLineInterface * ll : laserlines_) {
    ll->read();
  }
}



bool
ConveyorPoseThread::laserline_get_best_fit(fawkes::LaserLineInterface * &best_fit)
{
  best_fit = laserlines_.front();

  // get best line
  for (fawkes::LaserLineInterface * ll : laserlines_) {
    if ( ! ll->has_writer() )
      continue;
    if ( ll->visibility_history() <= 2 )
      continue;

    try {
      Eigen::Vector3f center = laserline_get_center_transformed(ll);
      if ( std::sqrt( center(0) * center(0) + center(2) * center(2) ) > 0.8f )
        continue;

      // take with lowest angle
      if ( fabs(best_fit->bearing()) > fabs(ll->bearing()) )
        best_fit = ll;
    } catch (fawkes::tf::TransformException &e) {
      logger->log_error(name(), e);
    }
  }

  if ( ! best_fit->has_writer() ) {
    logger->log_info(name(), "no writer for laser lines");
    best_fit = nullptr;
    return false;
  }
  if ( best_fit->visibility_history() <= 2 ) {
    best_fit = nullptr;
    return false;
  }
  if ( fabs(best_fit->bearing()) > 0.35f ) {
    best_fit = nullptr;
    return false; // ~20 deg
  }

  try {
    Eigen::Vector3f center = laserline_get_center_transformed(best_fit);
    if ( std::sqrt( center(0) * center(0) + center(2) * center(2) ) > 0.8f ) {
      best_fit = nullptr;
      return false;
    }
  } catch (tf::TransformException &e) {
    logger->log_error(name(), e);
    return false;
  }

  return true;
}


Eigen::Vector3f
ConveyorPoseThread::laserline_get_center_transformed(fawkes::LaserLineInterface * ll)
{
  fawkes::tf::Stamped<fawkes::tf::Point> tf_in, tf_out;
  tf_in.stamp = ll->timestamp();
  tf_in.frame_id = ll->frame_id();
  tf_in.setX( ll->end_point_2(0) + ( ll->end_point_1(0) - ll->end_point_2(0) ) / 2. );
  tf_in.setY( ll->end_point_2(1) + ( ll->end_point_1(1) - ll->end_point_2(1) ) / 2. );
  tf_in.setZ( ll->end_point_2(2) + ( ll->end_point_1(2) - ll->end_point_2(2) ) / 2. );

  try {
    tf_listener->transform_point(header_.frame_id, tf_in, tf_out);
  } catch (tf::ExtrapolationException &) {
    tf_in.stamp = Time(0,0);
    tf_listener->transform_point(header_.frame_id, tf_in, tf_out);
  }

  Eigen::Vector3f out( tf_out.getX(), tf_out.getY(), tf_out.getZ() );

  return out;
}


bool
ConveyorPoseThread::is_inbetween(double a, double b, double val) {
  double low = std::min(a, b);
  double up  = std::max(a, b);

  return val >= low && val <= up;
}


CloudPtr
ConveyorPoseThread::cloud_voxel_grid(CloudPtr in)
{
  float ls = cfg_voxel_grid_leaf_size_;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
  CloudPtr out (new Cloud);
  vg.setInputCloud (in);
 // logger->log_debug(name(), "voxel leaf size is %f", ls);
  vg.setLeafSize (ls, ls, ls);
  vg.filter (*out);
  out->header = in->header;
  return out;
}

void
ConveyorPoseThread::cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out)
{
  **cloud_out = *cloud_in;
  cloud_out->header = header_;
}


void
ConveyorPoseThread::pose_write()
{
  if (!result_pose_) {
    logger->log_error(name(), "BUG: calling pose_write() when result_pose_ is unset!");
    return;
  }
  bb_pose_->set_translation(0, result_pose_->getOrigin().getX());
  bb_pose_->set_translation(1, result_pose_->getOrigin().getY());
  bb_pose_->set_translation(2, result_pose_->getOrigin().getZ());
  bb_pose_->set_rotation(0, result_pose_->getRotation().getX());
  bb_pose_->set_rotation(1, result_pose_->getRotation().getY());
  bb_pose_->set_rotation(2, result_pose_->getRotation().getZ());
  bb_pose_->set_rotation(3, result_pose_->getRotation().getW());

  bb_pose_->set_frame(result_pose_->frame_id.c_str());
  bb_pose_->set_euclidean_fitness(result_fitness_);
  long timestamp[2];
  result_pose_->stamp.get_timestamp(timestamp[0], timestamp[1]);
  bb_pose_->set_input_timestamp(timestamp);
  bb_pose_->write();
}


void
ConveyorPoseThread::pose_publish_tf(const tf::Pose &pose)
{
  // transform data into gripper frame (this is better for later use)
  tf::Stamped<tf::Pose> tf_pose_cam, tf_pose_gripper;
  tf_pose_cam.stamp = fawkes::Time((long)header_.stamp / 1000);
  tf_pose_cam.frame_id = header_.frame_id;
  tf_pose_cam.setOrigin(pose.getOrigin());
  tf_pose_cam.setRotation(pose.getRotation());
  tf_listener->transform_pose("gripper", tf_pose_cam, tf_pose_gripper);

  // publish the transform from the gripper to the conveyor
  tf::Transform transform(
                  tf_pose_gripper.getRotation(),
                  tf_pose_gripper.getOrigin()
                );
  tf::StampedTransform stamped_transform(transform, tf_pose_gripper.stamp, tf_pose_gripper.frame_id, conveyor_frame_id_);
  tf_publisher->send_transform(stamped_transform);
}


void
ConveyorPoseThread::start_waiting()
{
  wait_start_ = Time();
}


bool
ConveyorPoseThread::need_to_wait()
{
  return Time() < wait_start_ + wait_time_;
}


void
ConveyorPoseThread::set_cg_thread(RecognitionThread *cg_thread)
{ recognition_thread_ = cg_thread; }


void ConveyorPoseThread::config_value_erased(const char *path)
{}

void ConveyorPoseThread::config_tag_changed(const char *new_tag)
{}

void ConveyorPoseThread::config_comment_changed(const Configuration::ValueIterator *v)
{}


void ConveyorPoseThread::config_value_changed(const Configuration::ValueIterator *v) {
  if (v->valid()) {
    std::string path = v->path();
    std::string sufx = path.substr(strlen(CFG_PREFIX));
    std::string sub_prefix = sufx.substr(0, sufx.substr(1).find("/")+1);
    std::string full_pfx = CFG_PREFIX + sub_prefix;
    std::string opt = path.substr(full_pfx.length());

    if (sub_prefix == "/gripper") {
      if (opt == "/bottom")
        change_val(opt, cfg_gripper_bottom_, v->get_float());
    } else if(sub_prefix == "/without_ll") {
      if (opt == "/left_cut")
        change_val(opt, cfg_left_cut_no_ll_, v->get_float());
      else if(opt == "/right_cut")
        change_val(opt, cfg_right_cut_no_ll_, v->get_float());
      else if(opt == "/top_cut")
        change_val(opt, cfg_top_cut_no_ll_, v->get_float());
      else if(opt == "/bottom_cut")
        change_val(opt, cfg_bottom_cut_no_ll_, v->get_float());
      else if(opt == "/front_cut")
        change_val(opt, cfg_front_cut_no_ll_, v->get_float());
      else if(opt == "/back_cut")
        change_val(opt, cfg_back_cut_no_ll_, v->get_float());
    } else if(sub_prefix == "/with_ll") {
      if (opt == "/left_cut")
        change_val(opt, cfg_left_cut_, v->get_float());
      else if(opt == "/right_cut")
        change_val(opt, cfg_right_cut_, v->get_float());
      else if(opt == "/top_cut")
        change_val(opt, cfg_top_cut_, v->get_float());
      else if(opt == "/bottom_cut")
        change_val(opt, cfg_bottom_cut_, v->get_float());
      else if(opt == "/front_cut")
        change_val(opt, cfg_front_cut_, v->get_float());
      else if(opt == "/back_cut")
        change_val(opt, cfg_back_cut_, v->get_float());
    } else if (sub_prefix == "/icp") {
      if (opt == "/max_correspondence_dist")
        change_val(opt, recognition_thread_->cfg_icp_max_corr_dist_, v->get_float());
      else if (opt == "/transformation_epsilon")
        change_val(opt, recognition_thread_->cfg_icp_tf_epsilon_, double(v->get_float()));
      else if (opt == "/refinement_factor")
        change_val(opt, recognition_thread_->cfg_icp_refinement_factor_, double(v->get_float()));
      else if (opt == "/max_iterations")
        change_val(opt, recognition_thread_->cfg_icp_max_iterations_, v->get_int());
      else if (opt == "/hv_penalty_threshold")
        change_val(opt, recognition_thread_->cfg_icp_hv_penalty_thresh_, v->get_float());
      else if (opt == "/hv_support_threshold")
        change_val(opt, recognition_thread_->cfg_icp_hv_support_thresh_, v->get_float());
      else if (opt == "/hv_inlier_threshold")
        change_val(opt, recognition_thread_->cfg_icp_hv_inlier_thresh_, v->get_float());
      else if (opt == "/min_loops")
        change_val(opt, recognition_thread_->cfg_icp_min_loops_, v->get_uint());
      else if (opt == "/max_loops")
        change_val(opt, recognition_thread_->cfg_icp_max_loops_, v->get_uint());
      else if (opt == "/auto_restart")
        change_val(opt, recognition_thread_->cfg_icp_auto_restart_, v->get_bool());
      else if (opt == "/hint/conveyor/x")
      {
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][0], v->get_float());
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][0], v->get_float());
      }
      else if (opt == "/hint/conveyor/y")
      {
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][1], v->get_float());
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][1], -(v->get_float()));
      }
      else if (opt == "/hint/conveyor/z")
      {
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][2], v->get_float());
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][2], v->get_float());
      }
      else if (opt == "/hint/left_shelf/x")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][0], v->get_float());
      else if (opt == "/hint/left_shelf/y")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][1], v->get_float());
      else if (opt == "/hint/left_shelf/z")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][2], v->get_float());
      else if (opt == "/hint/middle_shelf/x")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][0], v->get_float());
      else if (opt == "/hint/middle_shelf/y")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][1], v->get_float());
      else if (opt == "/hint/middle_shelf/z")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][2], v->get_float());
      else if (opt == "/hint/right_shelf/x")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][0], v->get_float());
      else if (opt == "/hint/right_shelf/y")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][1], v->get_float());
      else if (opt == "/hint/right_shelf/z")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][2], v->get_float());
      else if (opt == "/hint/slide/x")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SLIDE][0], v->get_float());
      else if (opt == "/hint/slide/y")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SLIDE][1], v->get_float());
      else if (opt == "/hint/slide/z")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SLIDE][2], v->get_float());
      else if (opt == "/hint/default/x")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION][0], v->get_float());
      else if (opt == "/hint/default/y")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION][1], v->get_float());
      else if (opt == "/hint/default/z")
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION][2], v->get_float());

      else if (opt == "/mps_type_offset/base_station/x")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::BASE_STATION][0], v->get_float());
      else if (opt == "/mps_type_offset/base_station/y")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::BASE_STATION][1], v->get_float());
      else if (opt == "/mps_type_offset/base_station/z")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::BASE_STATION][2], v->get_float());

      else if (opt == "/mps_type_offset/cap_station/x")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::CAP_STATION][0], v->get_float());
      else if (opt == "/mps_type_offset/cap_station/y")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::CAP_STATION][1], v->get_float());
      else if (opt == "/mps_type_offset/cap_station/z")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::CAP_STATION][2], v->get_float());

      else if (opt == "/mps_type_offset/ring_station/x")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::RING_STATION][0], v->get_float());
      else if (opt == "/mps_type_offset/ring_station/y")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::RING_STATION][1], v->get_float());
      else if (opt == "/mps_type_offset/ring_station/z")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::RING_STATION][2], v->get_float());

      else if (opt == "/mps_type_offset/delivery_station/x")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION][0], v->get_float());
      else if (opt == "/mps_type_offset/delivery_station/y")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION][1], v->get_float());
      else if (opt == "/mps_type_offset/delivery_station/z")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::DELIVERY_STATION][2], v->get_float());

      else if (opt == "/mps_type_offset/storage_station/x")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION][0], v->get_float());
      else if (opt == "/mps_type_offset/storage_station/y")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION][1], v->get_float());
      else if (opt == "/mps_type_offset/storage_station/z")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::STORAGE_STATION][2], v->get_float());

      else if (opt == "/mps_type_offset/default/x")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::NO_STATION][0], v->get_float());
      else if (opt == "/mps_type_offset/default/y")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::NO_STATION][1], v->get_float());
      else if (opt == "/mps_type_offset/default/z")
        change_val(opt, cfg_type_hint_[ConveyorPoseInterface::NO_STATION][2], v->get_float());
    }
    if (recognition_thread_->enabled())
      recognition_thread_->restart();
  }
}


void ConveyorPoseThread::bb_set_busy(bool busy)
{
  bb_pose_->set_busy(busy);
  bb_pose_->write();
}


float ConveyorPoseThread::cloud_resolution() const
{ return cfg_voxel_grid_leaf_size_; }


Eigen::Matrix4f pose_to_eigen(const fawkes::tf::Pose &pose)
{
  const tf::Matrix3x3 &rot = pose.getBasis();
  const tf::Vector3 &trans = pose.getOrigin();
  Eigen::Matrix4f rv(Eigen::Matrix4f::Identity());
  rv.block<3,3>(0,0)
      << float(rot[0][0]), float(rot[0][1]), float(rot[0][2]),
         float(rot[1][0]), float(rot[1][1]), float(rot[1][2]),
         float(rot[2][0]), float(rot[2][1]), float(rot[2][2]);
  rv.block<3,1>(0,3) << float(trans[0]), float(trans[1]), float(trans[2]);
  return rv;
}


fawkes::tf::Pose eigen_to_pose(const Eigen::Matrix4f &m)
{
  fawkes::tf::Pose rv;
  rv.setOrigin( { double(m(0,3)), double(m(1,3)), double(m(2,3)) } );
  rv.setBasis( {
                 double(m(0,0)), double(m(0,1)), double(m(0,2)),
                 double(m(1,0)), double(m(1,1)), double(m(1,2)),
                 double(m(2,0)), double(m(2,1)), double(m(2,2))
               } );
  return rv;
}

/*
 * This function trims the scene such that the gripper is not visible anymore.
 * Further the scene is cut down to the center, so the amount of points to correspond
 * becomes smaller.
 * Last but not least the background (e.g. cables) is cut away, making the model more general.
 *
 * The trimming happens inside the frame of the conveyor cam frame.
 *
 * From the point of the cam the coordinate system is like:
 *             z
 *            /
 *          /
 *        /
 *       *-------x
 *       |
 *       |
 *       |
 *       y
 * */
CloudPtr ConveyorPoseThread::cloud_trim(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll) {
    float x_min = -FLT_MAX, x_max = FLT_MAX, 
           y_min = -FLT_MAX, y_max = FLT_MAX, 
           z_min = -FLT_MAX, z_max = FLT_MAX;

    y_min = std::max((float) cfg_gripper_bottom_, y_min); // only points below the gripper

    if(use_ll){
        // get position of initial guess in conveyor cam frame
        // rotation is ignored, since rotation values are small
        tf::Stamped<tf::Pose> initial_guess_laser;
        tf_listener->transform_pose(in->header.frame_id,
                tf::Stamped<tf::Pose>(initial_guess_laser_odom_, Time(0,0), initial_guess_laser_odom_.frame_id),
                initial_guess_laser);
        float x_ini = initial_guess_laser.getOrigin()[0],
               y_ini = initial_guess_laser.getOrigin()[1],
               z_ini = initial_guess_laser.getOrigin()[2];

        x_min = std::max(x_ini + (float) cfg_left_cut_, x_min);
        x_max = std::min(x_ini + (float) cfg_right_cut_, x_max);

        y_min = std::max(y_ini + (float) cfg_top_cut_, y_min);
        y_max = std::min(y_ini + (float) cfg_bottom_cut_, y_max);

        z_min = std::max(z_ini + (float) cfg_front_cut_, z_min);
        z_max = std::min(z_ini + (float) cfg_back_cut_, z_max);

    } else { // there is no laser line matching tolerances, thus no initial tf guess
        logger->log_info(name(), "--------------STOPPED USING LASERLINE-----------");

        x_min = std::max((float) cfg_left_cut_no_ll_, x_min);
        x_max = std::min((float) cfg_right_cut_no_ll_, x_max);

        y_min = std::max((float) cfg_top_cut_no_ll_, y_min);
        y_max = std::min((float) cfg_bottom_cut_no_ll_, y_max);

        z_min = std::max((float) cfg_front_cut_no_ll_, z_min);
        z_max = std::min((float) cfg_back_cut_no_ll_, z_max);
    }

    CloudPtr out(new Cloud);
    for(Point p: *in) {
        if(    p.x < x_max && p.x > x_min
           &&  p.y < y_max && p.y > y_min
           &&  p.z < z_max && p.z > z_min) {
            out->push_back(p);
        }
    }

    out->header = in->header;
    return out;
}
