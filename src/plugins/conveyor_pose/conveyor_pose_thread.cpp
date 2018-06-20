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


std::string
ConveyorPoseThread::get_model_path(ConveyorPoseInterface *iface, ConveyorPoseInterface::MPS_TYPE type, int id, ConveyorPoseInterface::MPS_TARGET target)
{
  std::string path = std::string(CFG_PREFIX "/reference_models/")
      + iface->enum_tostring("MPS_TYPE", type) + "_" + std::to_string(id) + "_" + iface->enum_tostring("MPS_TARGET", target);
  if (config->exists(path)) {
    logger->log_info(name(), "Override for %s_%d_%s: %s",
                     iface->enum_tostring("MPS_TYPE", type),
                     id,
                     iface->enum_tostring("MPS_TARGET", target),
                     config->get_string(path).c_str());
    return CONFDIR "/" + config->get_string(path);
  }
  else {
    path = std::string(CFG_PREFIX "/reference_models/")
        + iface->enum_tostring("MPS_TYPE", type) + "_" + iface->enum_tostring("MPS_TARGET", target);
    if (config->exists(path)) {
      logger->log_info(name(), "Override for %s_%s: %s",
          iface->enum_tostring("MPS_TYPE", type),
          iface->enum_tostring("MPS_TARGET", target),
          config->get_string(path).c_str());
      return CONFDIR "/" + config->get_string(path);
    }
    else {
      switch(target) {
        case ConveyorPoseInterface::INPUT_CONVEYOR:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/input_conveyor");
        case ConveyorPoseInterface::OUTPUT_CONVEYOR:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/output_conveyor");
        case ConveyorPoseInterface::SHELF_LEFT:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/shelf_left");
        case ConveyorPoseInterface::SHELF_MIDDLE:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/shelf_middle");
        case ConveyorPoseInterface::SHELF_RIGHT:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/shelf_right");
        case ConveyorPoseInterface::SLIDE:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/slide");
        case ConveyorPoseInterface::NO_LOCATION:
          return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/default");
        case ConveyorPoseInterface::LAST_MPS_TARGET_ELEMENT:
          return "";
      }
    }
  }

  return "";
}


void
ConveyorPoseThread::init()
{
  config->add_change_handler(this);

  syncpoint_clouds_ready = syncpoint_manager->get_syncpoint(name(), syncpoint_clouds_ready_name);
  syncpoint_clouds_ready->register_emitter(name());

  cfg_debug_mode_ = config->get_bool( CFG_PREFIX "/debug" );
  cfg_force_shelf_ = config->get_int_or_default( CFG_PREFIX "/force_shelf", -1);
  cloud_in_name_ = config->get_string( CFG_PREFIX "/cloud_in" );

  cfg_if_prefix_ = config->get_string( CFG_PREFIX "/if/prefix" );
  if (cfg_if_prefix_.back() != '/')
    cfg_if_prefix_.append("/");


  laserlines_names_       = config->get_strings( CFG_PREFIX "/if/laser_lines" );

  conveyor_frame_id_          = config->get_string( CFG_PREFIX "/conveyor_frame_id" );

  recognition_thread_->cfg_icp_rotation_threshold_ = config->get_float_or_default( CFG_PREFIX "/icp/rotation_threshold", 15.0 );
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

  recognition_thread_->cfg_icp_shelf_hv_inlier_thresh_  = config->get_float( CFG_PREFIX "/icp/hv_shelf_inlier_threshold" );
  recognition_thread_->cfg_icp_shelf_hv_penalty_thresh_ = config->get_float( CFG_PREFIX "/icp/hv_shelf_penalty_threshold" );
  recognition_thread_->cfg_icp_shelf_hv_support_thresh_ = config->get_float( CFG_PREFIX "/icp/hv_shelf_support_threshold" );

  bb_pose_ = blackboard->open_for_writing<ConveyorPoseInterface>((cfg_if_prefix_ + "status").c_str());
  bb_pose_->set_current_mps_type(bb_pose_->NO_STATION);
  bb_pose_->set_current_mps_target(bb_pose_->NO_LOCATION);

  bb_pose_->write();

  // Init of station target hints
  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR];
  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR];
  cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT];
  cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT];
  cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE];
  cfg_target_hint_[ConveyorPoseInterface::SLIDE];
  cfg_target_hint_[ConveyorPoseInterface::NO_LOCATION];

  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][0]    = config->get_float( CFG_PREFIX "/icp/hint/input_conveyor/x" );
  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][1]    = config->get_float( CFG_PREFIX "/icp/hint/input_conveyor/y" );
  cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][2]    = config->get_float( CFG_PREFIX "/icp/hint/input_conveyor/z" );

  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][0]   = config->get_float( CFG_PREFIX "/icp/hint/input_conveyor/x");
  // Y * -1, because it should be the opposite of the input conveyor
  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][1]   = -config->get_float( CFG_PREFIX "/icp/hint/input_conveyor/y");
  cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][2]   = config->get_float( CFG_PREFIX "/icp/hint/input_conveyor/z");

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

  cfg_type_offset_[ConveyorPoseInterface::BASE_STATION];
  cfg_type_offset_[ConveyorPoseInterface::CAP_STATION];
  cfg_type_offset_[ConveyorPoseInterface::RING_STATION];
  cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION];
  cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION];

  cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/base_station/x", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/base_station/y", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/base_station/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/cap_station/x", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/cap_station/y", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/cap_station/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::RING_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/ring_station/x", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::RING_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/ring_station/y", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::RING_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/ring_station/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/delivery_station/x", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/delivery_station/y", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/delivery_station/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/storage_station/x", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/storage_station/y", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/storage_station/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::NO_STATION][0] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/default/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::NO_STATION][1] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/default/z", 0.f );
  cfg_type_offset_[ConveyorPoseInterface::NO_STATION][2] = config->get_float_or_default(
        CFG_PREFIX "/icp/conveyor_offset/default/z", 0.f );


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

  cfg_shelf_left_cut_no_ll_         = config->get_float( CFG_PREFIX "/shelf/without_ll/left_cut" );
  cfg_shelf_right_cut_no_ll_        = config->get_float( CFG_PREFIX "/shelf/without_ll/right_cut" );
  cfg_shelf_top_cut_no_ll_          = config->get_float( CFG_PREFIX "/shelf/without_ll/top_cut" );
  cfg_shelf_bottom_cut_no_ll_       = config->get_float( CFG_PREFIX "/shelf/without_ll/bottom_cut" );
  cfg_shelf_front_cut_no_ll_        = config->get_float( CFG_PREFIX "/shelf/without_ll/front_cut" );
  cfg_shelf_back_cut_no_ll_         = config->get_float( CFG_PREFIX "/shelf/without_ll/back_cut" );

  cfg_shelf_left_cut_               = config->get_float( CFG_PREFIX "/shelf/with_ll/left_cut" );
  cfg_shelf_right_cut_              = config->get_float( CFG_PREFIX "/shelf/with_ll/right_cut" );
  cfg_shelf_top_cut_                = config->get_float( CFG_PREFIX "/shelf/with_ll/top_cut" );
  cfg_shelf_bottom_cut_             = config->get_float( CFG_PREFIX "/shelf/with_ll/bottom_cut" );
  cfg_shelf_front_cut_              = config->get_float( CFG_PREFIX "/shelf/with_ll/front_cut" );
  cfg_shelf_back_cut_               = config->get_float( CFG_PREFIX "/shelf/with_ll/back_cut" );

  cfg_shelf_left_off_               = config->get_float( CFG_PREFIX "/shelf/left_off" );
  cfg_shelf_middle_off_             = config->get_float( CFG_PREFIX "/shelf/middle_off" );
  cfg_shelf_right_off_              = config->get_float( CFG_PREFIX "/shelf/right_off" );


  cfg_voxel_grid_leaf_size_  = config->get_float( CFG_PREFIX "/voxel_grid/leaf_size" );

  cfg_bb_realsense_switch_name_ = config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");
  wait_time_ = Time(double(config->get_float_or_default(CFG_PREFIX "/realsense_wait_time", 1.0f)));

  // Load reference pcl for shelf, input belt (with cone for input -> output machines, without cone for output <- -> output machines), output belt (without cone) and slide
  for ( int i = ConveyorPoseInterface::NO_STATION; i != ConveyorPoseInterface::LAST_MPS_TYPE_ELEMENT; i++ )
  {
    ConveyorPoseInterface::MPS_TYPE mps_type = static_cast<ConveyorPoseInterface::MPS_TYPE>(i);
    for(int id = 0; id < 5; id++)
    {

      for (int j = ConveyorPoseInterface::NO_LOCATION; j != ConveyorPoseInterface::LAST_MPS_TARGET_ELEMENT; j++)
      {
        ConveyorPoseInterface::MPS_TARGET mps_target = static_cast<ConveyorPoseInterface::MPS_TARGET>(j);

        type_id_target_to_path_[{mps_type, id, mps_target}] = get_model_path(bb_pose_, mps_type, id, mps_target);
      }
    }
  }

  // always use the output_conveyor model for Storage station and Base station
  for(int id = 0; id <5; id++)
  {
    //BASE STATION
    auto map_it = type_id_target_to_model_.find({ConveyorPoseInterface::BASE_STATION,id,ConveyorPoseInterface::INPUT_CONVEYOR});
    if ( map_it != type_id_target_to_model_.end())
    {
      type_id_target_to_path_[{ConveyorPoseInterface::BASE_STATION, id, ConveyorPoseInterface::INPUT_CONVEYOR}]=
        type_id_target_to_path_[{ConveyorPoseInterface::BASE_STATION, id, ConveyorPoseInterface::OUTPUT_CONVEYOR}];
    }

    //STORAGE_STATION
    map_it = type_id_target_to_model_.find({ConveyorPoseInterface::STORAGE_STATION,id,ConveyorPoseInterface::INPUT_CONVEYOR});
    if ( map_it != type_id_target_to_model_.end())
    {
      type_id_target_to_path_[{ConveyorPoseInterface::STORAGE_STATION, id, ConveyorPoseInterface::INPUT_CONVEYOR}]=
        type_id_target_to_path_[{ConveyorPoseInterface::STORAGE_STATION, id, ConveyorPoseInterface::OUTPUT_CONVEYOR}];
    }
  }

  trimmed_scene_.reset(new Cloud());

  cfg_model_origin_frame_ = config->get_string(CFG_PREFIX "/model_origin_frame");
  cfg_record_model_ = config->get_bool_or_default(CFG_PREFIX "/record_model", false);

  // if recording is set, a pointcloud written to the the cfg_record_path_
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
    model_.reset(new Cloud());
  }
  else {
    // Loading PCD file and calculation of model with normals for ALL! stations
    for (const auto &pair : type_id_target_to_path_) {
      CloudPtr model(new Cloud());
      if (pcl::io::loadPCDFile(pair.second, *model) < 0)
        throw fawkes::CouldNotOpenFileException(pair.second.c_str());

      type_id_target_to_model_.insert({pair.first, model});
    }

    model_ = type_id_target_to_model_[{ConveyorPoseInterface::NO_STATION, 0, ConveyorPoseInterface::NO_LOCATION}];
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

  realsense_switch_ = blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());

  if (cfg_debug_mode_){
    recognition_thread_->enable();
    switch(cfg_force_shelf_){
      case 0:
        current_mps_target_ = fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_LEFT;
        break;
      case 1:
        current_mps_target_ = fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_MIDDLE;
        break;
      case 2:
        current_mps_target_ = fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_RIGHT;
        break;
      default:
        break;
    }
  }

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
  // Check for Messages in ConveyorPoseInterface and update informations if needed
  while ( !bb_pose_->msgq_empty() ) {
    if (bb_pose_->msgq_first_is<ConveyorPoseInterface::SetStationMessage>() ) {

      //Update station related information
      logger->log_info(name(), "Received SetStationMessage");
      ConveyorPoseInterface::SetStationMessage *msg =
          bb_pose_->msgq_first<ConveyorPoseInterface::SetStationMessage>();
      update_station_information(*msg);
      bb_pose_->write();

      result_pose_.release();

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
    result_pose_.reset();
  }
  else if (update_input_cloud()) {
    fawkes::LaserLineInterface * ll = nullptr;
    have_laser_line_ = laserline_get_best_fit( ll );

    // No point in recording a model when there's no laser line
    if (cfg_record_model_ && !have_laser_line_)
      return;

    { 
        fawkes::MutexLocker locked(&cloud_mutex_);
        try {
            if (have_laser_line_) {
                set_initial_tf_from_laserline(ll,current_mps_type_,current_mps_target_);
            }
        } catch (std::exception &e) {
            logger->log_error(name(), "Exception generating initial transform: %s", e.what());
        }
    } // cloud_mutex_ lock

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
      cloud_publish(model_, cloud_out_model_);

      tf::Stamped<tf::Pose> initial_pose_cam;
      try {
        tf_listener->transform_pose(
            trimmed_scene_->header.frame_id,
            tf::Stamped<tf::Pose>(initial_guess_laser_odom_, Time(0,0), initial_guess_laser_odom_.frame_id),
            initial_pose_cam);
      } catch (tf::TransformException &e) {
        logger->log_error(name(),e);
        return;
      }
      tf_publisher->send_transform(
            tf::StampedTransform(initial_pose_cam, initial_pose_cam.stamp, initial_pose_cam.frame_id, "conveyor_pose_initial_guess"));
    }
    else {
      { MutexLocker locked { &bb_mutex_ };
        try {
          if (result_pose_) {
            pose_write();
            pose_publish_tf(*result_pose_);
            result_pose_.reset();
          }
        } catch (std::exception &e) {
          logger->log_error(name(), "Unexpected exception: %s", e.what());
        }
      }

      { MutexLocker locked { &cloud_mutex_ };

        scene_ = trimmed_scene_;
        syncpoint_clouds_ready->emit(name());
      }
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

  // Add distance from from center to mps_target
  initial_guess.setOrigin(initial_guess.getOrigin() + tf::Vector3 {
                            double(cfg_target_hint_[mps_target][0]),
                            double(cfg_target_hint_[mps_target][1]),
                            double(cfg_target_hint_[mps_target][2])});


  if (mps_target == ConveyorPoseInterface::MPS_TARGET::INPUT_CONVEYOR) {
    // Add distance offset for station type
    initial_guess.setOrigin(initial_guess.getOrigin() + tf::Vector3 {
                              double(cfg_type_offset_[mps_type][0]),
                              double(cfg_type_offset_[mps_type][1]),
                              double(cfg_type_offset_[mps_type][2])});
  } else if (mps_target == ConveyorPoseInterface::MPS_TARGET::OUTPUT_CONVEYOR) {
    // Invert Y axis of the offset
    initial_guess.setOrigin(initial_guess.getOrigin() + tf::Vector3 {
                              double(cfg_type_offset_[mps_type][0]),
                              double(-cfg_type_offset_[mps_type][1]),
                              double(cfg_type_offset_[mps_type][2])});
  }


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
  pcl::transformPointCloud(*trimmed_scene_, *model_, tf_to_cam);

  // Overwrite and atomically rename model so it can be copied at any time
  try {
    int rv = pcl::io::savePCDFileASCII(cfg_record_path_, *model_);
    if (rv)
      logger->log_error(name(), "Error %d saving point cloud to %s", rv, cfg_record_path_.c_str());
    else
      ::rename((cfg_record_path_ + ".pcd").c_str(), cfg_record_path_.c_str());
  } catch (pcl::IOException &e) {
    logger->log_error(name(), "Exception saving point cloud to %s: %s", cfg_record_path_.c_str(), e.what());
  }
}


void
ConveyorPoseThread::update_station_information(ConveyorPoseInterface::SetStationMessage &msg)
{
  ConveyorPoseInterface::MPS_TYPE mps_type = msg.mps_type_to_set();
  ConveyorPoseInterface::MPS_TARGET mps_target = msg.mps_target_to_set();
  int mps_id = msg.mps_id_to_set();

  auto map_it = type_id_target_to_model_.find({mps_type,mps_id,mps_target});
  if ( map_it == type_id_target_to_model_.end())
  {
    //cannot find specific id in cloudptr map, search for general one
    map_it = type_id_target_to_model_.find({mps_type,0,mps_target});
    if ( map_it == type_id_target_to_model_.end()){
      logger->log_error(name(), "Invalid station type or target: %i,%i", mps_type, mps_target);
      return;
    }
    else {
      logger->log_info(name(), "Setting Station to %s-general, %s",
                       bb_pose_->enum_tostring("MPS_TYPE", mps_type),
                       bb_pose_->enum_tostring("MPS_TARGET", mps_target));
      mps_id = 0;
    }
  }
  else {
    logger->log_info(name(), "Setting Station to %s-%d, %s",
                     bb_pose_->enum_tostring("MPS_TYPE", mps_type),
                     mps_id,
                     bb_pose_->enum_tostring("MPS_TARGET", mps_target));
  }

  MutexLocker locked2(&bb_mutex_);

  model_ = map_it->second;
  current_mps_type_ = mps_type;
  current_mps_target_ = mps_target;
  current_mps_id_ = mps_id;

  bb_pose_->set_current_mps_type(mps_type);
  bb_pose_->set_current_mps_id(mps_id);
  bb_pose_->set_current_mps_target(mps_target);
  result_fitness_ = std::numeric_limits<double>::min();
  bb_pose_->set_euclidean_fitness(result_fitness_);
  bb_pose_->set_msgid(msg.id());
  bb_pose_->set_busy(true);
  bb_pose_->write();
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
  best_fit = nullptr;

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
      if (!best_fit || fabs(best_fit->bearing()) > fabs(ll->bearing()) )
        best_fit = ll;
    } catch (fawkes::tf::TransformException &e) {
      logger->log_error(name(), e);
    }
  }

  if ( ! best_fit )
    return false;

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
ConveyorPoseThread::pose_publish_tf(const tf::Stamped<tf::Pose> &pose)
{
  // transform data into gripper frame (this is better for later use)
  tf::Stamped<tf::Pose> tf_pose_gripper;
  tf_listener->transform_pose("gripper", pose, tf_pose_gripper);

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

    if(sub_prefix == "/without_ll") {
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
    } else if(sub_prefix == "/shelf") {
      if (opt == "/without_ll/left_cut")
        change_val(opt, cfg_shelf_left_cut_no_ll_, v->get_float());
      else if(opt == "/without_ll/right_cut")
        change_val(opt, cfg_shelf_right_cut_no_ll_, v->get_float());
      else if(opt == "/without_ll/top_cut")
        change_val(opt, cfg_shelf_top_cut_no_ll_, v->get_float());
      else if(opt == "/without_ll/bottom_cut")
        change_val(opt, cfg_shelf_bottom_cut_no_ll_, v->get_float());
      else if(opt == "/without_ll/front_cut")
        change_val(opt, cfg_shelf_front_cut_no_ll_, v->get_float());
      else if(opt == "/without_ll/back_cut")
        change_val(opt, cfg_shelf_back_cut_no_ll_, v->get_float());
      else if (opt == "/with_ll/left_cut")
        change_val(opt, cfg_shelf_left_cut_, v->get_float());
      else if(opt == "/with_ll/right_cut")
        change_val(opt, cfg_shelf_right_cut_, v->get_float());
      else if(opt == "/with_ll/top_cut")
        change_val(opt, cfg_shelf_top_cut_, v->get_float());
      else if(opt == "/with_ll/bottom_cut")
        change_val(opt, cfg_shelf_bottom_cut_, v->get_float());
      else if(opt == "/with_ll/front_cut")
        change_val(opt, cfg_shelf_front_cut_, v->get_float());
      else if(opt == "/with_ll/back_cut")
        change_val(opt, cfg_shelf_back_cut_, v->get_float());
      else if(opt == "/left_off")
        change_val(opt, cfg_shelf_left_off_, v->get_float());
      else if(opt == "/middle_off")
        change_val(opt, cfg_shelf_middle_off_, v->get_float());
      else if(opt == "/right_off")
        change_val(opt, cfg_shelf_right_off_, v->get_float());
    } else if (sub_prefix == "/icp") {
      if (opt == "/max_correspondence_dist")
        change_val(opt, recognition_thread_->cfg_icp_max_corr_dist_, v->get_float());
      else if (opt == "/rotation_threshold")
        change_val(opt, recognition_thread_->cfg_icp_rotation_threshold_, v->get_float());
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
      else if (opt == "/hv_shelf_penalty_threshold")
        change_val(opt, recognition_thread_->cfg_icp_shelf_hv_penalty_thresh_, v->get_float());
      else if (opt == "/hv_shelf_support_threshold")
        change_val(opt, recognition_thread_->cfg_icp_shelf_hv_support_thresh_, v->get_float());
      else if (opt == "/hv_shelf_inlier_threshold")
        change_val(opt, recognition_thread_->cfg_icp_shelf_hv_inlier_thresh_, v->get_float());
      else if (opt == "/min_loops")
        change_val(opt, recognition_thread_->cfg_icp_min_loops_, v->get_uint());
      else if (opt == "/max_loops")
        change_val(opt, recognition_thread_->cfg_icp_max_loops_, v->get_uint());
      else if (opt == "/auto_restart")
        change_val(opt, recognition_thread_->cfg_icp_auto_restart_, v->get_bool());
      else if (opt == "/hint/input_conveyor/x")
      {
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][0], v->get_float());
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][0], v->get_float());
      }
      else if (opt == "/hint/input_conveyor/y")
      {
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][1], v->get_float());
        change_val(opt, cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][1], -(v->get_float()));
      }
      else if (opt == "/hint/input_conveyor/z")
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

      else if (opt == "/conveyor_offset/base_station/x")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][0], v->get_float());
      else if (opt == "/conveyor_offset/base_station/y")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][1], v->get_float());
      else if (opt == "/conveyor_offset/base_station/z")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][2], v->get_float());

      else if (opt == "/conveyor_offset/cap_station/x")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][0], v->get_float());
      else if (opt == "/conveyor_offset/cap_station/y")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][1], v->get_float());
      else if (opt == "/conveyor_offset/cap_station/z")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][2], v->get_float());

      else if (opt == "/conveyor_offset/ring_station/x")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::RING_STATION][0], v->get_float());
      else if (opt == "/conveyor_offset/ring_station/y")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::RING_STATION][1], v->get_float());
      else if (opt == "/conveyor_offset/ring_station/z")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::RING_STATION][2], v->get_float());

      else if (opt == "/conveyor_offset/delivery_station/x")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][0], v->get_float());
      else if (opt == "/conveyor_offset/delivery_station/y")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][1], v->get_float());
      else if (opt == "/conveyor_offset/delivery_station/z")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][2], v->get_float());

      else if (opt == "/conveyor_offset/storage_station/x")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][0], v->get_float());
      else if (opt == "/conveyor_offset/storage_station/y")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][1], v->get_float());
      else if (opt == "/conveyor_offset/storage_station/z")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][2], v->get_float());

      else if (opt == "/conveyor_offset/default/x")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::NO_STATION][0], v->get_float());
      else if (opt == "/conveyor_offset/default/y")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::NO_STATION][1], v->get_float());
      else if (opt == "/conveyor_offset/default/z")
        change_val(opt, cfg_type_offset_[ConveyorPoseInterface::NO_STATION][2], v->get_float());
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

    if(use_ll){
        // get position of initial guess in conveyor cam frame
        // rotation is ignored, since rotation values are small
      tf::Stamped<tf::Pose> origin_pose;
      if(cfg_record_model_){
        tf_listener->transform_origin(cfg_model_origin_frame_, in->header.frame_id,  origin_pose);
      } else {
        tf_listener->transform_pose(in->header.frame_id,
                tf::Stamped<tf::Pose>(initial_guess_laser_odom_, Time(0,0), initial_guess_laser_odom_.frame_id),
                origin_pose);
      }
        float x_ini = origin_pose.getOrigin()[0],
              y_ini = origin_pose.getOrigin()[1],
              z_ini = origin_pose.getOrigin()[2];

        if(is_target_shelf()){ //using shelf cut values
          float x_min_temp = x_ini + (float) cfg_shelf_left_cut_,
                x_max_temp = x_ini + (float) cfg_shelf_right_cut_;
          switch(current_mps_target_) {
            case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_LEFT:
              x_min_temp += (float) cfg_shelf_left_off_;
              x_max_temp += (float) cfg_shelf_left_off_;
              break;
            case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_MIDDLE:
              x_min_temp += (float) cfg_shelf_middle_off_;
              x_max_temp += (float) cfg_shelf_middle_off_;
              break;
            case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_RIGHT:
              x_min_temp += (float) cfg_shelf_right_off_;
              x_max_temp += (float) cfg_shelf_right_off_;
              break;
            default: //This should not happen
              break;
          }
          x_min = std::max(x_min_temp, x_min);
          x_max = std::min(x_max_temp, x_max);

          y_min = std::max(y_ini + (float) cfg_shelf_top_cut_, y_min);
          y_max = std::min(y_ini + (float) cfg_shelf_bottom_cut_, y_max);

          z_min = std::max(z_ini + (float) cfg_shelf_front_cut_, z_min);
          z_max = std::min(z_ini + (float) cfg_shelf_back_cut_, z_max);
        } else { //using general cut values
          x_min = std::max(x_ini + (float) cfg_left_cut_, x_min);
          x_max = std::min(x_ini + (float) cfg_right_cut_, x_max);

          y_min = std::max(y_ini + (float) cfg_top_cut_, y_min);
          y_max = std::min(y_ini + (float) cfg_bottom_cut_, y_max);

          z_min = std::max(z_ini + (float) cfg_front_cut_, z_min);
          z_max = std::min(z_ini + (float) cfg_back_cut_, z_max);
        }

    } else { //no laser line is equivalent to no usable initial tf guess
        if(is_target_shelf()){ //using shelf cut values
          x_min = std::max((float) cfg_shelf_left_cut_no_ll_, x_min);
          x_max = std::min((float) cfg_shelf_right_cut_no_ll_, x_max);

          y_min = std::max((float) cfg_shelf_top_cut_no_ll_, y_min);
          y_max = std::min((float) cfg_shelf_bottom_cut_no_ll_, y_max);

          z_min = std::max((float) cfg_shelf_front_cut_no_ll_, z_min);
          z_max = std::min((float) cfg_shelf_back_cut_no_ll_, z_max);
        } else { //using general cut values
          x_min = std::max((float) cfg_left_cut_no_ll_, x_min);
          x_max = std::min((float) cfg_right_cut_no_ll_, x_max);

          y_min = std::max((float) cfg_top_cut_no_ll_, y_min);
          y_max = std::min((float) cfg_bottom_cut_no_ll_, y_max);

          z_min = std::max((float) cfg_front_cut_no_ll_, z_min);
          z_max = std::min((float) cfg_back_cut_no_ll_, z_max);
        }

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

bool ConveyorPoseThread::is_target_shelf()
{
    switch (current_mps_target_)
    {
        case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_LEFT:
        case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_MIDDLE:
        case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_RIGHT:
            return true;
        default:
            return false;
    }
}
