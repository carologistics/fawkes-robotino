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
#include "correspondence_grouping_thread.h"

using namespace fawkes;

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
  , cloud_out_raw_name_("raw")
  , cloud_out_trimmed_name_("trimmed")
  , cg_thread_(nullptr)
  , result_pose_(tf::Pose::getIdentity(), Time(0,0), "NOT_INITIALIZED")
  , result_fitness_(std::numeric_limits<double>::min())
  , realsense_switch_(nullptr)
{}

void
ConveyorPoseThread::init()
{
  config->add_change_handler(this);


  cfg_debug_mode_ = config->get_bool( CFG_PREFIX "/debug" );
  cloud_in_name_ = config->get_string( CFG_PREFIX "/cloud_in" );

  cfg_if_prefix_ = config->get_string( CFG_PREFIX "/if/prefix" );
  if (cfg_if_prefix_.back() != '/')
    cfg_if_prefix_.append("/");


  laserlines_names_       = config->get_strings( CFG_PREFIX "/if/laser_lines" );

  conveyor_frame_id_          = config->get_string( CFG_PREFIX "/conveyor_frame_id" );

  cfg_icp_max_corr_dist_      = config->get_float( CFG_PREFIX "/icp/max_correspondence_dist" );
  cfg_conveyor_hint_[0]       = config->get_float( CFG_PREFIX "/icp/conveyor_hint/x" );
  cfg_conveyor_hint_[1]       = config->get_float( CFG_PREFIX "/icp/conveyor_hint/y" );
  cfg_conveyor_hint_[2]       = config->get_float( CFG_PREFIX "/icp/conveyor_hint/z" );

  cfg_enable_switch_          = config->get_bool( CFG_PREFIX "/switch_default" );

  cfg_gripper_y_min_          = config->get_float( CFG_PREFIX "/gripper/y_min" );
  cfg_gripper_y_max_          = config->get_float( CFG_PREFIX "/gripper/y_max" );
  cfg_gripper_z_max_          = config->get_float( CFG_PREFIX "/gripper/z_max" );
  cfg_gripper_slice_y_min_    = config->get_float( CFG_PREFIX "/gripper/slice/y_min" );
  cfg_gripper_slice_y_max_    = config->get_float( CFG_PREFIX "/gripper/slice/y_max" );

  cfg_front_space_            = config->get_float( CFG_PREFIX "/front/space" );
  cfg_front_offset_           = config->get_float( CFG_PREFIX "/front/offset" );

  cfg_left_cut_               = config->get_float( CFG_PREFIX "/left_right/left_cut" );
  cfg_right_cut_              = config->get_float( CFG_PREFIX "/left_right/right_cut" );
  cfg_left_cut_no_ll_         = config->get_float( CFG_PREFIX "/left_right/left_cut_no_ll" );
  cfg_right_cut_no_ll_        = config->get_float( CFG_PREFIX "/left_right/right_cut_no_ll" );

  cfg_voxel_grid_leaf_size_  = config->get_float( CFG_PREFIX "/voxel_grid/leaf_size" );

  cfg_bb_realsense_switch_name_ = config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");
  wait_time_ = Time(double(config->get_float_or_default(CFG_PREFIX "/realsense_wait_time", 1.0f)));


  cfg_model_path_ = config->get_string(CFG_PREFIX "/model_file");
  if (cfg_model_path_.substr(0, 1) != "/")
    cfg_model_path_ = CONFDIR "/" + cfg_model_path_;


  // Load all model paths, should be "default" if the default model path should be used

  station_to_path_.insert({"M-BS-I", config->get_string(CFG_PREFIX  "/model_files/M-BS-I")});
  station_to_path_.insert({"M-BS-O", config->get_string(CFG_PREFIX  "/model_files/M-BS-O")});
  station_to_path_.insert({"C-BS-I", config->get_string(CFG_PREFIX  "/model_files/C-BS-I")});
  station_to_path_.insert({"C-BS-O", config->get_string(CFG_PREFIX  "/model_files/C-BS-O")});
  station_to_path_.insert({"M-CS1-I", config->get_string(CFG_PREFIX "/model_files/M-CS1-I")});
  station_to_path_.insert({"M-CS1-O", config->get_string(CFG_PREFIX "/model_files/M-CS1-O")});
  station_to_path_.insert({"C-CS1-I", config->get_string(CFG_PREFIX "/model_files/C-CS1-I")});
  station_to_path_.insert({"C-CS1-O", config->get_string(CFG_PREFIX "/model_files/C-CS1-O")});
  station_to_path_.insert({"M-CS2-I", config->get_string(CFG_PREFIX "/model_files/M-CS2-I")});
  station_to_path_.insert({"M-CS2-O", config->get_string(CFG_PREFIX "/model_files/M-CS2-O")});
  station_to_path_.insert({"C-CS2-I", config->get_string(CFG_PREFIX "/model_files/C-CS2-I")});
  station_to_path_.insert({"C-CS2-O", config->get_string(CFG_PREFIX "/model_files/C-CS2-O")});
  station_to_path_.insert({"M-RS1-I", config->get_string(CFG_PREFIX "/model_files/M-RS1-I")});
  station_to_path_.insert({"M-RS1-O", config->get_string(CFG_PREFIX "/model_files/M-RS1-O")});
  station_to_path_.insert({"C-RS1-I", config->get_string(CFG_PREFIX "/model_files/C-RS1-I")});
  station_to_path_.insert({"C-RS1-O", config->get_string(CFG_PREFIX "/model_files/C-RS1-O")});
  station_to_path_.insert({"M-RS2-I", config->get_string(CFG_PREFIX "/model_files/M-RS2-I")});
  station_to_path_.insert({"M-RS2-O", config->get_string(CFG_PREFIX "/model_files/M-RS2-O")});
  station_to_path_.insert({"C-RS2-I", config->get_string(CFG_PREFIX "/model_files/C-RS2-I")});
  station_to_path_.insert({"C-RS2-O", config->get_string(CFG_PREFIX "/model_files/C-RS2-O")});
  station_to_path_.insert({"M-DS-I", config->get_string(CFG_PREFIX  "/model_files/M-DS-I")});
  station_to_path_.insert({"C-DS-I", config->get_string(CFG_PREFIX  "/model_files/C-DS-I")});


  // set not set station_model_path to default model_path
  std::map<std::string,std::string>::iterator path_it;
  for ( path_it = station_to_path_.begin(); path_it != station_to_path_.end(); path_it++ )
  {
    if( path_it->second == "default")
      path_it->second = cfg_model_path_;
  }

  trimmed_scene_.reset(new Cloud());

  cfg_model_origin_frame_ = config->get_string(CFG_PREFIX "/model_origin_frame");
  cfg_record_model_ = config->get_bool_or_default(CFG_PREFIX "/record_model", false);
  model_.reset(new Cloud());

  // if recording is set, record PCD file and stores in icp_models folder structure
  // else load pcd file for every station and calculate model with normals

  if (cfg_record_model_) {
    std::string reference_station = config->get_string_or_default(CFG_PREFIX "/record_station","default");
    std::string reference_path = CONFDIR "/icp_models/"
        + reference_station + "/" + reference_station;
    std::string new_reference_path = reference_path + ".pcd";
    bool exists;
    FILE *tmp;
    size_t count = 0;

    do {
      exists = false;
      tmp = ::fopen(new_reference_path.c_str(),"r");
      if(tmp) {
        exists = true;
        ::fclose(tmp);
        new_reference_path = reference_path + std::to_string(count++) + ".pcd";
      }
    } while(exists);

    reference_path = new_reference_path + ".pcd";
    logger->log_info(name(), "Writing point cloud to %s", reference_path.c_str());
  }
  else {

    //Load default PCD file from icp_models and calculate model with normals for it

    int errnum;

    if ((errnum = pcl::io::loadPCDFile(cfg_model_path_, *model_)) < 0)
      throw fawkes::CouldNotOpenFileException(cfg_model_path_.c_str(), errnum,
                                              "Set from " CFG_PREFIX "/model_file");

    norm_est_.setInputCloud(model_);
    model_with_normals_.reset(new pcl::PointCloud<pcl::PointNormal>());
    norm_est_.compute(*model_with_normals_);
    pcl::copyPointCloud(*model_, *model_with_normals_);

    // Loading PCD file and calculation of model with normals for ALL! stations
    std::map<std::string,std::string>::iterator it;
    for (it = station_to_path_.begin(); it != station_to_path_.end(); it++ ) {
      CloudPtr model(new Cloud());
      pcl::PointCloud<pcl::PointNormal>::Ptr insert_model_with_normals(new pcl::PointCloud<pcl::PointNormal>());
      if ((errnum = pcl::io::loadPCDFile(it->second, *model)) < 0)
        throw fawkes::CouldNotOpenFileException(it->second.c_str(), errnum,
                                                "Set from " CFG_PREFIX "/model_file");

      norm_est_.setInputCloud(model);
      norm_est_.compute(*insert_model_with_normals);
      pcl::copyPointCloud(*model, *insert_model_with_normals);

      station_to_model_.insert({it->first, insert_model_with_normals});
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
  realsense_switch_ = blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());
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

      //Update Station
      logger->log_info(name(), "Received UpdateStation message");
      ConveyorPoseInterface::SetStationMessage *msg =
          bb_pose_->msgq_first<ConveyorPoseInterface::SetStationMessage>();
      bb_pose_->set_current_station(msg->station());
      result_fitness_ = std::numeric_limits<double>::min();
    }
    else {
      logger->log_warn(name(), "Unknown message received");
    }
    bb_pose_->msgq_pop();
  }

  if_read();
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
  } else if (realsense_switch_->has_writer()
      && realsense_switch_->is_enabled()) {
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

  if (bb_enable_switch_->is_enabled() && update_input_cloud()) {
    fawkes::LaserLineInterface * ll = NULL;
    bool use_laserline = laserline_get_best_fit( ll );

    // No point in recording a model when there's no laser line
    if (cfg_record_model_ && !use_laserline)
      return;

    CloudPtr cloud_in(new Cloud(**cloud_in_));

    size_t in_size = cloud_in->points.size();
    CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
    size_t out_size = cloud_vg->points.size();
    if (in_size == out_size) {
      logger->log_error(name(), "Voxel Grid failed, skipping loop!");
      return;
    }
    CloudPtr cloud_gripper = cloud_remove_gripper(cloud_vg);
    CloudPtr cloud_front = cloud_remove_offset_to_front(cloud_gripper, ll, use_laserline);

    trimmed_scene_ = cloud_remove_offset_to_left_right(cloud_front, ll, use_laserline);

    cloud_publish(cloud_in, cloud_out_raw_);
    cloud_publish(trimmed_scene_, cloud_out_trimmed_);


    if (cfg_record_model_) {
      record_model();
      cloud_publish(model_, cloud_out_model_);
    }
    else if (use_laserline) {
      if (cloud_mutex_.try_lock()) {
        try {
          logger->log_info(name(), "Cloud incoming");
          initial_tf_ = guess_initial_tf_from_laserline(ll);

          scene_with_normals_.reset(new pcl::PointCloud<pcl::PointNormal>());
          norm_est_.setInputCloud(trimmed_scene_);
          norm_est_.compute(*scene_with_normals_);
          pcl::copyPointCloud(*trimmed_scene_, *scene_with_normals_);
          scene_with_normals_->header = trimmed_scene_->header;

          icp_cancelled_ = false;
          cg_thread_->wakeup();
        } catch (std::exception &e) {
          logger->log_error(name(), "Exception preprocessing point clouds: %s", e.what());
        }
        cloud_mutex_.unlock();
      }
    }
  }
}


Eigen::Matrix4f
ConveyorPoseThread::guess_initial_tf_from_laserline(fawkes::LaserLineInterface *line)
{
  Eigen::Matrix4f rv(Eigen::Matrix4f::Identity());
  tf::Stamped<tf::Pose> conveyor_hint_laser;

  // Vector from end point 1 to end point 2
  if (!tf_listener->transform_origin(line->end_point_2_frame_id(), line->end_point_1_frame_id(),
                                     conveyor_hint_laser, Time(0,0)))
    throw fawkes::Exception("Failed to transform from %s to %s",
                            line->end_point_2_frame_id(), line->end_point_1_frame_id());

  // Halve that to get line center
  conveyor_hint_laser.setOrigin(conveyor_hint_laser.getOrigin() / 2);

  // Add distance from line center to conveyor
  // TODO: Make configurable
  conveyor_hint_laser.setOrigin(conveyor_hint_laser.getOrigin() + tf::Vector3 {
                                  double(cfg_conveyor_hint_[0]),
                                  double(cfg_conveyor_hint_[1]),
                                  double(cfg_conveyor_hint_[2]) } );
  conveyor_hint_laser.setRotation(
        tf::Quaternion(tf::Vector3(0,1,0), -M_PI/2)
        * tf::Quaternion(tf::Vector3(0,0,1), M_PI/2));

  // Transform into PCL source frame
  tf::Stamped<tf::Pose> conveyor_hint_cam;
  tf_listener->transform_pose(header_.frame_id, conveyor_hint_laser, conveyor_hint_cam);

  tf::Matrix3x3 rot = conveyor_hint_cam.getBasis();
  tf::Vector3 trans = conveyor_hint_cam.getOrigin();
  rv.block<3,3>(0,0)
      << float(rot[0][0]), float(rot[0][1]), float(rot[0][2]),
      float(rot[1][0]), float(rot[1][1]), float(rot[1][2]),
      float(rot[2][0]), float(rot[2][1]), float(rot[2][2]);
  rv.block<3,1>(0,3) << float(trans[0]), float(trans[1]), float(trans[2]);
  return rv;
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
  Eigen::Matrix4f tf_to_cam = Eigen::Matrix4f::Identity();
  btMatrix3x3 &rot = pose_cam.getBasis();
  btVector3 &trans = pose_cam.getOrigin();
  tf_to_cam(0,3) = float(trans.getX());
  tf_to_cam(1,3) = float(trans.getY());
  tf_to_cam(2,3) = float(trans.getZ());
  tf_to_cam.block<3,3>(0,0)
      << float(rot[0][0]), float(rot[0][1]), float(rot[0][2]),
      float(rot[1][0]), float(rot[1][1]), float(rot[1][2]),
      float(rot[2][0]), float(rot[2][1]), float(rot[2][2]);
  pcl::transformPointCloud(*trimmed_scene_, *model_, tf_to_cam);

  // Overwrite and atomically rename model so it can be copied at any time
  int rv = pcl::io::savePCDFileASCII(cfg_model_path_ + "_tmp", *model_);
  if (rv)
    logger->log_error(name(), "Error %d saving point cloud to %s", rv, cfg_model_path_.c_str());
  else
    ::rename((cfg_model_path_ + "_tmp").c_str(), cfg_model_path_.c_str());
}



//writes computing station to ComputationInformation interface

void
ConveyorPoseThread::set_current_station(std::string station)
{
  logger->log_info(name(), "Set Station to: %s", station.c_str());

  { MutexLocker locked(&bb_mutex_);
    icp_cancelled_ = true;
    bb_pose_->set_current_station(station.c_str());
    result_fitness_ = std::numeric_limits<double>::min();
    bb_pose_->set_euclidean_fitness(result_fitness_);
    bb_pose_->write();
  }

  auto map_it = station_to_model_.find(station);
  if (map_it == station_to_model_.end())
    logger->log_error(name(), "Invalid station name: %s", station.c_str());
  else {
    MutexLocker locked(&cloud_mutex_);
    model_with_normals_ = map_it->second;
  }
}


bool
ConveyorPoseThread::update_input_cloud()
{
  if (pcl_manager->exists_pointcloud(cloud_in_name_.c_str())) {                // does the pc exists
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
ConveyorPoseThread::if_read()
{
  // enable switch
  bb_enable_switch_->read();

  bool rv = bb_enable_switch_->is_enabled();
  while ( ! bb_enable_switch_->msgq_empty() ) {
    logger->log_info(name(),"RECEIVED SWITCH MESSAGE");
    if (bb_enable_switch_->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
      rv = false;
    } else if (bb_enable_switch_->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
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
    // just with writer
    if ( ! ll->has_writer() )
      continue;
    // just with history
    if ( ll->visibility_history() <= 2 )
      continue;
    // just if not too far away
    if ( ! tf_listener->can_transform(header_.frame_id, ll->frame_id(), fawkes::Time(0,0))) {
      logger->log_error(name(), "Can't transform from %s to %s", ll->frame_id(), header_.frame_id.c_str());
      continue;
    }
    Eigen::Vector3f center = laserline_get_center_transformed(ll);

    if ( std::sqrt( center(0) * center(0) + center(2) * center(2) ) > 0.8f )
      continue;

    // take with lowest angle
    if ( fabs(best_fit->bearing()) > fabs(ll->bearing()) )
      best_fit = ll;
  }

  if ( ! best_fit->has_writer() ) {
    logger->log_info(name(), "no writer for laser lines");
    best_fit = NULL;
    return false;
  }
  if ( best_fit->visibility_history() <= 2 ) {
    best_fit = NULL;
    return false;
  }
  if ( fabs(best_fit->bearing()) > 0.35f ) {
    best_fit = NULL;
    return false; // ~20 deg
  }
  Eigen::Vector3f center = laserline_get_center_transformed(best_fit);

  if ( std::sqrt( center(0) * center(0) + center(2) * center(2) ) > 0.8f ) {
    best_fit = NULL;
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

  tf_listener->transform_point(header_.frame_id, tf_in, tf_out);

  Eigen::Vector3f out( tf_out.getX(), tf_out.getY(), tf_out.getZ() );

  return out;
}


bool
ConveyorPoseThread::is_inbetween(double a, double b, double val) {
  double low = std::min(a, b);
  double up  = std::max(a, b);

  if (val >= low && val <= up) {
    return true;
  } else {
    return false;
  }
}


CloudPtr
ConveyorPoseThread::cloud_remove_gripper(CloudPtr in)
{
  CloudPtr out(new Cloud);
  for (Point p : *in) {
    if ( !(is_inbetween(cfg_gripper_y_min_, cfg_gripper_y_max_, p.y) && p.z < cfg_gripper_z_max_) )  { // remove gripper
      if (p.y < cfg_gripper_slice_y_max_ && p.y > cfg_gripper_slice_y_min_) { // leave just correct hight
        out->push_back(p);
      }
    }
  }
  out->header = in->header;
  return out;
}


CloudPtr
ConveyorPoseThread::cloud_remove_offset_to_front(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll)
{
  double space = cfg_front_space_;
  double z_min, z_max;
  z_min = 0;	
  if ( use_ll ) {
    Eigen::Vector3f c = laserline_get_center_transformed(ll);
    z_min = c(2) + cfg_front_offset_ - ( space / 2. );
  }
  z_max = z_min + space;

  CloudPtr out(new Cloud);
  for (Point p : *in) {
    if ( p.z >= z_min && p.z <= z_max) {
      out->push_back(p);
    }
  }

  out->header = in->header;
  return out;
}


CloudPtr
ConveyorPoseThread::cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll)
{
  CloudPtr out(new Cloud);

  if (use_ll) {
    Eigen::Vector3f c = laserline_get_center_transformed(ll);

    double x_min = c(0) - cfg_left_cut_;
    double x_max = c(0) + cfg_right_cut_;

    for (Point p : *in)
      if ( p.x >= x_min && p.x <= x_max )
        out->push_back(p);

  } else {
    logger->log_info(name(), "-------------STOPPED USING LASERLINE-----------");
    for (Point p : *in)
      if ( p.x >= -cfg_left_cut_no_ll_ && p.x <= cfg_right_cut_no_ll_ )
        out->push_back(p);
  }

  out->header = in->header;
  return out;
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
ConveyorPoseThread::tf_send_from_pose_if(pose pose)
{
  fawkes::tf::StampedTransform transform;

  transform.frame_id = header_.frame_id;
  transform.child_frame_id = conveyor_frame_id_;
  // TODO use time of header, just don't works with bagfiles
  fawkes::Time pt;
  pt.set_time((long)header_.stamp / 1000);
  transform.stamp = pt;

  transform.setOrigin(pose.getOrigin());
  transform.setRotation(pose.getRotation());

  tf_publisher->send_transform(transform);
}


void
ConveyorPoseThread::pose_write()
{
  MutexLocker locked(&bb_mutex_);

  bb_pose_->set_translation(0, result_pose_.getOrigin().getX());
  bb_pose_->set_translation(1, result_pose_.getOrigin().getY());
  bb_pose_->set_translation(2, result_pose_.getOrigin().getZ());
  bb_pose_->set_rotation(0, result_pose_.getRotation().getX());
  bb_pose_->set_rotation(1, result_pose_.getRotation().getY());
  bb_pose_->set_rotation(2, result_pose_.getRotation().getZ());
  bb_pose_->set_rotation(3, result_pose_.getRotation().getW());

  bb_pose_->set_frame(header_.frame_id.c_str());
  bb_pose_->set_euclidean_fitness(result_fitness_);
  long timestamp[2];
  result_pose_.stamp.get_timestamp(timestamp[0], timestamp[1]);
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


Eigen::Quaternion<float>
ConveyorPoseThread::averageQuaternion(Eigen::Vector4f &cumulative, Eigen::Quaternion<float> newRotation, Eigen::Quaternion<float> firstRotation, float addDet){
  
  float w = 0.0;
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;	

  if(!areQuaternionsClose(newRotation, firstRotation)){
    newRotation = inverseSignQuaternion(newRotation);
  }

  cumulative.x() += newRotation.x();
  x = cumulative.x() * addDet;
  cumulative.y() += newRotation.y();
  y = cumulative.y() * addDet;
  cumulative.z() += newRotation.z();
  z = cumulative.z() * addDet;
  cumulative.w() += newRotation.w();
  w = cumulative.w() * addDet;
  
  Eigen::Quaternion<float> result = normalizeQuaternion(x, y, z, w);
  return result;
}
Eigen::Quaternion<float>
ConveyorPoseThread::normalizeQuaternion(float x, float y, float z, float w){

  float lengthD = 1.0 / (w*w + x*x + y*y + z*z);
  w *= lengthD;
  x *= lengthD;
  y *= lengthD;
  z *= lengthD;
  
  Eigen::Quaternion<float> result(x, y, z, w);
  return result;
}
 
//Changes the sign of the quaternion components. This is not the same as the inverse.
Eigen::Quaternion<float>
ConveyorPoseThread::inverseSignQuaternion(Eigen::Quaternion<float> q){
  Eigen::Quaternion<float> result(-q.x(), -q.y(), -q.z(), -q.w());
  return result;
}
 
//Returns true if the two input quaternions are close to each other. This can
//be used to check whether or not one of two quaternions which are supposed to
//be very similar but has its component signs reversed (q has the same rotation as
//-q)
bool
ConveyorPoseThread::areQuaternionsClose(Eigen::Quaternion<float> q1, Eigen::Quaternion<float> q2){
  
  float dot = q1.dot(q2);
  if(dot < 0.0){ 
    return false;					
  } 
  else{ 
    return true;
  }
}

bool
ConveyorPoseThread::need_to_wait()
{
  return Time() < wait_start_ + wait_time_;
}

void
ConveyorPoseThread::set_cg_thread(RecognitionThread *cg_thread)
{ cg_thread_ = cg_thread; }


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

    fawkes::MutexLocker locked { &config_mutex_ };

    if (sub_prefix == "/gripper") {
      if (opt == "/y_min")
        change_val(opt, cfg_gripper_y_min_, v->get_float());
      else if (opt == "/y_max")
        change_val(opt, cfg_gripper_y_max_, v->get_float());
      else if (opt == "/z_max")
        change_val(opt, cfg_gripper_z_max_, v->get_float());
      else if (opt == "/slice/y_min")
        change_val(opt, cfg_gripper_slice_y_min_, v->get_float());
      else if (opt == "/slice/y_max")
        change_val(opt, cfg_gripper_slice_y_max_, v->get_float());
    } else if (sub_prefix == "/front") {
      if (opt == "/space")
        change_val(opt, cfg_front_space_, v->get_float());
      else if (opt == "/offset")
        change_val(opt, cfg_front_offset_, v->get_float());
    } else if (sub_prefix == "/left_right") {
      if (opt == "/left_cut")
        change_val(opt, cfg_left_cut_, v->get_float());
      else if (opt == "/right_cut")
        change_val(opt, cfg_right_cut_, v->get_float());
      else if (opt == "/left_cut_no_ll")
        change_val(opt, cfg_left_cut_no_ll_, v->get_float());
      else if (opt == "/right_cut_no_ll")
        change_val(opt, cfg_right_cut_no_ll_, v->get_float());
    } else if (sub_prefix == "/icp") {
      if (opt == "/max_correspondence_dist")
        change_val(opt, cfg_icp_max_corr_dist_, v->get_float());
      if (opt == "/conveyor_hint/x")
        change_val(opt, cfg_conveyor_hint_[0], v->get_float());
      else if (opt == "/conveyor_hint/y")
        change_val(opt, cfg_conveyor_hint_[1], v->get_float());
      else if (opt == "/conveyor_hint/z")
        change_val(opt, cfg_conveyor_hint_[2], v->get_float());
    }
  }
}
