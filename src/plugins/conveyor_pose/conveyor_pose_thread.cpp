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
  , realsense_switch_(nullptr)
{}

void
ConveyorPoseThread::init()
{
  config->add_change_handler(this);

  cfg_debug_mode_ = config->get_bool( CFG_PREFIX "/debug" );
  cloud_in_name_ = config->get_string( CFG_PREFIX "/cloud_in" );

  const std::string if_prefix = config->get_string( CFG_PREFIX "/if/prefix" ) + "/";

  cfg_bb_conveyor_pose_name_  = if_prefix + config->get_string( CFG_PREFIX "/if/name" );
  cfg_bb_switch_name_         = if_prefix + config->get_string( CFG_PREFIX "/if/switch" );

  laserlines_names_       = config->get_strings( CFG_PREFIX "/if/laser_lines" );

  cfg_pose_close_if_no_new_pointclouds_  = config->get_bool( CFG_PREFIX "/if/pose_close_if_new_pc" );

  conveyor_frame_id_          = config->get_string( CFG_PREFIX "/conveyor_frame_id" );
  cfg_pose_diff_              = config->get_float( CFG_PREFIX "/vis_hist/diff_pose" );
  vis_hist_angle_diff_        = config->get_float( CFG_PREFIX "/vis_hist/diff_angle" );
  cfg_pose_avg_hist_size_     = config->get_uint( CFG_PREFIX "/vis_hist/average/size" );
  cfg_pose_avg_min_           = config->get_uint( CFG_PREFIX "/vis_hist/average/used_min" );
  cfg_allow_invalid_poses_    = config->get_uint( CFG_PREFIX "/vis_hist/allow_invalid_poses" );

  cfg_enable_switch_          = config->get_bool( CFG_PREFIX "/switch_default" );
  cfg_use_visualisation_      = config->get_bool( CFG_PREFIX "/use_visualisation" );

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

  cfg_model_ss_               = double(config->get_float_or_default(CFG_PREFIX "/cg/model_sampling_radius", 0.01f));
  cfg_scene_ss_               = double(config->get_float_or_default(CFG_PREFIX "/cg/scene_sampling_radius", 0.03f));
  cfg_rf_rad_                 = double(config->get_float_or_default(CFG_PREFIX "/cg/reference_frame_radius", 0.015f));
  cfg_descr_rad_              = double(config->get_float_or_default(CFG_PREFIX "/cg/descriptor_radius", 0.02f));
  cfg_cg_size_                = double(config->get_float_or_default(CFG_PREFIX "/cg/cluster_size", 0.01f));
  cfg_cg_thresh_              = config->get_int_or_default(CFG_PREFIX "/cg/clustering_threshold", 5);
  cfg_use_hough_              = config->get_bool_or_default(CFG_PREFIX "/cg/use_hough", false);
  cfg_max_descr_dist_         = config->get_float_or_default(CFG_PREFIX "/cg/max_descriptor_distance", 0.25f);

  cfg_voxel_grid_leaf_size_  = config->get_float( CFG_PREFIX "/voxel_grid/leaf_size" );

  cfg_bb_realsense_switch_name_ = config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");
  wait_time_ = Time(double(config->get_float_or_default(CFG_PREFIX "/realsense_wait_time", 1.0f)));

  cfg_model_path_ = config->get_string(CFG_PREFIX "/model_file");
  if (cfg_model_path_.substr(0, 1) != "/")
    cfg_model_path_ = CONFDIR "/" + cfg_model_path_;

  trimmed_scene_.reset(new Cloud());

  cfg_record_model_ = config->get_bool_or_default(CFG_PREFIX "/record_model", false);
  if (cfg_record_model_) {
    FILE *tmp = nullptr;
    unsigned int count = 1;
    bool exists;
    std::string new_model_path = cfg_model_path_;
    do {
      exists = false;
      tmp = ::fopen(new_model_path.c_str(), "r");
      if (tmp) {
        exists = true;
        ::fclose(tmp);
        new_model_path = cfg_model_path_ + std::to_string(count++);
      }
    } while(exists);
    cfg_model_path_ = new_model_path;
    logger->log_info(name(), "Writing point cloud to %s", cfg_model_path_.c_str());
  }
  else {
    int errnum;
    model_.reset(new Cloud());
    if ((errnum = pcl::io::loadPCDFile(cfg_model_path_, *model_)) < 0)
      throw fawkes::CouldNotOpenFileException(cfg_model_path_.c_str(), errnum,
                                              "Set from " CFG_PREFIX "/model_file");

    uniform_sampling_.setInputCloud(model_);
    uniform_sampling_.setRadiusSearch(cfg_model_ss_);
    model_keypoints_.reset(new Cloud());
    uniform_sampling_.filter(*model_keypoints_);
    logger->log_info(name(), "Model total points: %zu; Selected Keypoints: %zu",
                     model_->size(), model_keypoints_->size());

    model_normals_.reset(new pcl::PointCloud<pcl::Normal>());
    norm_est_.setKSearch(10);
    norm_est_.setInputCloud(model_);
    norm_est_.compute(*model_normals_);

    //  Compute Descriptor for keypoints
    descr_est_.setRadiusSearch(cfg_descr_rad_);
    descr_est_.setInputCloud(model_keypoints_);
    descr_est_.setInputNormals(model_normals_);
    descr_est_.setSearchSurface(model_);
    model_descriptors_.reset(new pcl::PointCloud<pcl::SHOT352>());
    descr_est_.compute(*model_descriptors_);

    if (!model_descriptors_->is_dense) {
      throw fawkes::Exception("Failed to compute model descriptors");
    }
  }

  cloud_in_registered_ = false;

  cloud_out_raw_ = new Cloud();
  cloud_out_trimmed_ = new Cloud();
  pcl_manager->add_pointcloud(cloud_out_raw_name_.c_str(), cloud_out_raw_);
  pcl_manager->add_pointcloud(cloud_out_trimmed_name_.c_str(), cloud_out_trimmed_);

  for (std::string ll : laserlines_names_) {
    laserlines_.push_back( blackboard->open_for_reading<fawkes::LaserLineInterface>(ll.c_str()) );
  }

  enable_pose_ = false;
  if ( ! cfg_pose_close_if_no_new_pointclouds_ ) {
    bb_pose_conditional_open();
  }

  bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>(cfg_bb_switch_name_.c_str());
  bb_enable_switch_->set_enabled( cfg_debug_mode_ || cfg_enable_switch_); // ignore cfg_enable_switch_ and set to true if debug mode is used
  bb_enable_switch_->write();

  realsense_switch_ = blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());

  visualisation_ = new Visualisation(rosnode);
}


void
ConveyorPoseThread::finalize()
{
  pcl_manager->remove_pointcloud(cloud_out_raw_name_.c_str());
  pcl_manager->remove_pointcloud(cloud_out_trimmed_name_.c_str());
  delete visualisation_;
  blackboard->close(bb_enable_switch_);
  logger->log_info(name(), "Unloading, disabling %s",
                   cfg_bb_realsense_switch_name_.c_str());
  realsense_switch_->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
  blackboard->close(realsense_switch_);
  bb_pose_conditional_close();
}

void
ConveyorPoseThread::bb_pose_conditional_open()
{
  if ( ! enable_pose_ ) {
    enable_pose_ = true;
    bb_pose_ = blackboard->open_for_writing<fawkes::Position3DInterface>(cfg_bb_conveyor_pose_name_.c_str());
  }
}

void
ConveyorPoseThread::bb_pose_conditional_close()
{
  if ( enable_pose_ ) {
    enable_pose_ = false;
    blackboard->close(bb_pose_);
  }
}

void
ConveyorPoseThread::loop()
{
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

  if ( ! update_input_cloud() || ! bb_enable_switch_->is_enabled() ) {
    if ( enable_pose_ ) {
      vis_hist_ = -1;
      pose trash { false };
      pose_write(trash);
    }
    if ( cfg_pose_close_if_no_new_pointclouds_ ) {
      bb_pose_conditional_close();
    }
  }

  if (need_to_wait()) {
    logger->log_debug(name(),
      "Waiting for %s for %f sec, still %f sec remaining",
      cfg_bb_realsense_switch_name_.c_str(),
      wait_time_.in_sec(), (wait_start_ + wait_time_ - Time()).in_sec() );
    return;
  }
  bb_pose_conditional_open();

  if (!cfg_record_model_) {
    pose pose_average { true };
    bool pose_average_availabe = pose_get_avg(pose_average);
    if (pose_average_availabe) {
      vis_hist_ = std::max(1, vis_hist_ + 1);
      pose_write(pose_average);

      pose_publish_tf(pose_average);

      //    tf_send_from_pose_if(pose_current);
      if (cfg_use_visualisation_) {
        visualisation_->marker_draw(header_, pose_average.getOrigin(), pose_average.getRotation());
      }
    } else {
      vis_hist_ = -1;
      pose trash { false };
      pose_write(trash);
    }
  }

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

  { MutexLocker locked(&cloud_mutex_);
    trimmed_scene_ = cloud_remove_offset_to_left_right(cloud_front, ll, use_laserline);

    cloud_publish(cloud_in, cloud_out_raw_);
    cloud_publish(trimmed_scene_, cloud_out_trimmed_);

    if (cfg_record_model_) {
      tf::Stamped<tf::Pose> pose_cam;
      tf_listener->transform_origin(cloud_in_->header.frame_id, "gripper", pose_cam);
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
      Cloud transformed_cloud;
      pcl::transformPointCloud(*trimmed_scene_, transformed_cloud, tf_to_cam);

      // Overwrite and atomically rename model file so it can be copied at any time
      int rv = pcl::io::savePCDFileASCII(cfg_model_path_ + "_tmp", transformed_cloud);
      if (rv)
        logger->log_error(name(), "Error %d saving point cloud to %s", rv, cfg_model_path_.c_str());
      else
        ::rename((cfg_model_path_ + "_tmp").c_str(), cfg_model_path_.c_str());
    }
  }

  if (!cfg_record_model_ && bb_enable_switch_->is_enabled())
    cg_thread_->wakeup();
}


CloudPtr ConveyorPoseThread::get_scene()
{
  MutexLocker locked(&cloud_mutex_);
  return trimmed_scene_;
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

    return time_old != header_.stamp;                                          // true, if there is a new cloud, false otherwise

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

void
ConveyorPoseThread::pose_add_element(pose element)
{
  MutexLocker locked(&pose_mutex_);
  //add element
  poses_.push_front(element);

  // if to full, remove oldest
  while (poses_.size() > cfg_pose_avg_hist_size_) {
    poses_.pop_back();
  }
}

bool
ConveyorPoseThread::pose_get_avg(pose & out)
{
  pose median { true };

  // count invalid loops
  unsigned int invalid = 0;
  for (pose p : poses_) {
    if ( ! p.valid ) {
      invalid++;
    }
  }

  if (invalid > cfg_allow_invalid_poses_) {
    logger->log_warn(name(), "view unstable, got %u invalid frames", invalid);
  }

  // Weiszfeld's algorithm to find the geometric median
  median.setOrigin({0, 0, 0});
  median.setRotation(fawkes::tf::Quaternion({0, 0, 0}));
  unsigned int iteraterions = 20;
  for (unsigned int i = 0; i < iteraterions; ++i) {

    fawkes::tf::Vector3 numerator(0, 0, 0);
    double divisor = 0;

    for (pose p : poses_) {
      if ( p.valid ) {
        double divisor_current = (p.getOrigin() - median.getOrigin()).norm();
        divisor += ( 1 / divisor_current );
        numerator += ( p.getOrigin() / divisor_current );
//        logger->log_info(name(), "(%lf\t%lf\t%lf) /\t%lf", numerator.x(), numerator.y(), numerator.z(), divisor);
      }
    }
    median.setOrigin(numerator / divisor);
  }

  // remove outliers
  std::list<pose> poses_used;
  for (pose p : poses_) {
    if ( p.valid ) {
      double dist = (p.getOrigin() - median.getOrigin()).norm();

//      logger->log_warn(name(), "(%f\t%f\t%f)\t(%f\t%f\t%f) => %lf",
//          median.translation.x(), median.translation.y(), median.translation.z(),
//          p.translation.x(), p.translation.y(), p.translation.z(),
//          dist);
      if (dist <= cfg_pose_diff_) {
        poses_used.push_back(p);
      }
    }
  }

  if (poses_used.size() <= cfg_pose_avg_min_) {
    logger->log_warn(name(), "not enough for average, got: %zu", poses_used.size());
    return false;
  }

  // calculate average
//  Eigen::Quaternion<float> avgRot;
//  Eigen::Vector4f cumulative;
//  Eigen::Quaternion<float> firstRotation(poses_used.front().rotation.x(), poses_used.front().rotation.y(), poses_used.front().rotation.z(), poses_used.front().rotation.w());
//  float addDet = 1.0 / (float)poses_used.size();
  for (pose p : poses_used) {
//    Eigen::Quaternion<float> newRotation(p.rotation.x(), p.rotation.y(), p.rotation.z(), p.rotation.w());

    out.setOrigin(out.getOrigin() + p.getOrigin());

  //  fawkes::tf::Matrix3x3 m(p.rotation);
  //  fawkes::tf::Scalar rc, pc, yc;
  //  m.getEulerYPR(yc, pc, rc);
  //  roll += fawkes::normalize_rad(rc);
  //  pitch += fawkes::normalize_rad(pc);
  //  yaw += fawkes::normalize_rad(yc);
//    avgRot = averageQuaternion(cumulative, newRotation, firstRotation, addDet);
  }

  // normalize
  out.setOrigin(out.getOrigin() / poses_used.size());

  //roll /= poses_used.size();
  //pitch /= poses_used.size();
  //yaw /= poses_used.size();
  //out.rotation.setEuler(yaw, pitch, roll);

//  logger->log_info(name(), "got %u for avg: (%f\t%f\t%f)\t(%f\t%f\t%f)", poses_used.size(),
//      out.translation.x(), out.translation.y(), out.translation.z(),
//      roll, pitch, yaw);

  return true;
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

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll)
{
  if (use_ll){
    Eigen::Vector3f c = laserline_get_center_transformed(ll);

    double x_min = c(0) - cfg_left_cut_;
    double x_max = c(0) + cfg_right_cut_;

    CloudPtr out(new Cloud);
    for (Point p : *in) {
      if ( p.x >= x_min && p.x <= x_max ) {
        out->push_back(p);
      }
    }
    return out;
  }else{
    logger->log_info(name(), "-------------STOPPED USING LASERLINE-----------");
    CloudPtr out(new Cloud);
    for (Point p : *in) {
      if ( p.x >= -cfg_left_cut_no_ll_ && p.x <= cfg_right_cut_no_ll_ ) {
        out->push_back(p);
      }
    }
    return out;
  }
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
ConveyorPoseThread::pose_write(pose pose)
{
  bb_pose_->set_translation(0, pose.getOrigin().getX());
  bb_pose_->set_translation(1, pose.getOrigin().getY());
  bb_pose_->set_translation(2, pose.getOrigin().getZ());
  bb_pose_->set_rotation(0, pose.getRotation().getX());
  bb_pose_->set_rotation(1, pose.getRotation().getY());
  bb_pose_->set_rotation(2, pose.getRotation().getZ());
  bb_pose_->set_rotation(3, pose.getRotation().getW());

  bb_pose_->set_frame(header_.frame_id.c_str());
  fawkes::Time stamp((long)header_.stamp);
  bb_pose_->set_visibility_history(vis_hist_);
  bb_pose_->write();
}

void
ConveyorPoseThread::pose_publish_tf(pose pose)
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
                  tf::create_quaternion_from_yaw(M_PI),
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
ConveyorPoseThread::set_cg_thread(CorrespondenceGroupingThread *cg_thread)
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
    logger->log_info(name(), "Updating config value %s", v->path());
    std::string sufx = path.substr(strlen(CFG_PREFIX));
    std::string sub_prefix = sufx.substr(0, sufx.substr(1).find("/")+1);
    std::string full_pfx = CFG_PREFIX + sub_prefix;
    std::string opt = path.substr(full_pfx.length());

    fawkes::MutexLocker locked { &config_mutex_ };

    if (sub_prefix == "/vis_hist") {
      if (opt == "/diff_pose")
        cfg_pose_diff_ = v->get_float();
      else if (opt == "/diff_angle")
        vis_hist_angle_diff_ = v->get_float();
    } else if (sub_prefix == "/cg") {
      if (opt == "/model_sampling_radius")
        cfg_model_ss_ = double(v->get_float());
      else if (opt == "/scene_sampling_radius")
        cfg_scene_ss_ = double(v->get_float());
      else if (opt == "/reference_frame_radius")
        cfg_rf_rad_ = double(v->get_float());
      else if (opt == "/descriptor_radius")
        cfg_descr_rad_ = double(v->get_float());
      else if (opt == "/cluster_size")
        cfg_cg_size_ = double(v->get_float());
      else if (opt == "/clustering_threshold")
        cfg_cg_thresh_ = v->get_int();
      else if (opt == "/max_descriptor_distance")
        cfg_max_descr_dist_ = v->get_float();
    } else if (sub_prefix == "/gripper") {
      if (opt == "/y_min")
        cfg_gripper_y_min_ = v->get_float();
      else if (opt == "/y_max")
        cfg_gripper_y_max_ = v->get_float();
      else if (opt == "/z_max")
        cfg_gripper_z_max_ = v->get_float();
      else if (opt == "/slice/y_min")
        cfg_gripper_slice_y_min_ = v->get_float();
      else if (opt == "/slice/y_max")
        cfg_gripper_slice_y_max_ = v->get_float();
    } else if (sub_prefix == "/front") {
      if (opt == "/space")
        cfg_front_space_ = v->get_float();
      else if (opt == "/offset")
        cfg_front_offset_ = v->get_float();
    } else if (sub_prefix == "/left_right") {
      if (opt == "/left_cut")
        cfg_left_cut_ = v->get_float();
      else if (opt == "/right_cut")
        cfg_right_cut_ = v->get_float();
      else if (opt == "/left_cut_no_ll")
        cfg_left_cut_no_ll_ = v->get_float();
      else if (opt == "/right_cut_no_ll")
        cfg_right_cut_no_ll_ = v->get_float();
    }
  }
}
