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

#include "conveyor_pose_thread.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/registration/distances.h>
#include <pcl/features/normal_3d_omp.h>

#include <tf/types.h>
#include <utils/math/angle.h>

#include <cmath>

using namespace fawkes;

/** @class ConveyorPoseThread "conveyor_pose_thread.cpp"
 * Plugin to detect the conveyor belt in a pointcloud (captured from Intel RealSense)
 * @author Tobias Neumann
 */

/** Constructor. */
ConveyorPoseThread::ConveyorPoseThread() :
		Thread("ConveyorPoseThread", Thread::OPMODE_WAITFORWAKEUP),
		BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
		fawkes::TransformAspect(fawkes::TransformAspect::BOTH,"conveyor_pose")
{

}

void
ConveyorPoseThread::init()
{
  const std::string cfg_prefix = "/conveyor_pose/";

  cfg_debug_mode_ = config->get_bool( (cfg_prefix + "debug").c_str() );

  cloud_in_name_ = config->get_string( (cfg_prefix + "cloud_in").c_str() );

  const std::string if_prefix = config->get_string( (cfg_prefix + "if/prefix").c_str() ) + "/";

  cloud_out_inter_1_name_     = if_prefix + config->get_string( (cfg_prefix + "if/cloud_out_intermediet").c_str() );
  cloud_out_result_name_      = if_prefix + config->get_string( (cfg_prefix + "if/cloud_out_result").c_str() );
  cfg_bb_conveyor_pose_name_  = if_prefix + config->get_string( (cfg_prefix + "if/pose_of_beld").c_str() );
  cfg_bb_switch_name_         = if_prefix + config->get_string( (cfg_prefix + "if/switch").c_str() );

  laserlines_names_       = config->get_strings( (cfg_prefix + "if/laser_lines").c_str() );

  cfg_pose_close_if_no_new_pointclouds_  = config->get_bool( (cfg_prefix + "if/pose_close_if_new_pc").c_str() );

  conveyor_frame_id_          = config->get_string( (cfg_prefix + "conveyor_frame_id").c_str() );
  cfg_pose_diff_              = config->get_float( (cfg_prefix + "vis_hist/diff_pose").c_str() );
  vis_hist_angle_diff_        = config->get_float( (cfg_prefix + "vis_hist/diff_angle").c_str() );
  cfg_pose_avg_hist_size_     = config->get_uint( (cfg_prefix + "vis_hist/average/size").c_str() );
  cfg_pose_avg_min_           = config->get_uint( (cfg_prefix + "vis_hist/average/used_min").c_str() );
  cfg_allow_invalid_poses_    = config->get_uint( (cfg_prefix + "vis_hist/allow_invalid_poses").c_str() );

  cfg_enable_switch_          = config->get_bool( (cfg_prefix + "switch_default").c_str() );
  cfg_use_visualisation_      = config->get_bool( (cfg_prefix + "use_visualisation").c_str() );

  cfg_gripper_y_min_          = config->get_float( (cfg_prefix + "gripper/y_min").c_str() );
  cfg_gripper_y_max_          = config->get_float( (cfg_prefix + "gripper/y_max").c_str() );
  cfg_gripper_z_max_          = config->get_float( (cfg_prefix + "gripper/z_max").c_str() );
  cfg_gripper_slice_y_min_    = config->get_float( (cfg_prefix + "gripper/slice/y_min").c_str() );
  cfg_gripper_slice_y_max_    = config->get_float( (cfg_prefix + "gripper/slice/y_max").c_str() );

  cfg_front_space_            = config->get_float( (cfg_prefix + "front/space").c_str() );
  cfg_front_offset_           = config->get_float( (cfg_prefix + "front/offset").c_str() );

  cfg_bottom_offset_          = config->get_float( (cfg_prefix + "bottom/offset").c_str() );

  cfg_left_cut_               = config->get_float( (cfg_prefix + "left_right/left_cut").c_str() );
  cfg_right_cut_              = config->get_float( (cfg_prefix + "left_right/right_cut").c_str() );
  cfg_left_cut_no_ll_         = config->get_float( (cfg_prefix + "left_right/left_cut_no_ll").c_str() );
  cfg_right_cut_no_ll_        = config->get_float( (cfg_prefix + "left_right/right_cut_no_ll").c_str() );

  cfg_plane_dist_threshold_   = config->get_float( (cfg_prefix + "plane/dist_threshold").c_str() );
  cfg_normal_z_minimum_       = config->get_float( (cfg_prefix + "plane/normal_z_minimum").c_str() );
  cfg_plane_height_minimum_   = config->get_float( (cfg_prefix + "plane/height_minimum").c_str() );
  cfg_plane_width_minimum_    = config->get_float( (cfg_prefix + "plane/width_minimum").c_str() );

  cfg_cluster_tolerance_      = config->get_float( (cfg_prefix + "cluster/tolerance").c_str() );
  cfg_cluster_size_min_       = config->get_float( (cfg_prefix + "cluster/size_min").c_str() );
  cfg_cluster_size_max_       = config->get_float( (cfg_prefix + "cluster/size_max").c_str() );

  cfg_voxel_grid_leave_size_  = config->get_float( (cfg_prefix + "voxel_grid/leave_size").c_str() );

  cloud_in_registered_ = false;

  cloud_out_inter_1_ = new Cloud();
  cloud_out_result_ = new Cloud();
  pcl_manager->add_pointcloud(cloud_out_inter_1_name_.c_str(), cloud_out_inter_1_);
  pcl_manager->add_pointcloud(cloud_out_result_name_.c_str(), cloud_out_result_);

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

  visualisation_ = new Visualisation(rosnode);
}

void
ConveyorPoseThread::finalize()
{
  pcl_manager->remove_pointcloud(cloud_out_inter_1_name_.c_str());
  pcl_manager->remove_pointcloud(cloud_out_result_name_.c_str());
  delete visualisation_;
  blackboard->close(bb_enable_switch_);
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
  //logger->log_debug(name(),"CONVEYOR-POSE 1: Interface read");
  if ( ! pc_in_check() || ! bb_enable_switch_->is_enabled() ) {
    if ( enable_pose_ ) {
      vis_hist_ = -1;
      pose trash;
      trash.valid = false;
      pose_write(trash);
    }
    if ( cfg_pose_close_if_no_new_pointclouds_ ) {
      bb_pose_conditional_close();
    }

    return;
  }
  //logger->log_info(name(),"CONVEYOR-POSE 2: Added Trash if no point cloud or not enabled and pose enabled");
  bb_pose_conditional_open();

  pose pose_average;
  bool pose_average_availabe = pose_get_avg(pose_average);
  //logger->log_info(name(),"CONVEYOR-POSE 3: set average");
  if (pose_average_availabe) {
    vis_hist_ = std::max(1, vis_hist_ + 1);
    pose_write(pose_average);

    pose_publish_tf(pose_average);

//    tf_send_from_pose_if(pose_current);
    if (cfg_use_visualisation_) {
      visualisation_->marker_draw(header_, pose_average.translation, pose_average.rotation);
    }
  } else {
    vis_hist_ = -1;
    pose trash;
    trash.valid = false;
    pose_write(trash);
  }
// logger->log_debug(name(),"CONVEYOR-POSE 4: checked average");
  fawkes::LaserLineInterface * ll = NULL;
  bool use_laserline = laserline_get_best_fit( ll );
// logger->log_debug(name(),"CONVEYOR-POSE 5: got laserline");
  
  CloudPtr cloud_in(new Cloud(**cloud_in_));

  uint in_size = cloud_in->points.size();
 // logger->log_debug(name(), "Size before voxel grid: %u", in_size);
  CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
  uint out_size = cloud_vg->points.size();
 // logger->log_debug(name(), "Size of voxel grid: %u", out_size);
  if (in_size == out_size) {
    logger->log_error(name(), "Voxel Grid failed, skipping loop!");
    return;
  }
  CloudPtr cloud_gripper = cloud_remove_gripper(cloud_vg);
  CloudPtr cloud_front = cloud_remove_offset_to_front(cloud_gripper, ll, use_laserline);
 // logger->log_debug(name(),"CONVEYOR-POSE 6: intially filtered pointcloud");

  CloudPtr cloud_front_side(new Cloud);
    cloud_front_side = cloud_remove_offset_to_left_right(cloud_front, ll, use_laserline);
  
// logger->log_debug(name(),"CONVEYOR-POSE 7: set cut off left and rigt");
  CloudPtr cloud_bottom_removed = cloud_remove_offset_to_bottom(cloud_front_side);

//  logger->log_debug(name(),"CONVEYOR-POSE 8: removed bottom offset");
  cloud_publish(cloud_bottom_removed, cloud_out_inter_1_);

  // search for best plane
  CloudPtr cloud_choosen;
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  do {
  //  logger->log_debug(name(), "In while loop");
    CloudPtr cloud_plane = cloud_get_plane(cloud_bottom_removed, coeff);
  //  logger->log_debug(name(), "After getting plane");
    if ( cloud_plane == NULL || ! cloud_plane ) {
      pose trash;
      trash.valid = false;
      pose_add_element(trash);
      return;
    }

    size_t id;
  //  logger->log_debug(name(), "Before clustering");
    boost::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices = cloud_cluster(cloud_plane);
  //  logger->log_debug(name(), "After clustering");
    if ( cluster_indices->size() <= 0 ) {
      pose trash;
      trash.valid = false;
      pose_add_element(trash);
      return;
    }
   // logger->log_debug(name(), "Before split");
    std::vector<CloudPtr> clouds_cluster = cluster_split(cloud_plane, cluster_indices);
   // logger->log_debug(name(), "Before finding biggest");
    cloud_choosen = cluster_find_biggest(clouds_cluster, id);
   // logger->log_debug(name(), "After finding biggest");

    // check if plane is ok, otherwise remove indicies

    // check if the height is ok (remove shelfs)
    float y_min = -200;
    float y_max = 200;
    for (Point p : *cloud_choosen ) {
      if (p.y > y_min) {
        y_min = p.y;
      }
      if (p.y < y_max) {
        y_max = p.y;
      }
    }

    float height = y_min - y_max;
    if (height < cfg_plane_height_minimum_) {
      logger->log_info(name(), "Discard plane, because of height restriction. is: %f\tshould: %f", height, cfg_plane_height_minimum_);

      boost::shared_ptr<pcl::PointIndices> extract_indicies( new pcl::PointIndices(cluster_indices->at(id)) );
      CloudPtr tmp(new Cloud);
      pcl::ExtractIndices<Point> extract;
      extract.setInputCloud (cloud_bottom_removed);
      extract.setIndices( extract_indicies );
      extract.setNegative (true);
      extract.filter (*tmp);
      //logger->log_debug(name(), "After extraction");
      *cloud_bottom_removed = *tmp;
    } else {
      // height is ok
      float x_min = -200;
      float x_max = 200;
      for (Point p : *cloud_choosen ) {
        if (p.x > x_min) {
          x_min = p.x;
        }
        if (p.x < x_max) {
          x_max = p.x;
        }
      }
    
    float width = x_min - x_max;
    if (width < cfg_plane_width_minimum_) {
      logger->log_info(name(), "Discard plane, because of width restriction. is: %f\tshould: %f", width, cfg_plane_width_minimum_);      
      boost::shared_ptr<pcl::PointIndices> extract_indicies( new pcl::PointIndices(cluster_indices->at(id)) );
      CloudPtr tmp(new Cloud);
      pcl::ExtractIndices<Point> extract;
      extract.setInputCloud (cloud_bottom_removed);
      extract.setIndices( extract_indicies );
      extract.setNegative (true);
      extract.filter (*tmp);
      *cloud_bottom_removed = *tmp;

    } else {
    //height and width ok
      break;
    }
   }
  } while (true);
  
 // logger->log_debug(name(),"CONVEYOR-POSE 9: left while true");
  cloud_publish(cloud_choosen, cloud_out_result_);

  // get centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<Point, float>(*cloud_choosen, centroid);

  Eigen::Vector3f normal;
  normal(0) = coeff->values[0];
  normal(1) = coeff->values[1];
  normal(2) = coeff->values[2];
  pose pose_current = calculate_pose(centroid, normal);;
  pose_add_element(pose_current);
}

bool
ConveyorPoseThread::pc_in_check()
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
    logger->log_info(name(),"RECIEVED SWITCH MESSAGE");
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
  pose median;

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
  median.translation.setValue(0, 0, 0);
  median.rotation.setEuler(0, 0, 0);
  unsigned int iteraterions = 20;
  for (unsigned int i = 0; i < iteraterions; ++i) {

    fawkes::tf::Vector3 numerator(0, 0, 0);
    double divisor = 0;

    for (pose p : poses_) {
      if ( p.valid ) {
        double divisor_current = (p.translation - median.translation).norm();
        divisor += ( 1 / divisor_current );
        numerator += ( p.translation / divisor_current );
//        logger->log_info(name(), "(%lf\t%lf\t%lf) /\t%lf", numerator.x(), numerator.y(), numerator.z(), divisor);
      }
    }
    median.translation = numerator / divisor;

  }

  // remove outliers
  std::list<pose> poses_used;
  for (pose p : poses_) {
    if ( p.valid ) {
      double dist = (p.translation - median.translation).norm();

//      logger->log_info(name(), "(%f\t%f\t%f)\t(%f\t%f\t%f) => %lf",
//          median.translation.x(), median.translation.y(), median.translation.z(),
//          p.translation.x(), p.translation.y(), p.translation.z(),
//          dist);

      if (dist <= cfg_pose_diff_) {
        poses_used.push_back(p);
      }
    }
  }

  if (poses_used.size() <= cfg_pose_avg_min_) {
    logger->log_warn(name(), "not enough for average, got: %u", poses_used.size());
    return false;
  }

  // calculate average
  Eigen::Quaternion<float> avgRot;
  Eigen::Vector4f cumulative;
  Eigen::Quaternion<float> firstRotation(poses_used.front().rotation.x(), poses_used.front().rotation.y(), poses_used.front().rotation.z(), poses_used.front().rotation.w());
  float addDet = 1.0 / (float)poses_used.size();
  for (pose p : poses_used) {
    Eigen::Quaternion<float> newRotation(p.rotation.x(), p.rotation.y(), p.rotation.z(), p.rotation.w());

    out.translation.setX( out.translation.x() + p.translation.x() );
    out.translation.setY( out.translation.y() + p.translation.y() );
    out.translation.setZ( out.translation.z() + p.translation.z() );

  //  fawkes::tf::Matrix3x3 m(p.rotation);
  //  fawkes::tf::Scalar rc, pc, yc;
  //  m.getEulerYPR(yc, pc, rc);
  //  roll += fawkes::normalize_rad(rc);
  //  pitch += fawkes::normalize_rad(pc);
  //  yaw += fawkes::normalize_rad(yc);
    avgRot = averageQuaternion(cumulative, newRotation, firstRotation, addDet);
  }

  // normalize
  out.translation.setX( out.translation.x() / poses_used.size() );
  out.translation.setY( out.translation.y() / poses_used.size() );
  out.translation.setZ( out.translation.z() / poses_used.size() );

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
    if ( ! ll->has_writer() ) { continue; }
    // just with history
    if ( ll->visibility_history() <= 2 ) { continue; }
    // just if not too far away
    Eigen::Vector3f center = laserline_get_center_transformed(ll);

    if ( std::sqrt( center(0) * center(0) +
                    center(2) * center(2) ) > 0.8 ) { continue; }

    // take with lowest angle
    if ( fabs(best_fit->bearing()) > fabs(ll->bearing()) ) {
      best_fit = ll;
    }
  }

  if ( ! best_fit->has_writer() ) { logger->log_info(name(), "no writer for laser lines"); best_fit = NULL; return false; }
  if ( best_fit->visibility_history() <= 2 ) { best_fit = NULL; return false;  }
  if ( fabs(best_fit->bearing()) > 0.35 ) { best_fit = NULL; return false;  } // ~20 deg
  Eigen::Vector3f center = laserline_get_center_transformed(best_fit);

  if ( std::sqrt( center(0) * center(0) +
                  center(2) * center(2) ) > 0.8 ) { best_fit = NULL; return false;  }

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
ConveyorPoseThread::cloud_remove_offset_to_bottom(CloudPtr in)
{
  float lowest = -200;
  for (Point p : *in) {
    if ( p.y > lowest ) { // the y axis faces downwards
      lowest = p.y;
    }
  }

  CloudPtr out(new Cloud);
  float limit = lowest - cfg_bottom_offset_;
  for (Point p : *in) {
    if (p.y >= limit) {
      out->push_back(p);
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
  } else {
    // get lowest z point
    double lowest_z = 1000;
    for (Point p : *in) {
      if ( p.z < lowest_z ) {
        lowest_z = p.z;
      }
    }

    z_min = lowest_z - (lowest_z / 2.);
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
ConveyorPoseThread::cloud_get_plane(CloudPtr in, pcl::ModelCoefficients::Ptr coeff)
{
  CloudPtr out ( new Cloud(*in) );

  // get planes
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (cfg_plane_dist_threshold_);

  seg.setInputCloud (out);
  seg.segment (*inliers, *coeff);

  if (inliers->indices.size () == 0) {
    logger->log_error(name(), "Could not estimate a planar model for the given dataset.");
    return CloudPtr();
  }

  // get inliers
  CloudPtr tmp ( new Cloud );
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud (out);
  extract.setIndices (inliers);
  extract.setNegative (false);

  extract.filter (*tmp);
  *out = *tmp;

  // check if cloud normal is ok
  return out;
}

boost::shared_ptr<std::vector<pcl::PointIndices>>
ConveyorPoseThread::cloud_cluster(CloudPtr in)
{
  //in = cloud_voxel_grid(in);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (in);

  boost::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices(new std::vector<pcl::PointIndices>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (cfg_cluster_tolerance_);
  ec.setMinClusterSize (cfg_cluster_size_min_);
  ec.setMaxClusterSize (cfg_cluster_size_max_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (in);
  ec.extract (*cluster_indices);

  return cluster_indices;
}

std::vector<CloudPtr>
ConveyorPoseThread::cluster_split(CloudPtr in, boost::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices)
{
  std::vector<CloudPtr> clouds_out;
  for (std::vector<pcl::PointIndices>::const_iterator c_it = cluster_indices->begin (); c_it != cluster_indices->end (); ++c_it) {
    CloudPtr cloud_cluster (new Cloud);
    for (std::vector<int>::const_iterator p_it = c_it->indices.begin (); p_it != c_it->indices.end (); ++p_it) {
      cloud_cluster->points.push_back (in->points[*p_it]);
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clouds_out.push_back(cloud_cluster);
  }

  return clouds_out;
}

CloudPtr
ConveyorPoseThread::cluster_find_biggest(std::vector<CloudPtr> clouds_in, size_t & id)
{
  CloudPtr biggest(new Cloud);
  size_t i = 0;
  for (CloudPtr current : clouds_in) {
    if ( biggest->size() < current->size() ) {
      biggest = current;
      id = i;
    }
    ++i;
  }

  return biggest;
}

CloudPtr
ConveyorPoseThread::cloud_voxel_grid(CloudPtr in)
{
  float ls = cfg_voxel_grid_leave_size_;
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

ConveyorPoseThread::pose
ConveyorPoseThread::calculate_pose(Eigen::Vector4f centroid, Eigen::Vector3f normal)
{
  Eigen::Vector3f tangent0 = normal.cross(Eigen::Vector3f(1,0,0));
  if (tangent0.dot(tangent0) < 0.0001){
    tangent0 = normal.cross(Eigen::Vector3f(0,1,0));
  }
  tangent0.normalize();
  Eigen::Vector3f tangent1 = normal.cross(tangent0);
  tangent1.normalize();
  Eigen::Matrix3f rotMatrix;
  rotMatrix << tangent0, tangent1, normal;
  Eigen::Quaternion<float> q(rotMatrix);
  fawkes::tf::Vector3 origin(centroid(0), centroid(1), centroid(2));
  fawkes::tf::Quaternion rot(q.x(), q.y(), q.z(), q.w());

  pose ret;
  ret.translation = origin;
  ret.rotation = rot;

  return ret;
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

  transform.setOrigin(pose.translation);
  transform.setRotation(pose.rotation);

  tf_publisher->send_transform(transform);
}

void
ConveyorPoseThread::pose_write(pose pose)
{
  double translation[3], rotation[4];
  translation[0] = pose.translation.x();
  translation[1] = pose.translation.y();
  translation[2] = pose.translation.z();
  rotation[0] = pose.rotation.x();
  rotation[1] = pose.rotation.y();
  rotation[2] = pose.rotation.z();
  rotation[3] = pose.rotation.w();
  bb_pose_->set_translation(translation);
  bb_pose_->set_rotation(rotation);

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
  tf_pose_cam.setOrigin(tf::Vector3( pose.translation.x(), pose.translation.y(), pose.translation.z() ));
  tf_pose_cam.setRotation(tf::Quaternion( pose.rotation.x(), pose.rotation.y(), pose.rotation.z(), pose.rotation.w() ));
  tf_listener->transform_pose("gripper", tf_pose_cam, tf_pose_gripper);

  // publish the transform from the gripper to the conveyor
  tf::Transform transform(
                  tf::create_quaternion_from_yaw(M_PI),
                  tf_pose_gripper.getOrigin()
                );
  tf::StampedTransform stamped_transform(transform, tf_pose_gripper.stamp, tf_pose_gripper.frame_id, "conveyor");
  tf_publisher->send_transform(stamped_transform);
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




