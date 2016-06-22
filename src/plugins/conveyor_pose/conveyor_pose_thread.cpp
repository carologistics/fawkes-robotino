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
 * Plugin to detect the conveyor beld in a pointcloud (captured from intel real sens)
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
  cfg_bb_config_name_         = if_prefix + config->get_string( (cfg_prefix + "if/config").c_str() );

  laserlines_names_       = config->get_strings( (cfg_prefix + "if/laser_lines").c_str() );

//  laserlines_names_.push_back("/laser-lines/1");
//  laserlines_names_.push_back("/laser-lines/2");
//  laserlines_names_.push_back("/laser-lines/3");
//  laserlines_names_.push_back("/laser-lines/4");
//  laserlines_names_.push_back("/laser-lines/5");
//  laserlines_names_.push_back("/laser-lines/6");
//  laserlines_names_.push_back("/laser-lines/7");
//  laserlines_names_.push_back("/laser-lines/8");

//  bb_tag_name_ = "/tag-vision/0";

  cfg_pose_close_if_no_new_pointclouds_  = config->get_bool( (cfg_prefix + "if/pose_close_if_new_pc").c_str() );

  conveyor_frame_id_          = config->get_string( (cfg_prefix + "conveyor_frame_id").c_str() );
  cfg_pose_diff_              = config->get_float( (cfg_prefix + "vis_hist/diff_pose").c_str() );
  vis_hist_angle_diff_        = config->get_float( (cfg_prefix + "vis_hist/diff_angle").c_str() );
  cfg_pose_avg_hist_size_     = config->get_uint( (cfg_prefix + "vis_hist/average/size").c_str() );
  cfg_pose_avg_min_           = config->get_uint( (cfg_prefix + "vis_hist/average/used_min").c_str() );

  cfg_enable_switch_          = config->get_bool( (cfg_prefix + "switch_default").c_str() );
  cfg_enable_product_removal_ = config->get_bool( (cfg_prefix + "product_removal_default").c_str() );
  cfg_use_visualisation_      = config->get_bool( (cfg_prefix + "use_visualisation").c_str() );

  cfg_gripper_y_min_          = config->get_float( (cfg_prefix + "gripper/y_min").c_str() );
  cfg_gripper_y_max_          = config->get_float( (cfg_prefix + "gripper/y_max").c_str() );
  cfg_gripper_z_max_          = config->get_float( (cfg_prefix + "gripper/z_max").c_str() );
  cfg_gripper_slice_y_min_    = config->get_float( (cfg_prefix + "gripper/slice/y_min").c_str() );
  cfg_gripper_slice_y_max_    = config->get_float( (cfg_prefix + "gripper/slice/y_max").c_str() );

  cfg_centroid_radius_        = config->get_float( (cfg_prefix + "centroid/radius").c_str() );

  cfg_front_space_            = config->get_float( (cfg_prefix + "front/space").c_str() );
  cfg_front_offset_           = config->get_float( (cfg_prefix + "front/offset").c_str() );

  cfg_bottom_offset_          = config->get_float( (cfg_prefix + "bottom/offset").c_str() );

  cfg_product_normal_distance_weight_ = config->get_float( (cfg_prefix + "product/normal_distance_weight").c_str() );
  cfg_product_dist_threshold_         = config->get_float( (cfg_prefix + "product/dist_threshold").c_str() );
  cfg_product_radius_limit_min_       = config->get_float( (cfg_prefix + "product/radius_limit_min").c_str() );
  cfg_product_radius_limit_max_       = config->get_float( (cfg_prefix + "product/radius_limit_max").c_str() );

  cfg_plane_dist_threshold_   = config->get_float( (cfg_prefix + "plane/dist_threshold").c_str() );
  cfg_normal_z_minimum_       = config->get_float( (cfg_prefix + "plane/normal_z_minimum").c_str() );
  cfg_plane_height_minimum_   = config->get_float( (cfg_prefix + "plane/height_minimum").c_str() );

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

//  bb_tag_ = blackboard->open_for_reading<fawkes::Position3DInterface>(bb_tag_name_.c_str());

  enable_pose_ = false;
  if ( ! cfg_pose_close_if_no_new_pointclouds_ ) {
    bb_pose_conditional_open();
  }

  bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>(cfg_bb_switch_name_.c_str());
  bb_config_        = blackboard->open_for_writing<ConveyorConfigInterface>(cfg_bb_config_name_.c_str());
  bb_enable_switch_->set_enabled( cfg_debug_mode_ || cfg_enable_switch_); // ignore cfg_enable_switch_ and set to true if debug mode is used
  bb_enable_switch_->write();
  bb_config_->set_product_removal( cfg_enable_product_removal_ );
  bb_config_->write();

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
  blackboard->close(bb_config_);
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

  if ( ! pc_in_check() || ! bb_enable_switch_->is_enabled() ) {
    if ( enable_pose_ ) {
      pose trash;
      trash.valid = false;
      pose_add_element(trash);
    }

    if ( cfg_pose_close_if_no_new_pointclouds_ ) {
      bb_pose_conditional_close();
    }

    return;
  }

  bb_pose_conditional_open();

  fawkes::LaserLineInterface * ll = NULL;
  bool use_laserline = laserline_get_best_fit( ll );

  CloudPtr cloud_in(new Cloud(**cloud_in_));

  CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
  CloudPtr cloud_gripper = cloud_remove_gripper(cloud_vg);
  CloudPtr cloud_front = cloud_remove_offset_to_front(cloud_gripper, ll, use_laserline);

//  CloudPtr cloud_front_side(new Cloud);
//  if ( use_laserline ) {
//    // TODO, if this is used, a cfg values for each machine is needed
//    cloud_front_side = cloud_remove_offset_to_left_right(cloud_pt, ll);
//  } else {
//    *cloud_front_side = *cloud_front;
//  }

  Eigen::Vector4f center;
  pcl::compute3DCentroid<Point, float>(*cloud_front, center);
  CloudPtr cloud_center = cloud_remove_centroid_based(cloud_front, center);
  CloudPtr cloud_bottom_removed = cloud_remove_offset_to_bottom(cloud_center);

  CloudPtr cloud_without_products(new Cloud);
  if ( bb_config_->is_product_removal() ) {
    cloud_without_products = cloud_remove_products(cloud_bottom_removed);
//    *cloud_without_products = *cloud_bottom_removed;
  } else {
    *cloud_without_products = *cloud_bottom_removed;
  }

  // search for best plane
  CloudPtr cloud_choosen;
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  do {
    CloudPtr cloud_plane = cloud_get_plane(cloud_without_products, coeff);
    if ( cloud_plane == NULL || ! cloud_plane ) {
      pose trash;
      trash.valid = false;
      pose_add_element(trash);
      return;
    }

    size_t id;
    boost::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices = cloud_cluster(cloud_plane);
    if ( cluster_indices->size() <= 0 ) {
      pose trash;
      trash.valid = false;
      pose_add_element(trash);
      return;
    }
    std::vector<CloudPtr> clouds_cluster = cluster_split(cloud_plane, cluster_indices);
    cloud_choosen = cluster_find_biggest(clouds_cluster, id);

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
      extract.setInputCloud (cloud_without_products);
      extract.setIndices( extract_indicies );
      extract.setNegative (true);
      extract.filter (*tmp);
      *cloud_without_products = *tmp;
    } else {
      // height is ok
      break;
    }
  } while (true);

  cloud_publish(cloud_without_products, cloud_out_inter_1_);
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

  pose pose_average;
  bool pose_average_availabe = pose_get_avg(pose_average);

  if (pose_average_availabe) {
    vis_hist_ = std::max(1, vis_hist_ + 1);
    pose_write(pose_average);

    tf_send_from_pose_if(pose_current);
    if (cfg_use_visualisation_) {
      visualisation_->marker_draw(header_, pose_average.translation, pose_average.rotation);
    }
  } else {
    vis_hist_ = -1;
  }
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

  // config
  bb_config_->read();

  rv = bb_config_->is_product_removal();
  while ( ! bb_config_->msgq_empty() ) {
    if (bb_config_->msgq_first_is<ConveyorConfigInterface::DisableProductRemovalMessage>()) {
      rv = false;
    } else if (bb_config_->msgq_first_is<ConveyorConfigInterface::EnableProductRemovalMessage>()) {
      rv = true;
    }

    bb_config_->msgq_pop();
  }

  if ( rv != bb_config_->is_product_removal() ) {
    logger->log_info(name(), "*** remove products from point cloud: %s", rv ? "yes" : "no");
    bb_config_->set_product_removal( rv );
    bb_config_->write();
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

  if (invalid > 3) {
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
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  for (pose p : poses_used) {
    out.translation.setX( out.translation.x() + p.translation.x() );
    out.translation.setY( out.translation.y() + p.translation.y() );
    out.translation.setZ( out.translation.z() + p.translation.z() );

    fawkes::tf::Matrix3x3 m(p.rotation);
    fawkes::tf::Scalar rc, pc, yc;
    m.getEulerYPR(yc, pc, rc);
    roll += fawkes::normalize_rad(rc);
    pitch += fawkes::normalize_rad(pc);
    yaw += fawkes::normalize_rad(yc);
  }

  // normalize
  out.translation.setX( out.translation.x() / poses_used.size() );
  out.translation.setY( out.translation.y() / poses_used.size() );
  out.translation.setZ( out.translation.z() / poses_used.size() );

  roll /= poses_used.size();
  pitch /= poses_used.size();
  yaw /= poses_used.size();
  out.rotation.setEuler(yaw, pitch, roll);

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
    if ( ! (is_inbetween(cfg_gripper_y_min_, cfg_gripper_y_max_, p.y) && p.z < cfg_gripper_z_max_) )  { // remove gripper
      if (p.y < cfg_gripper_slice_y_max_ && p.y > cfg_gripper_slice_y_min_) { // leave just correct hight
        out->push_back(p);
      }
    }
  }

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_remove_centroid_based(CloudPtr in, Eigen::Vector4f centroid)
{
  float distance = cfg_centroid_radius_;

  CloudPtr cloud_out(new Cloud);

  for (Point p : *in) {
    if ( p.x >= centroid(0) - distance && p.x <= centroid(0) + distance &&
         p.y >= centroid(1) - distance && p.y <= centroid(1) + distance) {
      cloud_out->push_back(p);
    }
  }

  return cloud_out;
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
ConveyorPoseThread::cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll)
{
  double space = 0.12;
  Eigen::Vector3f c = laserline_get_center_transformed(ll);

  double x_min = c(0) - ( space / 2. );
  double x_max = c(0) + ( space / 2. );

  CloudPtr out(new Cloud);
  for (Point p : *in) {
    if ( p.x >= x_min && p.x <= x_max ) {
      out->push_back(p);
    }
  }

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_remove_products(CloudPtr in)
{
  // Estimate point normals
  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (in);
  ne.setKSearch (50);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (cfg_product_normal_distance_weight_);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (cfg_product_dist_threshold_);
  seg.setRadiusLimits (cfg_product_radius_limit_min_, cfg_product_radius_limit_max_);
  seg.setInputCloud (in);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud (in);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (true);
  CloudPtr cloud_cylinder (new Cloud ());
  extract.filter (*cloud_cylinder);

  return cloud_cylinder;
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
//    seg.setOptimizeCoefficients (true);
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
  Eigen::Vector4f plane;
  float coverture;
  pcl::computePointNormal(*out, plane, coverture);
  if ( plane(2) > cfg_normal_z_minimum_ ) {
    return out;
  } else {
    logger->log_info(name(), "Discard plane, because of normal (%f\t%f\t%f)", plane(0), plane(1), plane(2));

    return CloudPtr();
  }
}

boost::shared_ptr<std::vector<pcl::PointIndices>>
ConveyorPoseThread::cloud_cluster(CloudPtr in)
{
  in = cloud_voxel_grid(in);
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
  CloudPtr biggeset(new Cloud);
  size_t i = 0;
  for (CloudPtr current : clouds_in) {
    if ( biggeset->size() < current->size() ) {
      biggeset = current;
      id = i;
    }
    ++i;
  }

  return biggeset;
}

CloudPtr
ConveyorPoseThread::cloud_voxel_grid(CloudPtr in)
{
  float ls = cfg_voxel_grid_leave_size_;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  CloudPtr out (new Cloud);
  vg.setInputCloud (in);
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
  Eigen::Vector3f tf_orign;
  tf_orign(0) = 1;
  tf_orign(1) = 0;
  tf_orign(2) = 0;
  Eigen::Quaternion<float> q;
  q.setFromTwoVectors(tf_orign, normal);
  Eigen::Quaternion<float> q_offset;
  Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf yawAngle(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitchAngle(1.57, Eigen::Vector3f::UnitX());
  q_offset = rollAngle * yawAngle * pitchAngle;
  q = q * q_offset;

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







