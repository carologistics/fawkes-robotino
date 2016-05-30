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

#include <tf/types.h>

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
//  cloud_in_name_ = "/depth/points";
  cloud_in_name_ = "/camera/depth/points";
//  cloud_in_name_ = "/conveyor/cam";
  cloud_out_inter_1_name_ = "conveyor_pose/plane";
  cloud_out_result_name_ = "conveyor_pose/result";
  conveyor_pose_name_ = "conveyor_pose/pose";
  switch_name_ = "conveyor_pose/switch";
  conveyor_frame_id_ = "conveyor_pose";
  vis_hist_pose_diff = 0.02;
  vis_hist_angle_diff = 0.1;

  bb_tag_name_ = "/tag-vision/0";
  cloud_in_registered_ = false;
  cfg_enable_switch_ = true;
  cfg_use_visualisation_ = true;

  laserlines_names_.push_back("/laser-lines/1");
  laserlines_names_.push_back("/laser-lines/2");
  laserlines_names_.push_back("/laser-lines/3");
  laserlines_names_.push_back("/laser-lines/4");
  laserlines_names_.push_back("/laser-lines/5");
  laserlines_names_.push_back("/laser-lines/6");
  laserlines_names_.push_back("/laser-lines/7");
  laserlines_names_.push_back("/laser-lines/8");

  for (std::string ll : laserlines_names_) {
    laserlines_.push_back( blackboard->open_for_reading<fawkes::LaserLineInterface>(ll.c_str()) );
  }
  bb_tag_ = blackboard->open_for_reading<fawkes::Position3DInterface>(bb_tag_name_.c_str());

  cloud_out_inter_1_ = new Cloud();
  cloud_out_result_ = new Cloud();
  pcl_manager->add_pointcloud(cloud_out_inter_1_name_.c_str(), cloud_out_inter_1_);
  pcl_manager->add_pointcloud(cloud_out_result_name_.c_str(), cloud_out_result_);

  bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>(switch_name_.c_str());
  bb_pose_ = blackboard->open_for_writing<fawkes::Position3DInterface>(conveyor_pose_name_.c_str());
  bb_enable_switch_->set_enabled(cfg_enable_switch_);
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
}

void
ConveyorPoseThread::loop()
{
  std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose_current;

  bb_switch_is_enabled();
  if ( ! cfg_enable_switch_ || ! pc_in_check() ) {
    vis_hist_ = -1;
    pose_write(pose_current);
    return;
  }

  if_read();

//  fawkes::LaserLineInterface * ll = NULL;
//  bool use_laserline = false;//laserline_get_best_fit( ll );

  CloudPtr cloud_in(new Cloud(**cloud_in_));

  CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
  CloudPtr cloud_gripper = cloud_remove_gripper(cloud_vg);
//  CloudPtr cloud_front = cloud_remove_offset_to_front(cloud_pt, ll, use_laserline);
//
//  CloudPtr cloud_front_side(new Cloud);
//  if ( use_laserline ) {
//    // TODO, if this is used, a cfg values for each machine is needed
//    cloud_front_side = cloud_remove_offset_to_left_right(cloud_pt, ll);
//  } else {
//    *cloud_front_side = *cloud_front;
//  }

  Eigen::Vector4f center;
  pcl::compute3DCentroid<Point, float>(*cloud_gripper, center);
  CloudPtr cloud_center = cloud_remove_centroid_based(cloud_gripper, center);

  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  CloudPtr cloud_plane = cloud_get_plane(cloud_center, coeff);
  if ( cloud_plane == NULL ) {
    vis_hist_ = -1;
    pose_write(pose_current);
    return;
  }

//  CloudPtr cloud_biggest = cloud_cluster(cloud_center);
  CloudPtr cloud_biggest = cloud_cluster(cloud_plane);
  cloud_publish(cloud_center, cloud_out_inter_1_);
  cloud_publish(cloud_biggest, cloud_out_result_);

  // get centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<Point, float>(*cloud_biggest, centroid);

  Eigen::Vector3f normal;
  normal(0) = coeff->values[0];
  normal(1) = coeff->values[1];
  normal(2) = coeff->values[2];
  pose_current = calculate_pose(centroid, normal);

  update_vis_hist_by_pose_diff(pose_current);
  pose_ = pose_current;
  pose_write(pose_);
  tf_send_from_pose_if(pose_);

  visualisation_->marker_draw(header_, centroid, coeff);
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
  for (fawkes::LaserLineInterface * ll : laserlines_) {
    ll->read();
  }
  bb_tag_->read();
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

  if ( ! best_fit->has_writer() ) { logger->log_info(name(), "no writer"); best_fit = NULL; return false; }
  if ( best_fit->visibility_history() <= 2 ) { logger->log_info(name(), "vis hist"); best_fit = NULL; return false;  }
  if ( fabs(best_fit->bearing()) > 0.35 ) { logger->log_info(name(), "angle"); best_fit = NULL; return false;  } // ~20 deg
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

//  logger->log_info(name(), "laser: %f %f %f", tf_in.getX(), tf_in.getY(), tf_in.getZ());
  tf_listener->transform_point(header_.frame_id, tf_in, tf_out);
//  logger->log_info(name(), "camer: %f %f %f", tf_out.getX(), tf_out.getY(), tf_out.getZ());

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

void
ConveyorPoseThread::update_vis_hist_by_pose_diff(std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose_current)
{
  // check distance
  float pl_sqr = pose_.first.x() * pose_.first.x() +
                 pose_.first.y() * pose_.first.y() +
                 pose_.first.z() * pose_.first.z();
  float pc_sqr = pose_current.first.x() * pose_current.first.x() +
                 pose_current.first.y() * pose_current.first.y() +
                 pose_current.first.z() * pose_current.first.z();
  if ( std::fabs(pc_sqr - pl_sqr) > vis_hist_pose_diff ) {
    vis_hist_ = 1;
    return;
  }

  // check angle
  if ( pose_.second.angleShortestPath( pose_current.second ) > vis_hist_angle_diff ) {
    vis_hist_ = 1;
    return;
  }

  // otherwise integrate
  vis_hist_ = std::max(vis_hist_, 0) + 1;
}

CloudPtr
ConveyorPoseThread::cloud_remove_gripper(CloudPtr in)
{
  CloudPtr out(new Cloud);
  for (Point p : *in) {
    if ( ! (is_inbetween(-0.02, -0.055, p.y) && p.z < 0.11) )  { // remove gripper
      if (p.y < 0.03 && p.y > -0.04) { // leave just correct hight
        out->push_back(p);
      }
    }
  }

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_remove_centroid_based(CloudPtr in, Eigen::Vector4f centroid)
{
  float distance = 0.05;

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
ConveyorPoseThread::cloud_remove_offset_to_front(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll)
{
  double space = 0.06;
  double z_min, z_max;
  if ( use_ll ) {
    Eigen::Vector3f c = laserline_get_center_transformed(ll);
    z_min = c(2) - ( space / 2. );
  } else {
    // get lowest z point
    double lowest_z = 1000;
    for (Point p : *in) {
      if ( p.z < lowest_z ) {
        lowest_z = p.z;
      }
    }

    z_min = lowest_z -0.02;
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
//  logger->log_info(name(), "%f => %f - %f", c(0), x_min, x_max);

  CloudPtr out(new Cloud);
  for (Point p : *in) {
    if ( p.x >= x_min && p.x <= x_max ) {
      out->push_back(p);
    }
  }

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_get_plane(CloudPtr in, pcl::ModelCoefficients::Ptr coeff)
{
  // get planes
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
//    seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.0015);

  seg.setInputCloud (in);
  seg.segment (*inliers, *coeff);

  if (inliers->indices.size () == 0) {
    logger->log_error(name(), "Could not estimate a planar model for the given dataset.");
    return CloudPtr();
  } else {
    // get plane
//      ROS_INFO("%sGot plane with %lu inliers", node_name_.c_str(), inliers->indices.size ());
  }

  // get inliers
  CloudPtr out(new Cloud);
  for (size_t i = 0; i < inliers->indices.size (); ++i) {
    Point p;
    p.x = in->points[inliers->indices[i]].x;
    p.y = in->points[inliers->indices[i]].y;
    p.z = in->points[inliers->indices[i]].z;

    out->push_back(p);
  }

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_cluster(CloudPtr in)
{
  in = cloud_voxel_grid(in);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.0011);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (in);
  ec.extract (cluster_indices);

  CloudPtr biggeset(new Cloud);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      cloud_cluster->points.push_back (in->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (biggeset->size() < cloud_cluster->width) {
      biggeset = cloud_cluster;
    }
  }

  return biggeset;
}

CloudPtr
ConveyorPoseThread::cloud_voxel_grid(CloudPtr in)
{
  float ls = 0.001;
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

void
ConveyorPoseThread::bb_switch_is_enabled()
{
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
    logger->log_info(name(), "*** enabled: %s", rv ? "yes" : "no");
    bb_enable_switch_->set_enabled(rv);
    bb_enable_switch_->write();
  }
  cfg_enable_switch_ = rv;
}

std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion>
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

  std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> ret;
  ret.first = origin;
  ret.second = rot;

  return ret;
}

void
ConveyorPoseThread::tf_send_from_pose_if(std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose)
{
  fawkes::tf::StampedTransform transform;

  transform.frame_id = header_.frame_id;
  transform.child_frame_id = conveyor_frame_id_;
  // TODO use time of header, just don't works with bagfiles
  transform.stamp = fawkes::Time((long)header_.stamp);

  transform.setOrigin(pose.first);
  transform.setRotation(pose.second);

  tf_publisher->send_transform(transform);
}

void
ConveyorPoseThread::pose_write(std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose)
{
  double translation[3], rotation[4];
  translation[0] = pose.first.x();
  translation[1] = pose.first.y();
  translation[2] = pose.first.z();
  rotation[0] = pose.second.x();
  rotation[1] = pose.second.y();
  rotation[2] = pose.second.z();
  rotation[3] = pose.second.w();
  bb_pose_->set_translation(translation);
  bb_pose_->set_rotation(rotation);

  bb_pose_->set_frame(header_.frame_id.c_str());
  fawkes::Time stamp((long)header_.stamp);
  bb_pose_->set_visibility_history(vis_hist_);
  bb_pose_->write();
}







