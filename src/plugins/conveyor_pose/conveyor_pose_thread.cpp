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
  cloud_in_registered_ = false;
  cfg_enable_switch_ = true;
  cfg_use_visualisation_ = true;

  cloud_out_plane_ = new Cloud();
  cloud_out_result_ = new Cloud();
  pcl_manager->add_pointcloud("conveyor_pose/plane", cloud_out_plane_);
  pcl_manager->add_pointcloud("conveyor_pose/result", cloud_out_result_);

  bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>("/conveyor-pose");
  bb_enable_switch_->set_enabled(cfg_enable_switch_);
  bb_enable_switch_->write();

  visualisation_ = new Visualisation(rosnode);
}

void
ConveyorPoseThread::finalize()
{
  delete visualisation_;
  blackboard->close(bb_enable_switch_);
}

void
ConveyorPoseThread::loop()
{
  bb_switch_is_enabled();
  if ( ! cfg_enable_switch_ ) { return; }
  if ( ! pc_in_check() ) { return; }

  CloudPtr cloud_in(new Cloud(**cloud_in_));

  CloudPtr cloud_pt = cloud_remove_gripper(cloud_in);
  CloudPtr cloud_front = cloud_remove_offset_to_front(cloud_pt);
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  CloudPtr cloud_pl = cloud_get_plane(cloud_front, coeff);
  if ( cloud_pl == NULL ) { return; }
  cloud_publish(cloud_pl, cloud_out_plane_);

  CloudPtr biggest = cloud_cluster(cloud_pl);
  cloud_publish(biggest, cloud_out_result_);

  // get centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<Point, float>(*biggest, centroid);

  Eigen::Vector3f normal;
  normal(0) = coeff->values[0];
  normal(1) = coeff->values[1];
  normal(2) = coeff->values[2];
  tf_send(centroid, normal);
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
    if ( ! (is_inbetween(-0.025, -0.055, p.y) && p.z < 0.11) )  { // remove gripper
      if (p.y < 0.03 && p.y > -0.04) { // leave just correct hight
        out->push_back(p);
      }
    }
  }

  return out;
}

CloudPtr
ConveyorPoseThread::cloud_remove_offset_to_front(CloudPtr in)
{
  // get lowest z point
  CloudPtr out(new Cloud);
  double lowest_z = 1000;
  for (Point p : *in) {
    if ( p.z < lowest_z ) {
      lowest_z = p.z;
    }
  }
  // get all points within 5cm of the clostest points
  for (Point p : *in) {
    if ( p.z - lowest_z < 0.05) {
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
  seg.setDistanceThreshold (0.0025);

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
  in = vg(in);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.0005);
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
ConveyorPoseThread::vg(CloudPtr in)
{
  float ls = 0.00025;
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

void
ConveyorPoseThread::tf_send(Eigen::Vector4f centroid, Eigen::Vector3f normal)
{
  fawkes::tf::StampedTransform transform;

  transform.frame_id = header_.frame_id;
  transform.child_frame_id = "conveyor";
  // TODO use time of header, just don't works with bagfiles
//  transform.stamp = fawkes::Time((long)header_.stamp);

  fawkes::tf::Vector3 origin(centroid(0), centroid(1), centroid(2));

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

  fawkes::tf::Quaternion rot(q.x(), q.y(), q.z(), q.w());

  transform.setOrigin(origin);
  transform.setRotation(rot);

  tf_publisher->send_transform(transform);
}









