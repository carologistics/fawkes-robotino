/***************************************************************************
 *  workpiece_pose_thread.cpp - workpiece_pose thread
 *
 *  Created: Sat 15 May 16:28:00 CEST 2021
 *  Copyright  2021 Sebastian Eltester
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

#include "workpiece_pose_thread.h"

#include <core/exceptions/system.h>
#include <interfaces/ConveyorPoseInterface.h>
#include <interfaces/LaserLineInterface.h>
#include <interfaces/SwitchInterface.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <tf/types.h>
#include <utils/math/angle.h>
#include <utils/time/clock.h>

#include <cmath>
#include <cstdio>

using namespace fawkes;

/** @class WorkpiecePoseThread "workpiece_pose_thread.cpp"
 * Plugin to to match a Yolo object detection bounding box center to pointcloud (captured from Intel
 * RealSense)
 * @author Sebastian Eltester
 */

/** Constructor. */
WorkpiecePoseThread::WorkpiecePoseThread()
: Thread("WorkpiecePoseThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  ConfigurationChangeHandler(CFG_PREFIX),
  fawkes::TransformAspect(fawkes::TransformAspect::BOTH, "workpiece_pose"),
  cloud_out_raw_name_("raw"),
  cloud_out_trimmed_name_("trimmed"),
  realsense_switch_(nullptr)
{
}

void
WorkpiecePoseThread::init()
{
	config->add_change_handler(this);

	cloud_in_name_ = config->get_string(CFG_PREFIX "/cloud_in");

	cfg_if_prefix_ = config->get_string(CFG_PREFIX "/if/prefix");
	if (cfg_if_prefix_.back() != '/')
		cfg_if_prefix_.append("/");

	wp_pose_ =
	  blackboard->open_for_writing<WorkpiecePoseInterface>((cfg_if_prefix_ + "status").c_str());
	wp_pose_->set_current_mps_type(wp_pose_->DEFAULT_TYPE);
	wp_pose_->set_current_mps_target(wp_pose_->DEFAULT_TARGET);

	wp_pose_->write();

	cfg_enable_switch_ = config->get_bool(CFG_PREFIX "/switch_default");

	cfg_bb_realsense_switch_name_ =
	  config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");
	wait_time_ = Time(double(config->get_float_or_default(CFG_PREFIX "/realsense_wait_time", 1.0f)));

	trimmed_scene_.reset(new Cloud());

	cloud_in_registered_ = false;

	cloud_out_raw_     = new Cloud();
	cloud_out_trimmed_ = new Cloud();
	cloud_out_model_   = new Cloud();
	pcl_manager->add_pointcloud(cloud_out_raw_name_.c_str(), cloud_out_raw_);
	pcl_manager->add_pointcloud(cloud_out_trimmed_name_.c_str(), cloud_out_trimmed_);
	pcl_manager->add_pointcloud("model", cloud_out_model_);

	realsense_switch_ =
	  blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());
}

void
WorkpiecePoseThread::finalize()
{
	pcl_manager->remove_pointcloud(cloud_out_raw_name_.c_str());
	pcl_manager->remove_pointcloud(cloud_out_trimmed_name_.c_str());
	pcl_manager->remove_pointcloud("model");
	blackboard->close(realsense_switch_);
	blackboard->close(wp_pose_);
}

void
WorkpiecePoseThread::loop()
{
	//-- skip processing if camera is not enabled
	realsense_switch_->read();

	// Check for Messages in ConveyorPoseInterface and update information if
	// needed
	while (!wp_pose_->msgq_empty()) {
		if (wp_pose_->msgq_first_is<ConveyorPoseInterface::RunICPMessage>()) {
			// Update station related information
			logger->log_info(name(), "Received RunICPMessage");
			ConveyorPoseInterface::RunICPMessage *msg =
			  wp_pose_->msgq_first<ConveyorPoseInterface::RunICPMessage>();

			result_pose_.release();
		}
		wp_pose_->msgq_pop();
	}

	if (need_to_wait()) {
		logger->log_debug(name(),
		                  "Waiting for %s for %f sec, still %f sec remaining",
		                  cfg_bb_realsense_switch_name_.c_str(),
		                  wait_time_.in_sec(),
		                  (wait_start_ + wait_time_ - Time()).in_sec());
		return;
	}

	if (!realsense_switch_->is_enabled()) {
		logger->log_warn(name(), "Waiting for RealSense camera to be enabled");
		return;
	}

	if (update_input_cloud()) {
		CloudPtr cloud_in(new Cloud(**cloud_in_));

		size_t   in_size  = cloud_in->points.size();
		CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
		size_t   out_size = cloud_vg->points.size();
		if (in_size == out_size) {
			logger->log_error(name(), "Voxel Grid failed, skipping loop!");
			return;
		}

		trimmed_scene_ = cloud_trim(cloud_vg);

		cloud_publish(cloud_in, cloud_out_raw_);
		cloud_publish(trimmed_scene_, cloud_out_trimmed_);
	} // ! cfg_record_model_
} // update_input_cloud()

bool
WorkpiecePoseThread::update_input_cloud()
{
	if (pcl_manager->exists_pointcloud(cloud_in_name_.c_str())) {
		if (!cloud_in_registered_) { // do I already have this pc
			cloud_in_ = pcl_manager->get_pointcloud<Point>(cloud_in_name_.c_str());
			if (cloud_in_->points.size() > 0) {
				cloud_in_registered_ = true;
			}
		}

		unsigned long time_old = input_pc_header_.stamp;
		input_pc_header_       = cloud_in_->header;

		return time_old != input_pc_header_.stamp; // true, if there is a new cloud, false otherwise

	} else {
		logger->log_debug(name(), "can't get pointcloud %s", cloud_in_name_.c_str());
		cloud_in_registered_ = false;
		return false;
	}
}

WorkpiecePoseThread::CloudPtr
WorkpiecePoseThread::cloud_voxel_grid(WorkpiecePoseThread::CloudPtr in)
{
	float                                    ls = cfg_voxel_grid_leaf_size_;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
	CloudPtr                                 out(new Cloud);
	vg.setInputCloud(in);
	// logger->log_debug(name(), "voxel leaf size is %f", ls);
	vg.setLeafSize(ls, ls, ls);
	vg.filter(*out);
	out->header = in->header;
	return out;
}

void
WorkpiecePoseThread::cloud_publish(WorkpiecePoseThread::CloudPtr cloud_in,
                                   fawkes::RefPtr<Cloud>         cloud_out)
{
	**cloud_out       = *cloud_in;
	cloud_out->header = input_pc_header_;
}

void
WorkpiecePoseThread::pose_write()
{
	if (!result_pose_) {
		logger->log_error(name(), "BUG: calling pose_write() when result_pose_ is unset!");
		return;
	}
	wp_pose_->set_translation(0, result_pose_->getOrigin().getX());
	wp_pose_->set_translation(1, result_pose_->getOrigin().getY());
	wp_pose_->set_translation(2, result_pose_->getOrigin().getZ());
	wp_pose_->set_rotation(0, result_pose_->getRotation().getX());
	wp_pose_->set_rotation(1, result_pose_->getRotation().getY());
	wp_pose_->set_rotation(2, result_pose_->getRotation().getZ());
	wp_pose_->set_rotation(3, result_pose_->getRotation().getW());

	wp_pose_->set_frame(result_pose_->frame_id.c_str());
	long timestamp[2];
	result_pose_->stamp.get_timestamp(timestamp[0], timestamp[1]);
	wp_pose_->set_input_timestamp(timestamp);
	wp_pose_->write();
}

void
WorkpiecePoseThread::pose_publish_tf(const tf::Stamped<tf::Pose> &pose)
{
	// transform data into gripper frame (this is better for later use)
	tf::Stamped<tf::Pose> tf_pose_gripper;
	tf_listener->transform_pose("gripper", pose, tf_pose_gripper);

	// publish the transform from the gripper to the conveyor
	tf::Transform        transform(tf_pose_gripper.getRotation(), tf_pose_gripper.getOrigin());
	tf::StampedTransform stamped_transform(transform,
	                                       tf_pose_gripper.stamp,
	                                       tf_pose_gripper.frame_id,
	                                       workpiece_frame_id_);
	tf_publisher->send_transform(stamped_transform);
}

void
WorkpiecePoseThread::start_waiting()
{
	wait_start_ = Time();
}

bool
WorkpiecePoseThread::need_to_wait()
{
	return Time() < wait_start_ + wait_time_;
}

void
WorkpiecePoseThread::bb_set_busy(bool busy)
{
	wp_pose_->set_busy(busy);
	wp_pose_->write();
}

float
WorkpiecePoseThread::cloud_resolution() const
{
	return cfg_voxel_grid_leaf_size_;
}

Eigen::Matrix4f
pose_to_eigen(const fawkes::tf::Pose &pose)
{
	const tf::Matrix3x3 &rot   = pose.getBasis();
	const tf::Vector3 &  trans = pose.getOrigin();
	Eigen::Matrix4f      rv(Eigen::Matrix4f::Identity());
	rv.block<3, 3>(0, 0) << float(rot[0][0]), float(rot[0][1]), float(rot[0][2]), float(rot[1][0]),
	  float(rot[1][1]), float(rot[1][2]), float(rot[2][0]), float(rot[2][1]), float(rot[2][2]);
	rv.block<3, 1>(0, 3) << float(trans[X_DIR]), float(trans[Y_DIR]), float(trans[Z_DIR]);
	return rv;
}

fawkes::tf::Pose
eigen_to_pose(const Eigen::Matrix4f &m)
{
	fawkes::tf::Pose rv;
	rv.setOrigin({double(m(0, 3)), double(m(1, 3)), double(m(2, 3))});
	rv.setBasis({double(m(0, 0)),
	             double(m(0, 1)),
	             double(m(0, 2)),
	             double(m(1, 0)),
	             double(m(1, 1)),
	             double(m(1, 2)),
	             double(m(2, 0)),
	             double(m(2, 1)),
	             double(m(2, 2))});
	return rv;
}
