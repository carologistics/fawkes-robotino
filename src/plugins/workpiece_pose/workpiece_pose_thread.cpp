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
#include <interfaces/SwitchInterface.h>
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
  realsense_switch_(nullptr)
{
}

void
WorkpiecePoseThread::init()
{
	config->add_change_handler(this);

	wp_pose_ = blackboard->open_for_writing<WorkpiecePoseInterface>("WorkpiecePose");

	wp_pose_->write();

	cfg_enable_switch_ = config->get_bool(CFG_PREFIX "/switch_default");

	cfg_bb_realsense_switch_name_ =
	  config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");

	realsense_switch_ =
	  blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());

	workpiece_frame_id_ = "workpiece";
}

void
WorkpiecePoseThread::finalize()
{
	blackboard->close(realsense_switch_);
	blackboard->close(wp_pose_);
}

void
WorkpiecePoseThread::loop()
{
	if (read_xyz()) {
		result_pose_.release();

		tf::Stamped<tf::Pose> result_pose{fawkes::tf::Pose(), fawkes::Time(), workpiece_frame_id_};

		result_pose_.reset(new tf::Stamped<tf::Pose>{result_pose});

		// set workpiece orientation to orientation of conveyor cam
		fawkes::tf::StampedTransform gripper_pose;
		tf_listener->lookup_transform("cam_conveyor", "world", gripper_pose);
		result_pose_->setRotation(gripper_pose.getRotation());
		result_pose_->setOrigin({msg->translation(0), msg->translation(1), msg->translation(2)});
		pose_write();
		pose_publish_tf(*result_pose_);
	}
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

/**
 * Read XYZ pose and write to result pose
 * @return new_msg
 */
bool
WorkpiecePoseThread::read_xyz()
{
	bool new_msg = false;
	while (!wp_pose_->msgq_empty()) {
		if (wp_pose_->msgq_first_is<WorkpiecePoseInterface::WPPoseMessage>()) {
			msg     = wp_pose_->msgq_first<WorkpiecePoseInterface::WPPoseMessage>();
			new_msg = true;
		} else {
			logger->log_warn(name(), "Unknown message received");
		}
		wp_pose_->msgq_pop();
	}
	return new_msg;
}

void
WorkpiecePoseThread::pose_publish_tf(const tf::Stamped<tf::Pose> &pose)
{
	// transform data into cam conveyor frame
	tf::Stamped<tf::Pose> tf_pose_gripper;
	tf_listener->transform_pose("cam_conveyor", pose, tf_pose_gripper);

	// publish the transform from the gripper to the conveyor
	tf::Transform        transform(tf_pose_gripper.getRotation(), tf_pose_gripper.getOrigin());
	tf::StampedTransform stamped_transform(transform,
	                                       tf_pose_gripper.stamp,
	                                       tf_pose_gripper.frame_id,
	                                       workpiece_frame_id_);
	tf_publisher->send_transform(stamped_transform);
}
