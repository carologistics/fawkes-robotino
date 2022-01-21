
/***************************************************************************
 *  gazsim_gripper_thread.h - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:54:42 2015
 *  Copyright  2015 Frederik Zwilling
 *  Copyright  2019 Mostafa Gomaa
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

#include "gazsim_gripper_thread.h"

#include <config/change_handler.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/ArduinoInterface.h>
#include <utils/math/angle.h>

#include <boost/lexical_cast.hpp>
#include <cmath>
#include <cstdarg>
#include <unistd.h>

using namespace fawkes;
using namespace gazebo;

/** @class GazsimGripperThread "gazsim_gripper_thread.h"
 * Thread simulates the Gripper in Gazebo
 *
 * @author Frederik Zwilling
 */

/** Constructor. */

GazsimGripperThread::GazsimGripperThread()
: Thread("GazsimGripperThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
  TransformAspect(TransformAspect::DEFER_PUBLISHER),
  dyn_x_pub{nullptr},
  dyn_y_pub{nullptr},
  dyn_z_pub{nullptr}
{
	set_name("GazsimGripperThread()");
}

void
GazsimGripperThread::load_config()
{
	cfg_prefix_                    = "/arduino/";
	cfg_gripper_frame_id_          = config->get_string(cfg_prefix_ + "/gripper_frame_id");
	cfg_gripper_origin_x_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_origin_x_frame_id");
	cfg_gripper_origin_y_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_origin_y_frame_id");
	cfg_gripper_origin_z_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_origin_z_frame_id");

	cfg_gripper_dyn_x_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_x_frame_id");
	cfg_gripper_dyn_y_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_y_frame_id");
	cfg_gripper_dyn_z_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_z_frame_id");

	cfg_x_max_ = config->get_float(cfg_prefix_ + "/x_max");
	cfg_y_max_ = config->get_float(cfg_prefix_ + "/y_max");
	cfg_z_max_ = config->get_float(cfg_prefix_ + "/z_max");

	arduino_if_name_ = config->get_string("/gazsim/gripper/arduino-if-name");
}

void
GazsimGripperThread::init()
{
	logger->log_debug(name(), "Initializing Simulation of gripper Plugin");

	load_config();

	//-- initialize subscriber objects
	gripper_pose_sub_   = gazebonode->Subscribe(config->get_string("/gazsim/topics/gripper-pose"),
                                            &GazsimGripperThread::on_gripper_pose_msg,
                                            this);
	gripper_final_sub_  = gazebonode->Subscribe(config->get_string("/gazsim/topics/gripper-final"),
                                             &GazsimGripperThread::on_gripper_final_msg,
                                             this);
	gripper_closed_sub_ = gazebonode->Subscribe(config->get_string("/gazsim/topics/gripper-closed"),
	                                            &GazsimGripperThread::on_gripper_closed_msg,
	                                            this);

	//-- initialize publisher objects
	tf_add_publisher(cfg_gripper_dyn_x_frame_id_.c_str());
	dyn_x_pub = tf_publishers[cfg_gripper_dyn_x_frame_id_];

	tf_add_publisher(cfg_gripper_dyn_y_frame_id_.c_str());
	dyn_y_pub = tf_publishers[cfg_gripper_dyn_y_frame_id_];

	tf_add_publisher(cfg_gripper_dyn_z_frame_id_.c_str());
	dyn_z_pub = tf_publishers[cfg_gripper_dyn_z_frame_id_];

	set_gripper_pub_ = gazebonode->Advertise<gazsim_msgs::GripperCommand>(
	  config->get_string("/gazsim/topics/set-gripper"));

	//open ArduinoInterface for writing
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>(arduino_if_name_.c_str());
	arduino_if_->set_x_position(0);
	arduino_if_->set_y_position(0);
	arduino_if_->set_z_position(0);
	arduino_if_->set_x_max(cfg_x_max_);
	arduino_if_->set_y_max(cfg_y_max_);
	arduino_if_->set_z_max(cfg_z_max_);
	arduino_if_->set_final(true);
	arduino_if_->write();
}

void
GazsimGripperThread::finalize()
{
	blackboard->close(arduino_if_);
}

void
GazsimGripperThread::loop()
{
	while (!arduino_if_->msgq_empty()) {
		if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
			ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);
			send_move_msg(msg->x(), msg->y(), msg->z());
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZRelMessage>()) {
			ArduinoInterface::MoveXYZRelMessage *msg = arduino_if_->msgq_first(msg);
			send_move_msg(arduino_if_->x_position() + msg->x(),
			              arduino_if_->y_position() + msg->y(),
			              arduino_if_->z_position() + msg->z());
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperAbsMessage>()) {
			logger->log_warn(name(),
			                 "%s is not implemented in the simulation.",
			                 arduino_if_->msgq_first()->type());
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperRelMessage>()) {
			logger->log_warn(name(),
			                 "%s is not implemented in the simulation.",
			                 arduino_if_->msgq_first()->type());
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::ToHomeMessage>()) {
			logger->log_warn(name(),
			                 "%s is not implemented in the simulation.",
			                 arduino_if_->msgq_first()->type());
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::CalibrateMessage>()) {
			send_move_msg(0, 0, 0);
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::CloseGripperMessage>()) {
			send_gripper_msg(0);
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::OpenGripperMessage>()) {
			send_gripper_msg(1);
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::StatusUpdateMessage>()) {
			logger->log_warn(name(),
			                 "%s is not implemented in the simulation.",
			                 arduino_if_->msgq_first()->type());
		} else {
			logger->log_warn(name(), "Unknown Arduino message received");
		}
		arduino_if_->msgq_pop();
	}
	update_transfrom();
}

void
GazsimGripperThread::send_gripper_msg(int value)
{
	// send message to gazebo
	// 0 means close and 1 open
	gazsim_msgs::GripperCommand msg;
	msg.set_command(value);
	set_gripper_pub_->Publish(msg);
}

void
GazsimGripperThread::send_move_msg(float x, float y, float z)
{
	// send message to gazebo
	// 2 means move
	gazsim_msgs::GripperCommand msg;
	msg.set_command(2);
	// position is relative to gripper_home frame
	msg.set_x(x);
	msg.set_y(y);
	msg.set_z(z);
	set_gripper_pub_->Publish(msg);
}

void
GazsimGripperThread::on_gripper_pose_msg(ConstPosePtr &msg)
{
	arduino_if_->set_x_position(msg->position().x());
	arduino_if_->set_y_position(msg->position().y());
	arduino_if_->set_z_position(msg->position().z());
	arduino_if_->write();
}

void
GazsimGripperThread::on_gripper_final_msg(ConstIntPtr &msg)
{
	arduino_if_->set_final(msg->data());
	arduino_if_->write();
}

void
GazsimGripperThread::on_gripper_closed_msg(ConstIntPtr &msg)
{
	arduino_if_->set_gripper_closed(msg->data());
	arduino_if_->write();
}

void
GazsimGripperThread::update_transfrom()
{
	boost::mutex::scoped_lock lock(data_mutex_);
	fawkes::Time              now(clock);

	tf::Quaternion q(0.0, 0.0, 0.0);
	tf::Vector3    v_x(arduino_if_->x_position(), 0.0, 0.0);
	tf::Vector3    v_y(0.0, arduino_if_->y_position(), 0.0);
	tf::Vector3    v_z(0.0, 0.0, arduino_if_->z_position());

	tf::Transform tf_pose_gripper_x(q, v_x);
	tf::Transform tf_pose_gripper_y(q, v_y);
	tf::Transform tf_pose_gripper_z(q, v_z);

	tf::StampedTransform stamped_transform_x(tf_pose_gripper_x,
	                                         now.stamp(),
	                                         cfg_gripper_origin_x_frame_id_,
	                                         cfg_gripper_dyn_x_frame_id_);
	tf::StampedTransform stamped_transform_y(tf_pose_gripper_y,
	                                         now.stamp(),
	                                         cfg_gripper_origin_y_frame_id_,
	                                         cfg_gripper_dyn_y_frame_id_);
	tf::StampedTransform stamped_transform_z(tf_pose_gripper_z,
	                                         now.stamp(),
	                                         cfg_gripper_origin_z_frame_id_,
	                                         cfg_gripper_dyn_z_frame_id_);
	dyn_x_pub->send_transform(stamped_transform_x);
	dyn_y_pub->send_transform(stamped_transform_y);
	dyn_z_pub->send_transform(stamped_transform_z);
}
