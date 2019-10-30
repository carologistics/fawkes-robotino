
/***************************************************************************
 *  gazsim_gripper_thread.h - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:54:42 2015
 *  Copyright  2015 Frederik Zwilling
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
#include <interfaces/RobotinoSensorInterface.h>
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

	sensor_if_name_  = config->get_string("/gazsim/gripper/sensor-if-name");
	arduino_if_name_ = config->get_string("/gazsim/gripper/arduino-if-name");
}

void
GazsimGripperThread::init()
{
	logger->log_debug(name(), "Initializing Simulation of gripper Plugin");

	load_config();
	moving_ = false;
	cur_x_  = 0.0;
	cur_y_  = 0.0;
	cur_z_  = 0.0;

	//-- initialize publisher objects
	tf_add_publisher(cfg_gripper_dyn_x_frame_id_.c_str());
	dyn_x_pub = tf_publishers[cfg_gripper_dyn_x_frame_id_];

	tf_add_publisher(cfg_gripper_dyn_y_frame_id_.c_str());
	dyn_y_pub = tf_publishers[cfg_gripper_dyn_y_frame_id_];

	tf_add_publisher(cfg_gripper_dyn_z_frame_id_.c_str());
	dyn_z_pub = tf_publishers[cfg_gripper_dyn_z_frame_id_];

	set_gripper_pub_ = gazebonode->Advertise<msgs::Int>(config->get_string("/gazsim/topics/gripper"));
	set_conveyor_pub_ =
	  gazebonode->Advertise<msgs::Int>(config->get_string("/gazsim/topics/conveyor"));
	gripper_has_puck_sub_ =
	  gazebonode->Subscribe(config->get_string("/gazsim/topics/gripper-has-puck"),
	                        &GazsimGripperThread::on_has_puck_msg,
	                        this);

	//open ArduinoInterface for writing
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>(arduino_if_name_.c_str());
	arduino_if_->set_x_position(0);
	arduino_if_->set_y_position(arduino_if_->y_max() / 2.);
	arduino_if_->set_y_position(0);
	arduino_if_->set_x_max(cfg_x_max_);
	arduino_if_->set_y_max(cfg_y_max_);
	arduino_if_->set_z_max(cfg_z_max_);
	arduino_if_->set_final(true);
	arduino_if_->write();

	//open RobotinoInterface where puck sensor connected to first 2 inputs
	sensor_if_ = blackboard->open_for_writing<RobotinoSensorInterface>(sensor_if_name_.c_str());
	sensor_if_->set_digital_in(0, false);
	sensor_if_->set_digital_in(1, false);
	sensor_if_->write();
}

void
GazsimGripperThread::finalize()
{
	blackboard->close(arduino_if_);
	blackboard->close(sensor_if_);
}

void
GazsimGripperThread::loop()
{
	while (!arduino_if_->msgq_empty()) {
		if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
			ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);
			arduino_if_->set_x_position(msg->x());
			arduino_if_->set_y_position(msg->y());
			arduino_if_->set_z_position(msg->z());
		} else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZRelMessage>()) {
			ArduinoInterface::MoveXYZRelMessage *msg = arduino_if_->msgq_first(msg);
			arduino_if_->set_x_position(arduino_if_->x_position() + msg->x());
			arduino_if_->set_y_position(arduino_if_->y_position() + msg->y());
			arduino_if_->set_z_position(arduino_if_->z_position() + msg->z());

			//msgs::Int s;
			//s.set_data(arduino_if_->z_position() + msg->z());
			//set_conveyor_pub_->Publish(s);
			//    } else if (
			//    arduino_if_->msgq_first_is<ArduinoInterface::MoveUpwardsMessage>() )
			//    {
			//      ArduinoInterface::MoveUpwardsMessage *msg =
			//      arduino_if_->msgq_first(msg); arduino_if_->set_z_position(
			//      arduino_if_->z_position() - msg->num_mm() ); msgs::Int s;
			//      s.set_data( - msg->num_mm() );
			//      set_conveyor_pub_->Publish( s );
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
			arduino_if_->set_x_position(0);
			arduino_if_->set_y_position(0);
			arduino_if_->set_z_position(0);
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
		arduino_if_->write();
	}

	if (!moving_) {
		boost::mutex::scoped_lock lock(data_mutex_);
		fawkes::Time              now(clock);

		tf::Quaternion q(0.0, 0.0, 0.0);

		tf::Vector3 v_x(cur_x_, 0.0, 0.0);

		//tf::Vector3 v_y(0.0, (cur_y_ - cfg_y_max_ / 2.), 0.0);
		tf::Vector3 v_y(0.0, cur_y_, 0.0);

		tf::Vector3 v_z(0.0, 0.0, cur_z_);

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
}

void
GazsimGripperThread::send_gripper_msg(int value)
{
	// send message to gazebo
	// 0 means close and 1 open
	msgs::Int msg;
	msg.set_data(value);
	set_gripper_pub_->Publish(msg);
}

void
GazsimGripperThread::on_has_puck_msg(ConstIntPtr &msg)
{
	// 1 means the gripper has a puck 0 not
	sensor_if_->set_digital_in(1, msg->data() > 0);
	sensor_if_->write();
}
