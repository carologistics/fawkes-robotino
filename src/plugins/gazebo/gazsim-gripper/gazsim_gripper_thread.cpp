
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
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
	set_name("GazsimGripperThread()");
}

void
GazsimGripperThread::init()
{
	logger->log_debug(name(), "Initializing Simulation of the Light Front Plugin");

	sensor_if_name_  = config->get_string("/gazsim/gripper/sensor-if-name");
	arduino_if_name_ = config->get_string("/gazsim/gripper/arduino-if-name");
	cfg_prefix_      = config->get_string("/gazsim/gripper/cfg-prefix");

	set_gripper_pub_ = gazebonode->Advertise<msgs::Int>(config->get_string("/gazsim/topics/gripper"));
	set_conveyor_pub_ =
	  gazebonode->Advertise<msgs::Int>(config->get_string("/gazsim/topics/conveyor"));
	gripper_has_puck_sub_ =
	  gazebonode->Subscribe(config->get_string("/gazsim/topics/gripper-has-puck"),
	                        &GazsimGripperThread::on_has_puck_msg,
	                        this);

	cfg_prefix_ = "/arduino/";
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>(arduino_if_name_.c_str());
	arduino_if_->set_x_position(0);
	arduino_if_->set_y_position(arduino_if_->y_max() / 2.);
	arduino_if_->set_y_position(0);
	arduino_if_->set_x_max(config->get_float(cfg_prefix_ + "x_max"));
	arduino_if_->set_y_max(config->get_float(cfg_prefix_ + "y_max"));
	arduino_if_->set_z_max(config->get_float(cfg_prefix_ + "z_max"));
	arduino_if_->set_final(true);
	arduino_if_->write();

	cfg_prefix_ = "//";
	sensor_if_  = blackboard->open_for_writing<RobotinoSensorInterface>(sensor_if_name_.c_str());
	sensor_if_->set_digital_in(0, false);
	sensor_if_->set_digital_in(1, false);
	sensor_if_->write();
}

void
GazsimGripperThread::finalize()
{
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

			msgs::Int s;
			s.set_data(arduino_if_->z_position() + msg->z());
			set_conveyor_pub_->Publish(s);
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
