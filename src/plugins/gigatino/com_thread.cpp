/***************************************************************************
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
 *                  2022  Matteo Tschesche
 *                  2023  Tim Wendt
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

#include "com_thread.h"

#include "ArduinoSketch/commands.h"
#include "com_message.h"
#include "interface/interface.h"
#include "interface/message.h"
#include "serialport.h"

#include <interfaces/ArduinoInterface.h>

using namespace fawkes;
using Home                = gigatino_msgs::action::Home;
using Calibrate           = gigatino_msgs::action::Calibrate;
using Move                = gigatino_msgs::action::Move;
using Feedback            = gigatino_msgs::msg::Feedback;
using GoalHandleHome      = rclcpp_action::ClientGoalHandle<Home>;
using GoalHandleCalibrate = rclcpp_action::ClientGoalHandle<Calibrate>;
using GoalHandleMove      = rclcpp_action::ClientGoalHandle<Move>;

/** @class GigatinoROSThread "com_thread.h"
 * To communicate with an Arduino Uno via boost::asio, it creates a SerialPort object that spins up a boost::asio::io_service thread and handles the communication. This object will be destroyed once the communication breaks down with the Arduino. This will then trigger a reconnect in the main loop. The main loop will be triggered every second by a wakeup timer.
 * @author Tim Niemueller, Nicolas Limpert, Matteo Tschesche, Tim Wendt
 */

/** Constructor. */
GigatinoROSThread::GigatinoROSThread(std::string &cfg_name, std::string &cfg_prefix)
: Thread("GigatinoROSThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("ArduinoThread(%s)", cfg_prefix.c_str())
{
	cfg_prefix_ = cfg_prefix;
	cfg_name_   = cfg_name;
}
/** Destructor. */
GigatinoROSThread::~GigatinoROSThread()
{
}

void
GigatinoROSThread::init()
{
	load_config();
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

	initInterface();

	bbil_add_message_interface(arduino_if_);

	blackboard->register_listener(this);
	arduino_if_->set_final(true);

	arduino_if_->set_status(ArduinoInterface::IDLE);
	arduino_if_->write();
	cb_group_ = node_handle->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	rclcpp::QoS qos(1);
	qos = qos.best_effort();
	rclcpp::SubscriptionOptions options;
	options.callback_group = cb_group_;
	auto sub_callback      = [this](Feedback::UniquePtr msg) -> void {
    std::scoped_lock lk(feedback_mtx_);
    arduino_if_->set_x_position(msg->stepper_positions[0]);
    arduino_if_->set_y_position(msg->stepper_positions[1]);
    arduino_if_->set_z_position(msg->stepper_positions[2]);
    arduino_if_->set_gripper_closed(msg->servo_positions[0] < 40.0);
    arduino_if_->set_wp_sensed(msg->wp_sensor);
    if (!final_) {
      final_ = !msg->busy;
      arduino_if_->set_final(final_);
    }
    arduino_if_->write();
	};
	feedback_sub_ = node_handle->create_subscription<Feedback>(cfg_feedback_topic_name_,
	                                                           qos,
	                                                           sub_callback,
	                                                           options);
	home_action_client_ =
	  rclcpp_action::create_client<Home>(node_handle, cfg_home_server_name_, cb_group_);
	move_action_client_ =
	  rclcpp_action::create_client<Move>(node_handle, cfg_move_server_name_, cb_group_);
	calibrate_action_client_ =
	  rclcpp_action::create_client<Calibrate>(node_handle, cfg_calibrate_server_name_, cb_group_);
	gripper_action_client_ =
	  rclcpp_action::create_client<Gripper>(node_handle, cfg_gripper_server_name_, cb_group_);
	stop_action_client_ =
	  rclcpp_action::create_client<Stop>(node_handle, cfg_stop_server_name_, cb_group_);
}

//@brief Writes the config values to the Interface
void
GigatinoROSThread::initInterface()
{
	arduino_if_->set_x_max(cfg_x_max_);
	arduino_if_->set_y_max(cfg_y_max_);
	arduino_if_->set_z_max(cfg_z_max_);
	arduino_if_->write();
}

void
GigatinoROSThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->close(arduino_if_);
}

void
GigatinoROSThread::loop()
{
}

void
GigatinoROSThread::load_config()
{
	cfg_home_server_name_      = "gigatino/home";
	cfg_calibrate_server_name_ = "gigatino/calibrate";
	cfg_move_server_name_      = "gigatino/move";
	cfg_feedback_topic_name_   = "gigatino/feedback";
	cfg_stop_server_name_      = "gigatino/stop";
	cfg_gripper_server_name_   = "gigatino/gripper";
}

bool
GigatinoROSThread::handle_home_message()
{
	Home::Goal g;
	handle_action_call<Home, Home::Goal, rclcpp_action::Client<Home>::SendGoalOptions>(
	  home_action_client_, g);
	return true;
}

void
GigatinoROSThread::update_final(bool final)
{
	std::scoped_lock lk(feedback_mtx_);
	arduino_if_->set_final(final);
	arduino_if_->write();
}

bool
GigatinoROSThread::handle_calibrate_message()
{
	Calibrate::Goal g;
	handle_action_call<Calibrate, Calibrate::Goal, rclcpp_action::Client<Calibrate>::SendGoalOptions>(
	  calibrate_action_client_, g);
	return true;
}

bool
GigatinoROSThread::handle_xyz_message(ArduinoInterface::MoveXYZAbsMessage *msg)
{
	Move::Goal g;

	g.relative     = false;
	g.x            = msg->x();
	g.y            = msg->y();
	g.z            = msg->z();
	g.target_frame = (std::string("robotinobase3/") + msg->target_frame()).c_str();
	g.use_gripper  = false;
	handle_action_call<Move, Move::Goal, rclcpp_action::Client<Move>::SendGoalOptions>(
	  move_action_client_, g);
	return true;
}

bool
GigatinoROSThread::handle_close_gripper_message()
{
	Gripper::Goal g;

	g.open = false;
	handle_action_call<Gripper, Gripper::Goal, rclcpp_action::Client<Gripper>::SendGoalOptions>(
	  gripper_action_client_, g);
	return true;
}

bool
GigatinoROSThread::handle_stop_message()
{
	Stop::Goal g;

	handle_action_call<Stop, Stop::Goal, rclcpp_action::Client<Stop>::SendGoalOptions>(
	  stop_action_client_, g);
	return true;
}

bool
GigatinoROSThread::handle_open_gripper_message()
{
	Gripper::Goal g;

	g.open = true;
	handle_action_call<Gripper, Gripper::Goal, rclcpp_action::Client<Gripper>::SendGoalOptions>(
	  gripper_action_client_, g);
	return true;
}

bool
GigatinoROSThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	bool status = false;
	if (message->is_of_type<ArduinoInterface::MoveXYZAbsMessage>()) {
		status = handle_xyz_message((ArduinoInterface::MoveXYZAbsMessage *)message);
	} else if (message->is_of_type<ArduinoInterface::MoveXYZRelMessage>()) {
		status = false;
	} else if (message->is_of_type<ArduinoInterface::ToHomeMessage>()) {
		status = handle_home_message();
	} else if (message->is_of_type<ArduinoInterface::CalibrateMessage>()) {
		status = handle_calibrate_message();
	} else if (message->is_of_type<ArduinoInterface::CalibrateXMessage>()) {
		status = true;
	} else if (message->is_of_type<ArduinoInterface::CloseGripperMessage>()) {
		status = handle_close_gripper_message();
	} else if (message->is_of_type<ArduinoInterface::OpenGripperMessage>()) {
		status = handle_open_gripper_message();
	} else if (message->is_of_type<ArduinoInterface::StatusUpdateMessage>()) {
		status = true;
	} else if (message->is_of_type<ArduinoInterface::StopMessage>()) {
		status = handle_stop_message();
	}
	return status;
}
