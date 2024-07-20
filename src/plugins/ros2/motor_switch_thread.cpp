/*****************************************************************************
 * motor_switch_thread.cpp
 -------------------
 Copyright (C) 2024 by Tim Wendt

 *****************************************************************************

 This program is free software: you can redistribute it and/or modify it under
 the terms of the GNU General Public License as published by the Free Software
 Foundation, either version 3 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 details.

 You should have received a copy of the GNU General Public License along with
 this program.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/
#include "motor_switch_thread.h"

#include "interfaces/SwitchInterface.h"

using namespace fawkes;
using SetOmniDriveEnabled = rto_msgs::srv::SetOmniDriveEnabled;
using SetVelLimits        = rto_msgs::srv::SetVelLimits;

ROS2MotorSwitchThread::ROS2MotorSwitchThread()
: Thread("ROS2MotorSwitchThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("ROS2MotorSwitchThread")
{
}

void
ROS2MotorSwitchThread::init()
{
	switch_client_ = node_handle->create_client<SetOmniDriveEnabled>(
	  std::string(node_handle->get_namespace()) + "/cmd_vel_enable");
	throttle_client_ = node_handle->create_client<SetVelLimits>(
	  std::string(node_handle->get_namespace()) + "/set_velocity_limits");
	goalparam_client_ = node_handle->create_client<SetParameters>(
	  std::string(node_handle->get_namespace()) + "/controller_server/set_parameters");
	while (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
		logger->log_warn(name(), "Switch service not available, waiting");
	}
	while (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
		logger->log_warn(name(), "Switch service not available, waiting");
	}
	while (!goalparam_client_->wait_for_service(std::chrono::seconds(5))) {
		logger->log_warn(name(), "Goalparam-set service not available, waiting");
	}

	switch_if_ = NULL;
	switch_if_ = blackboard->open_for_writing<SwitchInterface>("motor-switch");
	switch_if_->set_enabled(true);
	switch_if_->write();
	throttle_if_ = blackboard->open_for_writing<SwitchInterface>("motor-throttle");
	throttle_if_->set_enabled(false);
	throttle_if_->write();
	goalparam_if_ = blackboard->open_for_writing<SwitchInterface>("setgoal-param");
	goalparam_if_->set_enabled(false);
	goalparam_if_->write();
	bbil_add_message_interface(switch_if_);
	bbil_add_message_interface(throttle_if_);
	bbil_add_message_interface(goalparam_if_);

	blackboard->register_listener(this);
}

void
ROS2MotorSwitchThread::finalize()
{
}

void
ROS2MotorSwitchThread::enable_motor()
{
	auto request    = std::make_shared<SetOmniDriveEnabled::Request>();
	request->enable = true;

	using ServiceResponseFuture     = rclcpp::Client<SetOmniDriveEnabled>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto response = future.get();
		if (!response->success) {
			logger->log_error(name(), "Failed to enable robotino motors");
		}
	};
	auto future_result = switch_client_->async_send_request(request, response_received_callback);
}

void
ROS2MotorSwitchThread::disable_motor()
{
	auto request    = std::make_shared<SetOmniDriveEnabled::Request>();
	request->enable = false;

	using ServiceResponseFuture     = rclcpp::Client<SetOmniDriveEnabled>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto response = future.get();
		if (!response->success) {
			logger->log_error(name(), "Failed to disable robotino motors");
		}
	};
	auto future_result = switch_client_->async_send_request(request, response_received_callback);
}

void
ROS2MotorSwitchThread::drive_fast()
{
	auto request                    = std::make_shared<SetVelLimits::Request>();
	using ServiceResponseFuture     = rclcpp::Client<SetVelLimits>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto response = future.get();
		if (!response->success) {
			logger->log_error(name(), "Failed to unthrottle the motor");
		}
	};
	request->max_linear_vel  = 3.0;
	request->min_linear_vel  = 0.02;
	request->max_angular_vel = 3.0;
	request->min_angular_vel = 0.07;
	auto future_result = throttle_client_->async_send_request(request, response_received_callback);
}

void
ROS2MotorSwitchThread::drive_slow()
{
	auto request = std::make_shared<SetVelLimits::Request>();

	using ServiceResponseFuture     = rclcpp::Client<SetVelLimits>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto response = future.get();
		if (!response->success) {
			logger->log_error(name(), "Failed to throttle the motor");
		}
	};
	request->max_linear_vel  = 0.2;
	request->min_linear_vel  = 0.02;
	request->max_angular_vel = 1.0;
	request->min_angular_vel = 0.07;
	auto future_result = throttle_client_->async_send_request(request, response_received_callback);
}

void
ROS2MotorSwitchThread::low_tolerance()
{
	auto request                    = std::make_shared<SetParameters::Request>();
	using ServiceResponseFuture     = rclcpp::Client<SetParameters>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto response = future.get();
		if (!response->successful) {
			logger->log_error(name(), "Failed to set the low goal tolerance");
		}
	};
	request->goal_checker.goal_checker  = 0.25;
	request->goal_checker.yaw_goal_tolerance  = 0.25;
	
	auto future_result = goalparam_client_->async_send_request(request, response_received_callback);
}

void
ROS2MotorSwitchThread::high_tolerance()
{
	auto request                    = std::make_shared<SetParameters::Request>();
	using ServiceResponseFuture     = rclcpp::Client<SetParameters>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto response = future.get();
		if (!response->successful) {
			logger->log_error(name(), "Failed to set the low goal tolerance");
		}
	};
	request->goal_checker.goal_checker  = 0.75;
	request->goal_checker.yaw_goal_tolerance  = 0.25;
	
	auto future_result = goalparam_client_->async_send_request(request, response_received_callback);
}

bool
ROS2MotorSwitchThread::bb_interface_message_received(fawkes::Interface *interface,
                                                     fawkes::Message   *message) throw()
{
	if (message->is_of_type<SwitchInterface::SetMessage>()) {
		SwitchInterface::SetMessage *msg = (SwitchInterface::SetMessage *)message;
		if (interface == switch_if_) {
			if (msg->is_enabled()) {
				switch_if_->set_enabled(true);
				enable_motor();
			} else {
				switch_if_->set_enabled(false);
				disable_motor();
			}
			switch_if_->write();
		}
		if (interface == throttle_if_) {
			if (msg->is_enabled()) {
				throttle_if_->set_enabled(true);
				drive_slow();
			} else {
				throttle_if_->set_enabled(false);
				drive_fast();
			}
			throttle_if_->write();
		}
		if (interface == goalparam_if_) {
			if (msg->is_enabled()) {
				goalparam_if_->set_enabled(true);
				high_tolerance();
			} else {
				throttle_if_->set_enabled(false);
				low_tolerance();
			}
			throttle_if_->write();
		}
	}
	if (message->is_of_type<SwitchInterface::EnableSwitchMessage>()) {
		if (interface == switch_if_) {
			switch_if_->set_enabled(true);
			enable_motor();
			switch_if_->write();
		}
		if (interface == throttle_if_) {
			throttle_if_->set_enabled(true);
			drive_slow();
			throttle_if_->write();
		}
		if (interface == goalparam_if_) {
			throttle_if_->set_enabled(true);
			high_tolerance();
			throttle_if_->write();
		}
	}

	if (message->is_of_type<SwitchInterface::DisableSwitchMessage>()) {
		if (interface == switch_if_) {
			switch_if_->set_enabled(false);
			disable_motor();
			switch_if_->write();
		}
		if (interface == throttle_if_) {
			throttle_if_->set_enabled(false);
			drive_fast();
			throttle_if_->write();
		}
		if (interface == goalparam_if_) {
			throttle_if_->set_enabled(true);
			low_tolerance();
			throttle_if_->write();
		}
	}

	return true;
}
