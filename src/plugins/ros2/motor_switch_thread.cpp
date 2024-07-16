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

ROS2MotorSwitchThread::ROS2MotorSwitchThread()
: Thread("ROS2MotorSwitchThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("ROS2MotorSwitchThread")
{
}

void
ROS2MotorSwitchThread::init()
{
	client_ = node_handle->create_client<SetOmniDriveEnabled>("cmd_vel_enable");
	while (!client_->wait_for_service(std::chrono::seconds(5))) {
		logger->log_warn(name(), "Service not available, waiting");
	}

	switch_if_ = NULL;
	switch_if_ = blackboard->open_for_writing<SwitchInterface>("motor-switch");
	switch_if_->set_enabled(true);
	switch_if_->write();
	bbil_add_message_interface(switch_if_);

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
	auto future_result = client_->async_send_request(request, response_received_callback);
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
	auto future_result = client_->async_send_request(request, response_received_callback);
}

bool
ROS2MotorSwitchThread::bb_interface_message_received(fawkes::Interface *interface,
                                                     fawkes::Message   *message) throw()
{
	if (message->is_of_type<SwitchInterface::SetMessage>()) {
		SwitchInterface::SetMessage *msg = (SwitchInterface::SetMessage *)message;

		if (msg->is_enabled()) {
			switch_if_->set_enabled(true);
			enable_motor();
		} else {
			switch_if_->set_enabled(false);
			disable_motor();
		}

		switch_if_->write();
	}
	if (message->is_of_type<SwitchInterface::EnableSwitchMessage>()) {
		switch_if_->set_enabled(true);
		enable_motor();
		switch_if_->write();
	}

	if (message->is_of_type<SwitchInterface::DisableSwitchMessage>()) {
		switch_if_->set_enabled(false);
		disable_motor();
		switch_if_->write();
	}

	return true;
}
