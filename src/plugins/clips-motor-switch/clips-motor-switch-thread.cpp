
/***************************************************************************
 *  clips-motor-switch-thread.cpp - Switch motor from CLIPS
 *
 *  Created: Thu Apr 25 12:31:51 2013
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "clips-motor-switch-thread.h"

#include "rto_msgs/srv/set_omni_drive_enabled.hpp"

#include <core/threading/mutex_locker.h>

#include <clipsmm.h>

using namespace fawkes;
using SetOmniDriveEnabled = rto_msgs::srv::SetOmniDriveEnabled;

/** @class ClipsMotorSwitchThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsMotorSwitchThread::ClipsMotorSwitchThread()
: Thread("ClipsMotorSwitchThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("motor-switch"),
  CLIPSFeatureAspect(this)
{
}

/** Destructor. */
ClipsMotorSwitchThread::~ClipsMotorSwitchThread()
{
	envs_.clear();
}

void
ClipsMotorSwitchThread::init()
{
	client_ = node_handle->create_client<SetOmniDriveEnabled>("cmd_vel_enable");
	while (!client_->wait_for_service(std::chrono::seconds(5))) {
		logger->log_warn(name(), "Service not available, waiting");
	}
}

void
ClipsMotorSwitchThread::finalize()
{
}

void
ClipsMotorSwitchThread::clips_context_init(const std::string           &env_name,
                                           LockPtr<CLIPS::Environment> &clips)
{
	envs_[env_name] = clips;

	clips->add_function(
	  "motor-enable",
	  sigc::slot<void>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMotorSwitchThread::clips_motor_enable), env_name)));
	clips->add_function(
	  "motor-disable",
	  sigc::slot<void>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMotorSwitchThread::clips_motor_disable), env_name)));
}

void
ClipsMotorSwitchThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
}

void
ClipsMotorSwitchThread::loop()
{
}

void
ClipsMotorSwitchThread::clips_motor_enable(std::string env_name)
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
ClipsMotorSwitchThread::clips_motor_disable(std::string env_name)
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
