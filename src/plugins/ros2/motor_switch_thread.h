/*****************************************************************************
 * motor_switch_thread.h
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
#ifndef MOTOR_SWITCH_THREAD_H
#define MOTOR_SWITCH_THREAD_H

#include "rclcpp/rclcpp.hpp"
#include "rto_msgs/srv/set_omni_drive_enabled.hpp"
#include "rto_msgs/srv/set_vel_limits.hpp"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/SwitchInterface.h>
#include <plugins/ros2/aspect/ros2.h>

class ROS2MotorSwitchThread : public fawkes::Thread,
                              public fawkes::LoggingAspect,
                              public fawkes::BlackBoardAspect,
                              public fawkes::BlackBoardInterfaceListener,
                              public fawkes::ROS2Aspect
{
public:
	ROS2MotorSwitchThread();

	virtual void init();
	virtual void finalize();

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void enable_motor();
	void disable_motor();
	void drive_fast();
	void drive_slow();

	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) throw();
	rclcpp::Client<rto_msgs::srv::SetOmniDriveEnabled>::SharedPtr switch_client_;
	rclcpp::Client<rto_msgs::srv::SetVelLimits>::SharedPtr        throttle_client_;
	fawkes::SwitchInterface                                      *switch_if_;
	fawkes::SwitchInterface                                      *throttle_if_;
};
#endif /* MOTOR_SWITCH_THREAD_H */
