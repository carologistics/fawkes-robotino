/***************************************************************************
 *  direct_com_thread.h - Arduino com thread for direct communication
 *
 *  Created: Mon Apr 04 11:48:36 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_GIGATINO_ROS_THREAD_H_
#define __PLUGINS_GIGATINO_ROS_THREAD_H_

#include "com_message.h"
#include "gigatino_msgs/action/calibrate.hpp"
#include "gigatino_msgs/action/gripper.hpp"
#include "gigatino_msgs/action/home.hpp"
#include "gigatino_msgs/action/move.hpp"
#include "gigatino_msgs/action/stop.hpp"
#include "gigatino_msgs/msg/feedback.hpp"
#include "interfaces/ArduinoInterface.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interface/interface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <memory>

class GigatinoROSMessage;

namespace fawkes {
class ArduinoInterface;
} // namespace fawkes

class GigatinoROSThread : public fawkes::Thread,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::BlackBoardInterfaceListener,
                          public fawkes::ROS2Aspect

{
	using Home      = gigatino_msgs::action::Home;
	using Calibrate = gigatino_msgs::action::Calibrate;
	using Move      = gigatino_msgs::action::Move;
	using Stop      = gigatino_msgs::action::Stop;
	using Gripper   = gigatino_msgs::action::Gripper;
	using Feedback  = gigatino_msgs::msg::Feedback;

public:
	GigatinoROSThread();
	/**
   * @brief Constructor for the arduino communication thread
   *
   * @param cfg_name Name of the config file
   * @param cfg_prefix Prefix tags to arduino config
   * @param tf_thread Pointer to transform thread
   */
	GigatinoROSThread(std::string &cfg_name, std::string &cfg_prefix);
	virtual ~GigatinoROSThread();

	virtual void init();
	virtual void loop();
	void         initInterface();
	virtual void finalize();

	// For BlackBoardInterfaceListener
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) throw();

private:
	std::string               cfg_prefix_;
	std::string               cfg_name_;
	std::string               cfg_feedback_topic_name_;
	std::string               cfg_home_server_name_;
	std::string               cfg_gripper_server_name_;
	std::string               cfg_move_server_name_;
	std::string               cfg_stop_server_name_;
	std::string               cfg_calibrate_server_name_;
	float                     cfg_x_max_;
	float                     cfg_y_max_;
	float                     cfg_z_max_;
	fawkes::ArduinoInterface *arduino_if_;

	std::mutex feedback_mtx_;
	bool       final_;

	rclcpp::Subscription<Feedback>::SharedPtr   feedback_sub_;
	rclcpp::CallbackGroup::SharedPtr            cb_group_;
	rclcpp_action::Client<Home>::SharedPtr      home_action_client_;
	rclcpp_action::Client<Calibrate>::SharedPtr calibrate_action_client_;
	rclcpp_action::Client<Move>::SharedPtr      move_action_client_;
	rclcpp_action::Client<Stop>::SharedPtr      stop_action_client_;
	rclcpp_action::Client<Gripper>::SharedPtr   gripper_action_client_;

	void load_config();

	template <typename ActionT, typename GoalT, typename GoalOptionsT>
	void
	handle_action_call(std::shared_ptr<rclcpp_action::Client<ActionT>> client_t, GoalT &g)
	{
		using GoalHandleActionT        = rclcpp_action::ClientGoalHandle<ActionT>;
		GoalOptionsT send_goal_options = GoalOptionsT();
		send_goal_options.goal_response_callback =
		  [this](const std::shared_ptr<GoalHandleActionT> goal_handle) {
			  if (!goal_handle) {
				  // should not happen, all goals are accepted!
				  RCLCPP_ERROR(node_handle->get_logger(), "Goal was rejected by server");
				  update_final(true);
			  }
			  // ignore happy case, we set to busy before confirmation
		  };

		send_goal_options.feedback_callback = nullptr;

		send_goal_options.result_callback =
		  [this](const typename GoalHandleActionT::WrappedResult &result) {
			  switch (result.code) {
			  case rclcpp_action::ResultCode::SUCCEEDED:
				  RCLCPP_INFO(node_handle->get_logger(), "Done");
				  break;
			  case rclcpp_action::ResultCode::ABORTED:
				  RCLCPP_ERROR(node_handle->get_logger(), "Goal was aborted");
				  return;
			  case rclcpp_action::ResultCode::CANCELED:
				  RCLCPP_ERROR(node_handle->get_logger(), "Goal was canceled");
				  return;
			  default: RCLCPP_ERROR(node_handle->get_logger(), "Unknown result code"); return;
			  }
			  update_final(true);
		  };
		update_final(false);
		client_t->async_send_goal(g, send_goal_options);
	}

	void update_final(bool final);

	bool handle_home_message();
	bool handle_calibrate_message();
	bool handle_open_gripper_message();
	bool handle_close_gripper_message();
	bool handle_stop_message();
	bool handle_xyz_message(fawkes::ArduinoInterface::MoveXYZAbsMessage *message);

	bool handle_rel_xyz_messag(fawkes::ArduinoInterface::MoveXYZRelMessage *msg);
};

#endif
