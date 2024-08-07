/***************************************************************************
 *  gazsim_light_front_plugin.cpp - Plugin provides
 *     the detected light signals of the llsf-machines
 *
 *  Created: Tue Aug 20 22:32:52 2013
 *  Copyright  2013 Frederik Zwilling
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

#ifndef __PLUGINS_GAZSIM_LIGHT_FRONT_THREAD_H_
#define __PLUGINS_GAZSIM_LIGHT_FRONT_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/SwitchInterface.h>
#include <llsf_msgs/LightSignals.pb.h>
#include <plugins/gazebo/aspect/gazebo.h>

#include <string.h>

// from Gazebo
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

typedef const boost::shared_ptr<llsf_msgs::AllMachineSignals const> ConstAllMachineSignalsPtr;

namespace fawkes {
class RobotinoLightInterface;
}

class LightFrontSimThread : public fawkes::Thread,
                            public fawkes::ClockAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::BlackBoardAspect,
                            public fawkes::BlockedTimingAspect,
                            public fawkes::GazeboAspect
{
public:
	LightFrontSimThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	// Subscriber to receive light signal data from gazebo
	gazebo::transport::SubscriberPtr light_signals_sub_;
	// Subscriber to receive localization data from gazebo
	//(light front works relative, and the light signal msgs are absolute)
	gazebo::transport::SubscriberPtr localization_sub_;

	// interface needed to determine position of the light
	fawkes::Position3DInterface *pose_if_;

	// provided interfaces
	fawkes::RobotinoLightInterface *light_if_;
	fawkes::RobotinoLightInterface *light_if_0_;
	fawkes::RobotinoLightInterface *light_if_1_;
	fawkes::RobotinoLightInterface *light_if_2_;
	fawkes::SwitchInterface        *switch_if_;
	fawkes::SwitchInterface        *deliver_mode_if_;
	fawkes::Position3DInterface    *deliver_pose_if_;

	// handler function for incoming messages about the machine light signals
	void on_light_signals_msg(ConstAllMachineSignalsPtr &msg);

	// copy of last msg to write the interface in the next loop
	llsf_msgs::AllMachineSignals last_msg_;

	// handler function for incoming localization data messages
	void on_localization_msg(ConstPosePtr &msg);

	/// set the light interface to the signal state nearest to the cluster
	/// position
	void set_interface_of_nearest();

	/// set the light interfaces of the three delivery gates
	void set_interfaces_of_gates();

	// config value for maximal distance before the plugin says
	// it can not detect any light
	double max_distance_;

	/// does the robot stand in front of the delivery gate?
	bool standing_in_front_of_deliver();

	/// get machine signal of specific machine
	llsf_msgs::MachineSignal get_machine(std::string name);

	/// set interface according to signal
	void set_interface_to_signal(fawkes::RobotinoLightInterface *interface,
	                             llsf_msgs::MachineSignal        signal);

	/// set interface unready and with low visibility history
	void set_interface_unready(fawkes::RobotinoLightInterface *interface);

	/// get light signals of a machine from msg (returns values by reference)
	void get_signals_from_msg(llsf_msgs::MachineSignal                    signal_msg,
	                          fawkes::RobotinoLightInterface::LightState *red,
	                          fawkes::RobotinoLightInterface::LightState *yellow,
	                          fawkes::RobotinoLightInterface::LightState *green);

	// config value for perception at the delivery-gates
	bool        see_all_delivery_gates_;
	std::string interface_id_multiple_;
	// area and ori where we assume the robot looks at the delivery gates (also
	// mirrored for other half)
	float deliver_x_min_;
	float deliver_y_min_;
	float deliver_y_max_;
	float deliver_ori_max_diff_;

	int success_visibility_history_;
	int current_vis_history_;
	int fail_visibility_history_;

	int visibility_history_increase_per_update_;

	// variables needed to reset visibility history
	int                                        current_machine_;
	fawkes::RobotinoLightInterface::LightState current_signal_red_;
	fawkes::RobotinoLightInterface::LightState current_signal_yellow_;
	fawkes::RobotinoLightInterface::LightState current_signal_green_;

	std::string light_pos_if_name_;
	std::string light_state_if_name_;
	std::string switch_if_name_;
	std::string delivery_switch_if_name_;
	std::string delivery_pose_if_name_;

	// the robots position in the simulation
	double robot_x_;
	double robot_y_;
	double robot_ori_;

	// interface values to write in the next loop
	bool new_data_;
};

#endif
