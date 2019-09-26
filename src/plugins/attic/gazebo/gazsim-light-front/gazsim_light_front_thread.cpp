/***************************************************************************
 *  gazsim_light_front_plugin.cpp - Plugin provides
 *     the detected light signals of the llsf-machines
 *
 *  Created: Tue Aug 20 22:33:18 2013
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

#include "gazsim_light_front_thread.h"

#include <aspect/logging.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoLightInterface.h>
#include <llsf_msgs/LightSignals.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/Pose2D.pb.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <math.h>
#include <stdio.h>

using namespace fawkes;
using namespace gazebo;

/** @class LightFrontSimThread "gazsim_light_front_thread.h"
 * Thread simulates the Light-Front Plugin in Gazebo
 *
 * Gazebo sends the light signals of all machines and the plugin determines
 * On which machine the robotino as looking at with laser cluster results
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
LightFrontSimThread::LightFrontSimThread()
: Thread("LightFrontSimThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void
LightFrontSimThread::init()
{
	logger->log_debug(name(), "Initializing Simulation of the Light Front Plugin");

	// read config values
	max_distance_               = config->get_float("/gazsim/light-front/max-distance");
	success_visibility_history_ = config->get_int("/gazsim/light-front/success-visibility-history");
	fail_visibility_history_    = config->get_int("/gazsim/light-front/fail-visibility-history");
	visibility_history_increase_per_update_ =
	  config->get_int("/gazsim/light-front/visibility-history-increase-per-update");
	light_state_if_name_     = config->get_string("/gazsim/light-front/interface-id-single");
	light_pos_if_name_       = config->get_string("/plugins/light_front/light_position_if");
	switch_if_name_          = config->get_string("/gazsim/light-front/interface-id-switch");
	delivery_switch_if_name_ = config->get_string("/gazsim/light-front/interface-id-delivery-switch");
	see_all_delivery_gates_  = config->get_bool("/gazsim/light-front/see-all-delivery-gates");
	interface_id_multiple_   = config->get_string("/gazsim/light-front/interface-id-multiple");
	deliver_x_min_           = config->get_float("/gazsim/light-front/deliver-pos-x-min");
	deliver_y_min_           = config->get_float("/gazsim/light-front/deliver-pos-y-min");
	deliver_y_max_           = config->get_float("/gazsim/light-front/deliver-pos-y-max");
	deliver_ori_max_diff_    = config->get_float("/gazsim/light-front/deliver-ori-max-diff");
	delivery_pose_if_name_   = config->get_string("/gazsim/light-front/interface-id-delivery-pose");

	// open interfaces
	light_if_ = blackboard->open_for_writing<RobotinoLightInterface>(light_state_if_name_.c_str());
	pose_if_  = blackboard->open_for_reading<fawkes::Position3DInterface>(light_pos_if_name_.c_str());
	switch_if_ = blackboard->open_for_writing<fawkes::SwitchInterface>(switch_if_name_.c_str());
	if (see_all_delivery_gates_) {
		light_if_0_ =
		  blackboard->open_for_writing<RobotinoLightInterface>((interface_id_multiple_ + "0").c_str());
		light_if_1_ =
		  blackboard->open_for_writing<RobotinoLightInterface>((interface_id_multiple_ + "1").c_str());
		light_if_2_ =
		  blackboard->open_for_writing<RobotinoLightInterface>((interface_id_multiple_ + "2").c_str());
		deliver_mode_if_ =
		  blackboard->open_for_writing<fawkes::SwitchInterface>(delivery_switch_if_name_.c_str());
		deliver_pose_if_ =
		  blackboard->open_for_writing<Position3DInterface>(delivery_pose_if_name_.c_str());
	}

	// enable plugin by default
	switch_if_->set_enabled(true);
	switch_if_->write();

	// subscribing to gazebo publisher
	light_signals_sub_ = gazebonode->Subscribe(config->get_string("/gazsim/topics/machine-lights"),
	                                           &LightFrontSimThread::on_light_signals_msg,
	                                           this);
	localization_sub_  = gazebonode->Subscribe(config->get_string("/gazsim/topics/gps"),
                                            &LightFrontSimThread::on_localization_msg,
                                            this);

	new_data_            = false;
	current_vis_history_ = 0;
}

void
LightFrontSimThread::finalize()
{
	blackboard->close(light_if_);
	blackboard->close(pose_if_);
	blackboard->close(switch_if_);
	if (see_all_delivery_gates_) {
		blackboard->close(light_if_0_);
		blackboard->close(light_if_1_);
		blackboard->close(light_if_2_);
		blackboard->close(deliver_mode_if_);
		blackboard->close(deliver_pose_if_);
	}
}

void
LightFrontSimThread::loop()
{
	// the acual work takes place in on_light_signals_msg

	// check messages of the switch interface
	while (!switch_if_->msgq_empty()) {
		if (SwitchInterface::DisableSwitchMessage *msg = switch_if_->msgq_first_safe(msg)) {
			switch_if_->set_enabled(false);
		} else if (SwitchInterface::EnableSwitchMessage *msg = switch_if_->msgq_first_safe(msg)) {
			switch_if_->set_enabled(true);
		}
		switch_if_->msgq_pop();
		switch_if_->write();
	}

	// write light interface
	if (new_data_) {
		new_data_ = false;

		// stop if the switch is dibabled
		if (!switch_if_->is_enabled()) {
			return;
		}

		// set light interface of the machine the robot is looking at
		set_interface_of_nearest();

		// if we stand in front of the delivery we have to set the three interfaces
		// to the signals of the deliverygates
		if (see_all_delivery_gates_ && standing_in_front_of_deliver()) {
			set_interfaces_of_gates();
		} else {
			set_interface_unready(light_if_0_);
			set_interface_unready(light_if_1_);
			set_interface_unready(light_if_2_);
			deliver_pose_if_->set_visibility_history(fail_visibility_history_);
			deliver_pose_if_->write();
		}
	}
}

void
LightFrontSimThread::on_light_signals_msg(ConstAllMachineSignalsPtr &msg)
{
	// logger->log_info(name(), "Got new Machine Light signals.\n");
	last_msg_.CopyFrom(*msg);
	new_data_ = true;
}

void
LightFrontSimThread::on_localization_msg(ConstPosePtr &msg)
{
	// logger->log_info(name(), "Got new Localization data.\n");

	// read data from message
	robot_x_ = msg->position().x();
	robot_y_ = msg->position().y();
	// calculate ori from quaternion
	robot_ori_ = tf::get_yaw(tf::Quaternion(msg->orientation().x(),
	                                        msg->orientation().y(),
	                                        msg->orientation().z(),
	                                        msg->orientation().w()));
}

void
LightFrontSimThread::set_interface_of_nearest()
{
	// read cluster position to determine
	// on which machine the robotino is looking at
	//(the cluster returns the position relative to the robots pos and ori)
	pose_if_->read();
	double rel_x = pose_if_->translation(0);
	double rel_y = pose_if_->translation(1);
	// if both is 0 light-front looks direct in front (e.g. when under the rfid)
	if (rel_x == 0 && rel_y == 0) {
		rel_x = 0.28; // distance between laser and light signal when standing under
		              // the rfid
	}
	double look_pos_x = robot_x_ + cos(robot_ori_) * rel_x - sin(robot_ori_) * rel_y;
	double look_pos_y = robot_y_ + sin(robot_ori_) * rel_x + cos(robot_ori_) * rel_y;
	// find machine nearest to the look_pos
	double min_distance = 1000.0;
	int    index_min    = 0;
	for (int i = 0; i < last_msg_.machines_size(); i++) {
		llsf_msgs::MachineSignal machine = last_msg_.machines(i);
		llsf_msgs::Pose2D        pose    = machine.pose();
		double                   x       = pose.x();
		double                   y       = pose.y();
		double                   distance =
		  sqrt((look_pos_x - x) * (look_pos_x - x) + (look_pos_y - y) * (look_pos_y - y));
		if (distance < min_distance) {
			min_distance = distance;
			index_min    = i;
		}
	}

	// if the distance is greater than a threashold, no light is determined
	if (min_distance > max_distance_) {
		// logger->log_info(name(), "Distance between light pos and machine (%f) is
		// too big.\n", min_distance); set ready and visibility history
		set_interface_unready(light_if_);
		// reset visibility history
		current_vis_history_ = 0;
	} else {
		// reset visibility history if we look at another machine
		if (index_min != current_machine_) {
			current_vis_history_ = 0;
			current_machine_     = index_min;
		}
		// reset visibility history if the signal changed
		fawkes::RobotinoLightInterface::LightState red    = RobotinoLightInterface::OFF;
		fawkes::RobotinoLightInterface::LightState yellow = RobotinoLightInterface::OFF;
		fawkes::RobotinoLightInterface::LightState green  = RobotinoLightInterface::OFF;
		get_signals_from_msg(last_msg_.machines(index_min), &red, &yellow, &green);
		if (red != current_signal_red_ || yellow != current_signal_yellow_
		    || green != current_signal_green_) {
			current_vis_history_   = 0;
			current_signal_red_    = red;
			current_signal_yellow_ = yellow;
			current_signal_green_  = green;
		}
		// printf("looking at machine :%s\n",
		// last_msg_.machines(index_min).name().c_str());
		set_interface_to_signal(light_if_, last_msg_.machines(index_min));
		// increase visibility history for next iteration
		current_vis_history_ += visibility_history_increase_per_update_;
	}
}

void
LightFrontSimThread::set_interfaces_of_gates()
{
	std::string green_machine_name;
	if (robot_x_ > 0) {
		// standing in front of the cyan gates
		set_interface_to_signal(light_if_0_, get_machine("D3"));
		set_interface_to_signal(light_if_1_, get_machine("D2"));
		set_interface_to_signal(light_if_2_, get_machine("D1"));
		if (light_if_0_->green() == RobotinoLightInterface::ON)
			green_machine_name = "D3";
		else if (light_if_1_->green() == RobotinoLightInterface::ON)
			green_machine_name = "D2";
		else
			green_machine_name = "D1";
	} else {
		// standing in front of the magenta gates
		set_interface_to_signal(light_if_0_, get_machine("D4"));
		set_interface_to_signal(light_if_1_, get_machine("D5"));
		set_interface_to_signal(light_if_2_, get_machine("D6"));
		if (light_if_0_->green() == RobotinoLightInterface::ON)
			green_machine_name = "D4";
		else if (light_if_1_->green() == RobotinoLightInterface::ON)
			green_machine_name = "D5";
		else
			green_machine_name = "D6";
	}
	// set green light position when looking at a green delivery gate
	llsf_msgs::MachineSignal green_machine = get_machine(green_machine_name.c_str());
	deliver_pose_if_->set_frame("/base_laser");
	deliver_pose_if_->set_visibility_history(success_visibility_history_);
	double rel_x = green_machine.pose().x() - robot_x_;
	double rel_y = green_machine.pose().y() - robot_y_;
	deliver_pose_if_->set_translation(0, cos(-robot_ori_) * rel_x - sin(-robot_ori_) * rel_y);
	deliver_pose_if_->set_translation(1, sin(-robot_ori_) * rel_x + cos(-robot_ori_) * rel_y);
	deliver_pose_if_->set_translation(2, 0.15);
	deliver_pose_if_->write();
}

bool
LightFrontSimThread::standing_in_front_of_deliver()
{
	// we have to stand in the area in front of the deliver
	// with respect to both sides
	if (fabs(robot_x_) < deliver_x_min_ || robot_y_ < deliver_y_min_ || robot_y_ > deliver_y_max_) {
		return false;
	}
	// and look in the right direction
	if (robot_x_ > 0) {
		return fabs(robot_ori_) < deliver_ori_max_diff_;
	} else {
		return fabs(robot_ori_) > (M_PI - deliver_ori_max_diff_);
	}
}

llsf_msgs::MachineSignal
LightFrontSimThread::get_machine(std::string machine_name)
{
	std::string search_string = machine_name + "::"; // because in name stands the name of the link
	for (int i = 0; i < last_msg_.machines_size(); i++) {
		llsf_msgs::MachineSignal machine = last_msg_.machines(i);
		if (machine.name().find(search_string) != std::string::npos) {
			return machine;
		}
	}
	logger->log_error(name(), "Could not find Machine %s in msg.", machine_name.c_str());
	return last_msg_.machines(0);
}

void
LightFrontSimThread::set_interface_to_signal(fawkes::RobotinoLightInterface *interface,
                                             llsf_msgs::MachineSignal        signal)
{
	// read out lights
	fawkes::RobotinoLightInterface::LightState red    = RobotinoLightInterface::OFF;
	fawkes::RobotinoLightInterface::LightState yellow = RobotinoLightInterface::OFF;
	fawkes::RobotinoLightInterface::LightState green  = RobotinoLightInterface::OFF;
	get_signals_from_msg(signal, &red, &yellow, &green);

	// write interface
	interface->set_red(red);
	interface->set_yellow(yellow);
	interface->set_green(green);
	interface->set_ready(true);
	interface->set_visibility_history(current_vis_history_);
	interface->write();
}

void
LightFrontSimThread::set_interface_unready(fawkes::RobotinoLightInterface *interface)
{
	interface->set_ready(false);
	interface->set_visibility_history(fail_visibility_history_);
	interface->write();
}

void
LightFrontSimThread::get_signals_from_msg(llsf_msgs::MachineSignal                    signal_msg,
                                          fawkes::RobotinoLightInterface::LightState *red,
                                          fawkes::RobotinoLightInterface::LightState *yellow,
                                          fawkes::RobotinoLightInterface::LightState *green)
{
	// read out lights
	for (int i = 0; i < signal_msg.lights_size(); i++) {
		llsf_msgs::LightSpec               light_spec = signal_msg.lights(i);
		RobotinoLightInterface::LightState state      = RobotinoLightInterface::OFF;
		switch (light_spec.state()) {
		case llsf_msgs::OFF: state = RobotinoLightInterface::OFF; break;
		case llsf_msgs::ON: state = RobotinoLightInterface::ON; break;
		case llsf_msgs::BLINK: state = RobotinoLightInterface::BLINKING; break;
		}
		switch (light_spec.color()) {
		case llsf_msgs::RED: *red = state; break;
		case llsf_msgs::YELLOW: *yellow = state; break;
		case llsf_msgs::GREEN: *green = state; break;
		}
	}
}
