/***************************************************************************
 *  gazsim_machine_signal_plugin.cpp - Plugin provides
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

#include "gazsim_machine_signal_thread.h"

#include <aspect/logging.h>
#include <interfaces/RobotinoLightInterface.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <math.h>
#include <stdio.h>

using namespace fawkes;
using namespace gazebo;

/** @class MachineSignalSimThread "gazsim_machine_signal_thread.h"
 * Thread simulates the Light-Front Plugin in Gazebo
 *
 * Gazebo sends the ground truth detected light signals
 * This plugin writes this information into the blackboard
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
MachineSignalSimThread::MachineSignalSimThread()
: Thread("MachineSignalSimThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void
MachineSignalSimThread::init()
{
	logger->log_debug(name(), "Initializing Simulation of the Machine Signal Plugin");

	// read config values
	light_state_if_name_   = config->get_string("/gazsim/light-front/interface-id-single");
	switch_if_name_        = config->get_string("/gazsim/light-front/interface-id-switch");
	hint_if_name_          = config->get_string("/gazsim/light-front/interface-id-hint");
	delivery_mode_if_name_ = config->get_string("/gazsim/light-front/interface-id-delivery-switch");

	// open interfaces
	light_if_  = blackboard->open_for_writing<RobotinoLightInterface>(light_state_if_name_.c_str());
	switch_if_ = blackboard->open_for_writing<fawkes::SwitchInterface>(switch_if_name_.c_str());
	hint_if_   = blackboard->open_for_writing<SignalHintInterface>(hint_if_name_.c_str());
	delivery_if_ =
	  blackboard->open_for_writing<fawkes::SwitchInterface>(delivery_mode_if_name_.c_str());

	// enable plugin by default
	switch_if_->set_enabled(true);
	switch_if_->write();

	// subscribing to gazebo publisher
	light_signals_sub_ =
	  gazebonode->Subscribe(config->get_string("/gazsim/topics/mps-machine-signal"),
	                        &MachineSignalSimThread::on_light_signals_msg,
	                        this);

	new_data_ = false;
}

void
MachineSignalSimThread::finalize()
{
	blackboard->close(light_if_);
	blackboard->close(switch_if_);
}

void
MachineSignalSimThread::loop()
{
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

	// consume messages to the hint interface (they are not used in the
	// simulation)
	while (!hint_if_->msgq_empty()) {
		hint_if_->msgq_pop();
		hint_if_->write();
	}
	// consume messages to the delivery-mode interface (they are not used in the
	// simulation)
	while (!delivery_if_->msgq_empty()) {
		delivery_if_->msgq_pop();
		delivery_if_->write();
	}

	// write light interface
	if (new_data_) {
		new_data_ = false;

		// stop if the switch is dibabled
		if (!switch_if_->is_enabled()) {
			return;
		}

		// set light interface according to the last message
		set_interface();
	}
}

void
MachineSignalSimThread::on_light_signals_msg(ConstLightSignalDetectionPtr &msg)
{
	// logger->log_info(name(), "Got new Machine Light signals.\n");
	last_msg_.CopyFrom(*msg);
	new_data_ = true;
}

void
MachineSignalSimThread::set_interface()
{
	// read out lights
	fawkes::RobotinoLightInterface::LightState red    = RobotinoLightInterface::OFF;
	fawkes::RobotinoLightInterface::LightState yellow = RobotinoLightInterface::OFF;
	fawkes::RobotinoLightInterface::LightState green  = RobotinoLightInterface::OFF;
	get_signals_from_msg(&red, &yellow, &green);

	// write interface
	light_if_->set_red(red);
	light_if_->set_yellow(yellow);
	light_if_->set_green(green);
	light_if_->set_ready(last_msg_.visible());
	light_if_->set_visibility_history(last_msg_.visibility_history());
	light_if_->write();
}

void
MachineSignalSimThread::get_signals_from_msg(fawkes::RobotinoLightInterface::LightState *red,
                                             fawkes::RobotinoLightInterface::LightState *yellow,
                                             fawkes::RobotinoLightInterface::LightState *green)
{
	// read out lights
	for (int i = 0; i < last_msg_.lights_size(); i++) {
		gazsim_msgs::LightSignalDetection::LightSpec light_spec = last_msg_.lights(i);
		RobotinoLightInterface::LightState           state      = RobotinoLightInterface::OFF;
		switch (light_spec.state()) {
		case gazsim_msgs::LightSignalDetection::OFF: state = RobotinoLightInterface::OFF; break;
		case gazsim_msgs::LightSignalDetection::ON: state = RobotinoLightInterface::ON; break;
		case gazsim_msgs::LightSignalDetection::BLINK: state = RobotinoLightInterface::BLINKING; break;
		}
		switch (light_spec.color()) {
		case gazsim_msgs::LightSignalDetection::RED: *red = state; break;
		case gazsim_msgs::LightSignalDetection::YELLOW: *yellow = state; break;
		case gazsim_msgs::LightSignalDetection::GREEN: *green = state; break;
		}
	}
}
