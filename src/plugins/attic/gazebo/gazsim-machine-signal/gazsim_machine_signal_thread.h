/***************************************************************************
 *  gazsim_machine_signal_plugin.cpp - Plugin provides
 *     the detected light signals of the llsf-machines
 *
 *  Created: Thu Apr 02 14:46:52 2015
 *  Copyright  2015 Frederik Zwilling
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

#ifndef __PLUGINS_GAZSIM_MACHINE_SIGNAL_THREAD_H_
#define __PLUGINS_GAZSIM_MACHINE_SIGNAL_THREAD_H_

#include "../msgs/LightSignalDetection.pb.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/SignalHintInterface.h>
#include <interfaces/SwitchInterface.h>
#include <plugins/gazebo/aspect/gazebo.h>

#include <string.h>

// from Gazebo
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

typedef const boost::shared_ptr<gazsim_msgs::LightSignalDetection const>
  ConstLightSignalDetectionPtr;

namespace fawkes {
class RobotinoLightInterface;
}

class MachineSignalSimThread : public fawkes::Thread,
                               public fawkes::ClockAspect,
                               public fawkes::LoggingAspect,
                               public fawkes::ConfigurableAspect,
                               public fawkes::BlackBoardAspect,
                               public fawkes::BlockedTimingAspect,
                               public fawkes::GazeboAspect
{
public:
	MachineSignalSimThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	// Subscriber to receive light signal data from gazebo
	gazebo::transport::SubscriberPtr light_signals_sub_;

	// provided interfaces
	fawkes::RobotinoLightInterface *light_if_;
	fawkes::SwitchInterface *       switch_if_;
	fawkes::SignalHintInterface *   hint_if_;
	fawkes::SwitchInterface *       delivery_if_;

	// handler function for incoming messages about the machine light signals
	void on_light_signals_msg(ConstLightSignalDetectionPtr &msg);

	// copy of last msg to write the interface in the next loop
	gazsim_msgs::LightSignalDetection last_msg_;

	/// set the light interface to the signal state nearest to the cluster
	/// position
	void set_interface();

	/// get light signals of a machine from msg (returns values by reference)
	void get_signals_from_msg(fawkes::RobotinoLightInterface::LightState *red,
	                          fawkes::RobotinoLightInterface::LightState *yellow,
	                          fawkes::RobotinoLightInterface::LightState *green);

	std::string light_state_if_name_;
	std::string switch_if_name_;
	std::string hint_if_name_;
	std::string delivery_mode_if_name_;

	// interface values to write in the next loop
	bool new_data_;
};

#endif
