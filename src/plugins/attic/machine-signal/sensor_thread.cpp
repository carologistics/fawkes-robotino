/***************************************************************************
 *  sensor_thread.cpp - Machine signal thread that puses data into the interface
 *
 *  Created: Wed Oct 08 13:32:57 2008
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
 *             2012-2013  Johannes Rothe
 *             2014       Victor Mataré
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

#include "sensor_thread.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/Position3DInterface.h>

#include <cmath>
#include <cstdio>
#include <limits.h>
#include <stdlib.h>
#include <string>

using namespace fawkes;
using namespace std;
/** @class MachineSignalSensorThread "sensor_thread.h"
 * Sensor thread.
 * This thread integrates into the Fawkes main loop at the sensor hook and
 * publishes new data when available from the MachineSignalPipelineThread.
 * @author Tim Niemueller
 * @author Johannes Rothe
 * @author Victor Mataré
 */

/** Constructor.
 * @param pipeline_thread MachineSignalPipelineThread to get data from (runs
 * continuously)
 */
MachineSignalSensorThread::MachineSignalSensorThread(MachineSignalPipelineThread *pipeline_thread)
: Thread("MachineSignalSensorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
	pipeline_thread_  = pipeline_thread;
	bb_signal_compat_ = NULL;
}

void
MachineSignalSensorThread::init()
{
	// Open required blackboard interfaces
	for (int i = 0; i < MAX_SIGNALS; i++) {
		std::string iface_name = "/machine-signal/";
		iface_name += std::to_string(i);
		bb_signal_states_.push_back(
		  blackboard->open_for_writing<RobotinoLightInterface>(iface_name.c_str()));
	}
	bb_signal_compat_ = blackboard->open_for_writing<RobotinoLightInterface>("/machine-signal/best");
}

void
MachineSignalSensorThread::finalize()
{
	for (std::vector<RobotinoLightInterface *>::iterator bb_it = bb_signal_states_.begin();
	     bb_it != bb_signal_states_.end();
	     bb_it++) {
		blackboard->close(*bb_it);
	}
	blackboard->close(bb_signal_compat_);
}

void
MachineSignalSensorThread::loop()
{
	if (pipeline_thread_->lock_if_new_data()) {
		std::list<SignalState>           known_signals = pipeline_thread_->get_known_signals();
		std::list<SignalState>::iterator best_signal   = pipeline_thread_->get_best_signal();

		// Go through all known signals...
		std::list<SignalState>::iterator known_signal = known_signals.begin();
		for (size_t i = 0; i < MAX_SIGNALS; i++) {
			if (known_signal != known_signals.end()) {
				// Put their states into the blackboard interfaces
				bb_signal_states_[i]->set_red(known_signal->red);
				bb_signal_states_[i]->set_yellow(known_signal->yellow);
				bb_signal_states_[i]->set_green(known_signal->green);
				bb_signal_states_[i]->set_visibility_history(
				  known_signal->unseen > 1 ? -1 : known_signal->visibility);
				bb_signal_states_[i]->set_ready(known_signal->ready);
				bb_signal_states_[i]->write();

				known_signal++;
			} else {
				bb_signal_states_[i]->set_red(RobotinoLightInterface::LightState::UNKNOWN);
				bb_signal_states_[i]->set_yellow(RobotinoLightInterface::LightState::UNKNOWN);
				bb_signal_states_[i]->set_green(RobotinoLightInterface::LightState::UNKNOWN);
				bb_signal_states_[i]->set_visibility_history(-1);
				bb_signal_states_[i]->set_ready(false);
			}
		}

		bb_signal_compat_->set_red(best_signal->red);
		bb_signal_compat_->set_yellow(best_signal->yellow);
		bb_signal_compat_->set_green(best_signal->green);
		bb_signal_compat_->set_visibility_history(best_signal->unseen > 1 ? -1
		                                                                  : best_signal->visibility);
		bb_signal_compat_->set_ready(best_signal->ready);
		bb_signal_compat_->write();
	}

	if (!pipeline_thread_->is_enabled()) {
		for (size_t i = 0; i < MAX_SIGNALS; i++) {
			bb_signal_states_[i]->set_visibility_history(0);
			bb_signal_states_[i]->set_ready(true);
			bb_signal_states_[i]->write();
		}
		bb_signal_compat_->set_visibility_history(0);
		bb_signal_compat_->set_ready(true);
		bb_signal_compat_->write();
	}

	pipeline_thread_->unlock();
}
