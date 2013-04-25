/***************************************************************************
 *  plugin_template_thread.cpp - template thread
 *
 *  Created: Mi 23. Mai 17:44:14 CEST 2012
 *  Copyright  2012 Daniel Ewert 
 *
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

#include "voltagelogger_thread.h"

#include <interfaces/BatteryInterface.h>

#include <cmath>

#define FORWARD_SPEED 1.0f
#define SIDEWARD_SPEED 1.0f
#define ROTATIONAL_SPEED 1.0f

using namespace fawkes;

/** @class PluginTemplateThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
VoltageLoggerThread::VoltageLoggerThread() :
		Thread("PluginTemplateThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void VoltageLoggerThread::init() {

	logger->log_info(name(), "Plugin Template starts up");
	bat_if_ = blackboard->open_for_reading<BatteryInterface>("Robotino");
}

bool VoltageLoggerThread::prepare_finalize_user() {
	return true;
}

void VoltageLoggerThread::finalize() {
	blackboard->close(bat_if_);
}


void VoltageLoggerThread::loop() {
	bat_if_->read();
	bat_if_->voltage();

}

