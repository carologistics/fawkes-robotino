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

#include "robotino_worldmodel_thread.h"

#include <interfaces/MotorInterface.h>

#include <cmath>


using namespace fawkes;

/** @class RobotinoWorldModelThread "robotino_worldmodel_thread.h"
 * Testing if the worldmodel works
 * @author Daniel Ewert
 */

/** Constructor. */
RobotinoWorldModelThread::RobotinoWorldModelThread() :
		Thread("RobotinoWorldModelThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void RobotinoWorldModelThread::init() {
	logger->log_info(name(), "Plugin Template starts up");
	wm_if_ = blackboard->open_for_writing<RobotinoWorldModelInterface>("Model fll");
}

bool RobotinoWorldModelThread::prepare_finalize_user() {
	return true;
}

void RobotinoWorldModelThread::finalize() {
	blackboard->close(wm_if_);
}

void RobotinoWorldModelThread::loop() {

}

