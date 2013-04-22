/***************************************************************************
 *  sensor_thread.cpp - Laser thread that puses data into the interface
 *
 *  Created: Wed Oct 08 13:32:57 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "sensor_thread.h"

#include <interfaces/Position3DInterface.h>

#include <stdlib.h>
#include <cstdio>
#include <cmath>

#include <string>

using namespace fawkes;
using namespace std;
/** @class OmniVisionSensorThread "sensor_thread.h"
 * Laser sensor thread.
 * This thread integrates into the Fawkes main loop at the sensor hook and
 * publishes new data when available from the LaserAcquisitionThread.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 * @param aqt LaserAcquisitionThread to get data from
 */
OmniVisionSensorThread::OmniVisionSensorThread(RobotinoOmniVisionPipelineThread *aqt) :
		Thread("OmniVisionSensorThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS) {
	__aqt = aqt;
	cfg_frame_ = ("/hardware/robotino/omnivision/frame");
}

void OmniVisionSensorThread::init() {

}

void OmniVisionSensorThread::finalize() {

}

void OmniVisionSensorThread::loop() {
	if (__aqt->lock_if_new_data()) {
		//Iterator over the List in the Pipeline Thread whilst iterating over the sensor thread list
		std::list<fawkes::Position3DInterface*>::iterator pipeline_pucks;
		for (pipeline_pucks = __aqt->puck_ifs_.begin(); pipeline_pucks!=__aqt->puck_ifs_.end(); pipeline_pucks++) {
//			char *history;
//			char *translation_x;
//			char *translation_y;
			(*pipeline_pucks)->write();
//			asprintf(&history, "Visbility History: %d",(*pipeline_pucks)->visibility_history());
//			asprintf(&translation_x, "Translation X: %f",(*pipeline_pucks)->translation(0));
//			asprintf(&translation_y, "Translation Y: %f",(*pipeline_pucks)->translation(1));
//			logger->log_debug(name(), history);
//			logger->log_debug(name(), translation_x);
//			logger->log_debug(name(), translation_y);
			// set x and y translation to zero so it will count down below zero when not seen
			(*pipeline_pucks)->set_translation(0,0);
			(*pipeline_pucks)->set_translation(1,0);
		}
	} 
	__aqt->unlock();
}
