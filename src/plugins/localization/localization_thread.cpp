/***************************************************************************
 *  localization_thread.cpp - localization thread
 *
 *  Created: Do 31. Mai 00:27:39 CEST 2012
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

#include "localization_thread.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/Position3DInterface.h>
#include <utils/time/time.h>

#include <cmath>

using namespace fawkes;

/** @class LocalizationThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
LocalizationThread::LocalizationThread() :
		Thread("LocalizationThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void LocalizationThread::init() {

	logger->log_info(name(), "Localization starts up");
	laser_if_ = blackboard->open_for_reading<Laser360Interface>("Laser urg");
	pos3d_if_ = blackboard->open_for_writing<Position3DInterface>("Robotino");
}

bool LocalizationThread::prepare_finalize_user() {
	return true;
}

void LocalizationThread::finalize() {
	blackboard->close(laser_if_);
	blackboard->close(pos3d_if_);
}

void LocalizationThread::loop() {

	//fetching laser data
	laser_if_->read();
	float* distances = laser_if_->distances();
	char* frame_id = laser_if_->frame();
	const Time *timestamp_laser = laser_if_->timestamp();
	size_t maxlen_laser = laser_if_->maxlenof_distances();

	//output generation
	logger->log_info(name(), "Received laser data at %s",
			timestamp_laser->str(true));
	for (size_t i=0;i<maxlen_laser;i++){
		logger->log_info(name(), "Distance at [%i]: %f",i,distances[i]);
	}



	//writing the robotino position
	double translation[3] = { 0.0, 0.0, 0.0 };
	pos3d_if_->set_translation(translation);
	double rotation[4] = { 0.0, 0.0, 0.0, 1.0 };
	pos3d_if_->set_rotation(rotation);

	pos3d_if_->set_frame(frame_id);
	pos3d_if_->write();
}

