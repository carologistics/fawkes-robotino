
/***************************************************************************
 *  laser_front_dist_thread.cpp - laser_front_dist
 *
 *  Plugin created: Thu Jun 23 22:35:32 2016

 *  Copyright  2016  Frederik Zwilling
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

#include "laser_front_dist_thread.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/Position3DInterface.h>
#include <tf/types.h>

#include <cmath>

using namespace fawkes;

/** @class LaserFrontDistThread 'laser_front_dist_thread.h'
 * Calculates distance to object in front with average of laser beams in front
 * @author Frederik Zwilling
 */

LaserFrontDistThread::LaserFrontDistThread()
: Thread("LaserFrontDistThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL),
  TransformAspect(TransformAspect::ONLY_PUBLISHER, "laser-front-dist")
{
}

void
LaserFrontDistThread::init()
{
	// read config values
	beams_used_   = config->get_int("plugins/laser-front-dist/number_beams_used");
	target_frame_ = config->get_string("plugins/laser-front-dist/target_frame");
	// open interfaces
	if_laser_ = blackboard->open_for_reading<Laser360Interface>(
	  config->get_string("plugins/laser-front-dist/input_laser_interface").c_str());
	if_result_ = blackboard->open_for_writing<Position3DInterface>(
	  config->get_string("plugins/laser-front-dist/output_result_interface").c_str());
}

void
LaserFrontDistThread::loop()
{
	// calculate average:
	if_laser_->read();
	float sum = 0.0;
	for (int i = 0; i < beams_used_ / 2 + beams_used_ % 2; i++) {
		if (!std::isnormal(if_laser_->distances(i))) {
			// this is invalid
			if_result_->set_visibility_history(-1);
			if_result_->write();
			return;
		}
		sum += if_laser_->distances(i);
	}
	for (int i = 0; i < beams_used_ / 2; i++) {
		if (!std::isnormal(if_laser_->distances(359 - i))) {
			// this is invalid
			if_result_->set_visibility_history(-1);
			if_result_->write();
			return;
		}
		sum += if_laser_->distances(359 - i);
	}
	float average = sum / (float)beams_used_;
	frame_        = if_laser_->frame();

	// publish transform
	tf::Transform        transform(tf::create_quaternion_from_yaw(M_PI), tf::Vector3(average, 0, 0));
	Time                 time(clock);
	tf::StampedTransform stamped_transform(transform, time, frame_.c_str(), target_frame_.c_str());
	tf_publisher->send_transform(stamped_transform);

	// write result
	if_result_->set_visibility_history(1);
	if_result_->set_translation(0, average);
	if_result_->set_frame(frame_.c_str());
	if_result_->write();
}

void
LaserFrontDistThread::finalize()
{
	blackboard->close(if_laser_);
	blackboard->close(if_result_);
}
