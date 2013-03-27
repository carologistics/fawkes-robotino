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

#include "laser_cluster_detector_thread.h"

#include <interfaces/PolarPosition2DInterface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/TransformInterface.h>
#include <tf/types.h>

#include <vector>
#include <utility>
#include <algorithm>

#include <cmath>
#include <cfloat>
#include <utils/math/coord.h>
#include <utils/math/angle.h>

#define CFG_PREFIX "/plugins/laserclusterdetector/"

#define LASER_OUTPUT
using namespace fawkes;
using namespace std;

/** @class MachinePositionerThread "machinepositioner_thread.h"
 * Reducing a laserscan to the relevant clusters
 * @author Daniel Ewert
 */

float fit_angle_to_360(float angle) {
	if (angle < 0)
		angle += 360;
	if (angle > 359)
		angle -= 360;
	return angle;
}

/** Constructor. */
LaserClusterDetector::LaserClusterDetector() :
		Thread("MachinePositionerThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void LaserClusterDetector::init() {

	logger->log_info(name(), "Laser_Cluster_Detector starts up");
	laser_if_ = blackboard->open_for_reading<Laser360Interface>(
			config->get_string(CFG_PREFIX"laser_interface").c_str());
	laser_if_out_ = blackboard->open_for_writing<Laser360Interface>(
			"Laser cluster");
	num_scans_ = laser_if_->maxlenof_distances();

	cfg_laser_min_ = config->get_float(CFG_PREFIX"laser_min_length");
	cfg_laser_max_ = config->get_float(CFG_PREFIX"laser_max_length");
	cfg_laser_scanrange_ = config->get_uint(CFG_PREFIX"laser_scanrange");
	cfg_dist_threshold_ = config->get_float(CFG_PREFIX"dist_threshold");
	cfg_laser_offset_ = config->get_float(CFG_PREFIX"laser_offset");

	cfg_valid_cluster_radius_ = config->get_float(
			CFG_PREFIX"valid_cluster_radius");

	polar_if_ = blackboard->open_for_writing<PolarPosition2DInterface>(
			"Closest_Machine");

}

bool LaserClusterDetector::prepare_finalize_user() {
	return true;
}

bool compareReadings(const pair<unsigned int, float>& f,
		const pair<unsigned int, float>& s) {
	return f.first < s.first;
}

void LaserClusterDetector::finalize() {
	blackboard->close(laser_if_);
	blackboard->close(polar_if_);
	blackboard->close(laser_if_out_);
}

void LaserClusterDetector::find_lights() {

	lights_.clear();

	int last_angle =0;
	float last_distance =0;
	int angle_last_peak =0;
	float distance_last_peak =0;

	for (laserscan::iterator it = filtered_scan_.begin();
			it != filtered_scan_.end(); ++it) {

		if (it == filtered_scan_.begin()) {
			angle_last_peak = it->first;
			distance_last_peak = it->second;
			last_angle = it->first;
			last_distance = it->second;
			continue;
		}

		int current_angle = it->first;
		float current_distance = it->second;

		//if more than one reading was discarded between current and last scan, ignore
		if (current_angle - last_angle > 1) {
			last_angle = current_angle;
			last_distance = current_distance;
			continue;
		}

		if (abs(last_distance - current_distance) > cfg_dist_threshold_) {
			//we have a sufficient peak, can either be the begin or the end

			//check if angle between current_angle and angle_last_peak
			//given the measured distances could be an signal light
			//with c= sqrt(a**2 + b**2 - 2 ab cos(gamma)) las of cosines
			//c then must be a feasible diameter of the light at the measured height
			int angle = current_angle - angle_last_peak;
			float diam_light = sqrt(
					distance_last_peak * distance_last_peak
							+ current_distance * current_distance
							- 2 * distance_last_peak * current_distance
									* cos(angle * M_PI / 180.0));
			if (abs(diam_light - cfg_cluster_valid_size_)
					<= cfg_cluster_allowed_variance_) {

				//We've found a light! Store it's position by it's center
				int light_angle = current_angle - (angle / 2.0);
				float light_distance = (distance_last_peak + current_distance)
						/ 2.0;
				lights_.push_back(polarPos(light_angle, light_distance));

			}
			//Peak can be the beginning of a new light
			distance_last_peak = current_distance;
			angle_last_peak = current_angle;
		}
		last_angle = current_angle;
		last_distance = current_distance;
	}

}

void LaserClusterDetector::read_laser() {

	//reading at [0] is in driving direction
	//laser is reading counterclockwise
	laser_if_->read();
	filtered_scan_.clear();
	for (size_t i = 0; i < num_scans_; i++) {
		//filter and transform to correct orientation
		float distance = laser_if_->distances(i);

		if (isfinite(distance) // valid number?
		&& distance > cfg_laser_min_ //big enough?
		&& distance < cfg_laser_max_) { //small enough?

			int angle = 360 - i; //invert to clockwise laser scan

			//rotate angles so that scan starts at left scan range limit
			//Only for calculation, needs to be undone before publishing!!!
			angle = (angle + cfg_laser_scanrange_ / 2) % 360;

			filtered_scan_.insert(laserreading(angle, distance));
		}
	}
}

void LaserClusterDetector::loop() {
	read_laser();
	find_lights();
	if (lights_.size()>0){
		//publish found lights
		//TODO only publish closest light. Or more?
	}
}

