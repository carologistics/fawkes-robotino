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
#include <utils/math/coord.h>
#include <utils/math/angle.h>

#include <cmath>
#include <iterator>

#define CFG_PREFIX "/plugins/laserclusterdetector/"

#define LASER_OUTPUT
using namespace fawkes;
using namespace std;

/** @class LaserClusterDetector "laser_cluster_detector_thread.h"
 * Find clusters in a laser scan
 * @author Daniel Ewert
 */

bool compare_polar_pos(const LaserClusterDetector::PolarPos& a,
		const LaserClusterDetector::PolarPos& b) {
	return a.angle < b.angle;
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
	num_scans_ = laser_if_->maxlenof_distances();

	cfg_laser_min_ = config->get_float(CFG_PREFIX"laser_min_length");
	cfg_laser_max_ = config->get_float(CFG_PREFIX"laser_max_length");
	cfg_laser_scanrange_ = config->get_uint(CFG_PREFIX"laser_scanrange");
	cfg_dist_threshold_ = config->get_float(CFG_PREFIX"dist_threshold");
	cfg_debug_ = config->get_bool(CFG_PREFIX"show_debug_messages");

	cfg_cluster_valid_size_ = config->get_float(CFG_PREFIX"cluster_size");

	cfg_cluster_allowed_variance_ = config->get_float(
			CFG_PREFIX"cluster_variance");

	logger->log_debug(name(), "Configuration values:");
	logger->log_debug(name(), "laser_min_length: %f", cfg_laser_min_);
	logger->log_debug(name(), "laser_max_length: %f", cfg_laser_max_);
	logger->log_debug(name(), "laser_scanrange: %d", cfg_laser_scanrange_);
	logger->log_debug(name(), "threshold: %f", cfg_dist_threshold_);
	logger->log_debug(name(), "cluster size: %f", cfg_cluster_valid_size_);
	logger->log_debug(name(), "cluster_variance: %f",
			cfg_cluster_allowed_variance_);

	polar_if_ = blackboard->open_for_writing<PolarPosition2DInterface>(
			"Closest_Machine");

	loopcnt = 0;
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
}

void LaserClusterDetector::find_lights() {

	lights_.clear();

	PolarPos last_peak = filtered_scan_.front();
	if (filtered_scan_.size() < 5) {
		logger->log_error(name(), "not enough laser scan points!!");
		return;
	}

	//Incredibly ugly, but i need a list, and i need to know predec and succ..
	laserscan::iterator beforebefore = filtered_scan_.begin();
	laserscan::iterator before = filtered_scan_.begin();
	before++;

	laserscan::iterator current = filtered_scan_.begin();
	std::advance(current, 2);

	laserscan::iterator following = filtered_scan_.begin();
	std::advance(following, 3);
	laserscan::iterator followingfollowing = filtered_scan_.begin();
	std::advance(followingfollowing, 4);

	while (followingfollowing != filtered_scan_.end()) {
		//we can either have a peak (the begin or end of a cluster) iff
		// - the difference between two adjacent values is bigger than threshold
		// - we read two invalid readings before or after the current reading
		if (current->distance != -1
				&& (abs(before->distance - current->distance)
						> cfg_dist_threshold_
						|| (before->distance == -1
								&& beforebefore->distance == -1)
						|| (following->distance == -1
								&& followingfollowing->distance == -1)

				)) {
			//we have a sufficient peak, can either be the begin or the end

			if (cfg_debug_)
				logger->log_debug(name(), "peak at %d", current->angle);

			//check if we have a valid last_peak.
			//if not, go on
			if (last_peak.distance != -1.0) {

				//check if angle between current_angle and angle_last_peak
				//given the measured distances could be an signal light
				//with c= sqrt(a**2 + b**2 - 2 ab cos(gamma)) las of cosines
				//c then must be a feasible diameter of the light at the measured height
				int angle = current->angle - last_peak.angle;

				float diam_light = sqrt(
						last_peak.distance * last_peak.distance
								+ current->distance * current->distance
								- 2 * last_peak.distance * current->distance
										* cos(angle * M_PI / 180.0));

				if (cfg_debug_)
					logger->log_debug(name(),
							"lastpeak angle: %d, current angle: %d,-> angle: %d, Distance: %f, Size of cluster: %f",
							last_peak.angle, current->angle, angle,
							last_peak.distance, diam_light);

				if (abs(diam_light - cfg_cluster_valid_size_)
						<= cfg_cluster_allowed_variance_) {
					if (cfg_debug_)
						logger->log_debug(name(), "Valid light!");
					//We've found a light! Store it's position by it's center
					int light_angle = current->angle - (angle / 2.0);

					float light_distance = (last_peak.distance
							+ current->distance) / 2.0;
					lights_.push_back(PolarPos(light_angle, light_distance));
					return; //for the moment only interested in one light

				}
			}
			//Peak can be the beginning of a new light
			last_peak = *current;
		}

		//yeah, it's ugly.
		++beforebefore;
		++before;
		++current;
		++following;
		++followingfollowing;
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

		if (!(isfinite(distance) // valid number?
		&& distance > cfg_laser_min_ //big enough?
		&& distance < cfg_laser_max_)) { //small enough?
			distance = -1;
		}
		int angle = 360 - i; //invert to clockwise laser scan

		//rotate angles so that scan starts at left scan range limit
		//Only for calculation, needs to be undone before publishing!!!
		angle = (angle + cfg_laser_scanrange_ / 2) % 360;
		filtered_scan_.push_back(PolarPos(angle, distance));

		//skip all the readings that are not to be considered:
		if (i == cfg_laser_scanrange_ / 2)
			i = (360 - cfg_laser_scanrange_ / 2) - 1;

	}
	filtered_scan_.sort(compare_polar_pos);
}

LaserClusterDetector::PolarPos LaserClusterDetector::apply_tf(
		LaserClusterDetector::PolarPos src) {

	const char* target_frame = "/base_link";
	const char* source_frame = "/base_laser";

	bool link_frame_exists = tf_listener->frame_exists(target_frame);
	bool laser_frame_exists = tf_listener->frame_exists(source_frame);

	if (!link_frame_exists || !laser_frame_exists) {
		logger->log_warn(name(), "Frame missing: %s %s   %s %s", source_frame,
				link_frame_exists ? "exists" : "missing", target_frame,
				laser_frame_exists ? "exists" : "missing");
	} else {
		tf::StampedTransform transform;
		try {
			tf_listener->lookup_transform(target_frame, source_frame,
					transform);
		} catch (tf::ExtrapolationException &e) {
			logger->log_debug(name(), "Extrapolation error");
			return src;
		} catch (tf::ConnectivityException &e) {
			logger->log_debug(name(), "Connectivity exception: %s", e.what());
			return src;
		}

		float x = abs(transform.getOrigin().getX());
		float src_x, src_y;
		src.toCart(&src_x, &src_y);
		src_x += x;
		PolarPos target;
		target.fromCart(src_x, src_y);
		return target;

	}
	return src;
}

void LaserClusterDetector::loop() {
	cfg_debug_ = (loopcnt++ % 20) == 0;
	if (cfg_debug_)
		logger->log_debug(name(),
				"#################################################");
	read_laser();
	find_lights();
	if (lights_.size() > 0) {
		if (cfg_debug_)
			logger->log_debug(name(), "before tf: %s",
					lights_.front().to_string().c_str());

		PolarPos nearest_light = lights_.front();
		nearest_light.angle = (nearest_light.angle - cfg_laser_scanrange_ / 2
		) % 360;
		nearest_light = apply_tf(nearest_light);
		if (cfg_debug_)
			logger->log_debug(name(), "after tf: %s",
					nearest_light.to_string().c_str());
		polar_if_->set_angle(nearest_light.angle);
		polar_if_->set_distance(nearest_light.distance);
		polar_if_->set_frame("/base_link");
		polar_if_->write();
	}
}

