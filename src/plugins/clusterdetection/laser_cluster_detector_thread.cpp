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

#include <interfaces/Position3DInterface.h>
#include <interfaces/Laser360Interface.h>
#include <utils/math/coord.h>
#include <utils/math/angle.h>
#include <tf/types.h>
#include <utils/system/file.h>

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
	cfg_cluster_allowed_variance_over_time_ = config->get_float(
			CFG_PREFIX"cluster_variance_over_time");

	logger->log_debug(name(), "Configuration values:");
	logger->log_debug(name(), "laser_min_length: %f", cfg_laser_min_);
	logger->log_debug(name(), "laser_max_length: %f", cfg_laser_max_);
	logger->log_debug(name(), "laser_scanrange: %d", cfg_laser_scanrange_);
	logger->log_debug(name(), "threshold: %f", cfg_dist_threshold_);
	logger->log_debug(name(), "cluster size: %f", cfg_cluster_valid_size_);
	logger->log_debug(name(), "cluster_variance: %f",
			cfg_cluster_allowed_variance_);
	logger->log_debug(name(), "cluster_variance_over_time: %f",
			cfg_cluster_allowed_variance_over_time_);

	pos3d_if_ = blackboard->open_for_writing<Position3DInterface>(
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
	blackboard->close(pos3d_if_);
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

		//rotate angles so that scan starts at right scan range limit
		//Only for calculation, needs to be undone before publishing!!!
		int angle = (i + cfg_laser_scanrange_ / 2) % 360;
		filtered_scan_.push_back(PolarPos(angle, distance));

		//skip all the readings that are not to be considered:
		if (i == cfg_laser_scanrange_ / 2)
			i = (360 - cfg_laser_scanrange_ / 2) - 1;

	}
	filtered_scan_.sort(compare_polar_pos);
}

tf::Stamped<tf::Point> LaserClusterDetector::apply_tf(
		tf::Stamped<tf::Point> src) {

	const char* target_frame = "/base_link";
	const char* source_frame = "/base_laser";

	tf::Stamped<tf::Point> targetPoint;
	targetPoint.frame_id = target_frame;

	bool link_frame_exists = tf_listener->frame_exists(target_frame);
	bool laser_frame_exists = tf_listener->frame_exists(source_frame);

	if (!link_frame_exists || !laser_frame_exists) {
		logger->log_warn(name(), "Frame missing: %s %s   %s %s", source_frame,
				link_frame_exists ? "exists" : "missing", target_frame,
				laser_frame_exists ? "exists" : "missing");
	} else {
		src.frame_id = source_frame;
		try {
			tf_listener->transform_point(target_frame, src, targetPoint);
		} catch (tf::ExtrapolationException &e) {
			logger->log_debug(name(), "Extrapolation error");
			return src;
		} catch (tf::ConnectivityException &e) {
			logger->log_debug(name(), "Connectivity exception: %s", e.what());
			return src;
		}

		return targetPoint;

	}
	return src;
}

//for analyzing laserdata manually
void LaserClusterDetector::write_laser_to_file() {
	fawkes::File file("laserdata.csv", fawkes::File::ADD_SUFFIX);
	for (laserscan::iterator it = filtered_scan_.begin();
			it != filtered_scan_.end(); ++it) {
		fprintf(file.stream(), "%s\n ", it->to_string().c_str());

	}
}

void LaserClusterDetector::publish_nearest_light() {
	Point3d light;
	if (lights_.size() > 0) {
		PolarPos& nearest_light = lights_.front();
		float min_distance = nearest_light.distance;
		for (std::list<PolarPos>::iterator it = lights_.begin();
				it != lights_.end(); ++it) {
			if (it->distance < min_distance) {
				nearest_light = *it;
				min_distance = nearest_light.distance;
			}
		}
		PolarPos nearest_light_transformed(
				(nearest_light.angle - cfg_laser_scanrange_ / 2) % 360,
				nearest_light.distance);
		float x;
		float y;
		nearest_light_transformed.toCart(&x, &y);
		if (cfg_debug_)
			logger->log_debug(name(),
					"before tf: %s, which is rotated to %s -> x: %f, y: %f",
					nearest_light_transformed.to_string().c_str(),
					nearest_light.to_string().c_str(), x, y);

		light.setX(x);
		light.setY(y);
		light.setZ(0);

		light = apply_tf(light);

		if (cfg_debug_)
			logger->log_debug(name(), "transformed: x: %f, y: %f", light.getX(),
					light.getY());
		pos3d_if_->set_translation(0, light.getX());
		pos3d_if_->set_translation(1, light.getY());
		pos3d_if_->set_frame("/base_link");
	}
	if (lights_.size() == 0) {
		// if we have no light, we set visibility history to at least -1
		pos3d_if_->set_visibility_history(
				min(pos3d_if_->visibility_history() - 1, -1));

	} else if (pos3d_if_->visibility_history() <= 0) {
		// we see a light for the first time (or again but not the last time)
		pos3d_if_->set_visibility_history(1);
		last_light_ = light;
	} else {
		float distance = fawkes::distance(light.getX(), light.getY(),
				last_light_.getX(), last_light_.getY());
		if (cfg_debug_) {
			logger->log_debug(name(), "Distance to last cluster: %f", distance);
		}
		if (distance < cfg_cluster_allowed_variance_over_time_) {
			//distance small enough to count up
			pos3d_if_->set_visibility_history(
					(pos3d_if_->visibility_history() + 1));
			last_light_ = light;
		} else {
			//distance to large -> new light
			pos3d_if_->set_visibility_history(1);
			last_light_ = light;
		}
	}
	pos3d_if_->write();
}

void LaserClusterDetector::loop() {
	cfg_debug_ = (loopcnt++ % 50) == 0;
	read_laser();
//if (cfg_debug_)
//	write_laser_to_file();
	find_lights();
	publish_nearest_light();

}

