/***************************************************************************
 *  laser_cluster_detector_thread.h - reads laserdata and finds adjacent clusters
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

#ifndef __PLUGINS_LASER_CLUSTER_DETECTOR_THREAD_H_
#define __PLUGINS_LASER_CLUSTER_DETECTOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/tf.h>
#include <tf/types.h>
#include <list>
#include <cmath>
#include <sstream>

#include <string>

namespace fawkes {
class Laser360Interface;
class Position3DInterface;
}

class LaserClusterDetector: public fawkes::Thread,
		public fawkes::BlockedTimingAspect,
		public fawkes::LoggingAspect,
		public fawkes::ConfigurableAspect,
		public fawkes::BlackBoardAspect,
		public fawkes::ClockAspect,
		public fawkes::TransformAspect {

public:
	LaserClusterDetector();

	virtual void init();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	struct PolarPos {
		int angle;
		float distance;

		PolarPos(int angle, float distance) :
				angle(angle), distance(distance) {
		}

		PolarPos() {
			angle = 0;
			distance = 0;
		}

		void toCart(float* x, float* y) {
			*x = distance * cos(angle * M_PI / 180.0);
			*y = distance * sin(angle * M_PI / 180.0);
		}

		void fromCart(float x, float y) {
			angle = atan2f(y, x) * 180 / M_PI;
			distance = sqrtf(x * x + y * y);
		}

		std::string to_string() {
			std::stringstream str;
			str << "(angle: " << angle << ", distance: " << distance << ")";
			return str.str();
		}
	};

	typedef std::list<PolarPos> laserscan;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void run() {
		Thread::run();
	}

private:
	void find_lights();
	void read_laser();
	void write_laser_to_file();
	fawkes::tf::Stamped<fawkes::tf::Point> apply_tf(fawkes::tf::Stamped<fawkes::tf::Point> src);

private:
	fawkes::Laser360Interface *laser_if_;
	fawkes::Position3DInterface *pos3d_if_;

	std::list<PolarPos> lights_;
	unsigned int num_scans_;
	laserscan filtered_scan_;

	float cfg_laser_min_;
	float cfg_laser_max_;
	unsigned int cfg_laser_scanrange_;
	float cfg_cluster_valid_size_;
	float cfg_cluster_allowed_variance_;
	float cfg_dist_threshold_;

	int loopcnt;
	bool cfg_debug_;
};

#endif
