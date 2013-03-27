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
#include <vector>
#include <utility>

#include <string>

namespace fawkes
{
class Laser360Interface;
class PolarPosition2DInterface;
class TransformInterface;
}

class LaserClusterDetector:
		public fawkes::Thread,
		public fawkes::BlockedTimingAspect,
		public fawkes::LoggingAspect,
		public fawkes::ConfigurableAspect,
		public fawkes::BlackBoardAspect,
		public fawkes::ClockAspect
{

public:
	LaserClusterDetector();

	virtual void init();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	typedef std::map<int,float> laserscan;
	typedef std::pair<int,float> laserreading;

	struct polarPos{
		int angle;
		float distance;

		polarPos(int angle,float distance):
			angle(angle),distance(distance){}
	};

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void run()
	{
		Thread::run();
	}

private:
	void find_lights();
	void read_laser();
	void publish_cluster_position();
	void publish_cluster_as_laser();
	void add_offset(float x, float y, float *offset_x, float *offset_y);


private:
	fawkes::Laser360Interface *laser_if_;
	fawkes::Laser360Interface *laser_if_out_;
	fawkes::PolarPosition2DInterface *polar_if_;
	bool transform_available();

	std::list<polarPos> lights_;
	unsigned int num_scans_;
	laserscan filtered_scan_;


	float cfg_laser_min_;
	float cfg_laser_max_;
	unsigned int cfg_laser_scanrange_;
	unsigned int cfg_cluster_valid_size_;
	unsigned int cfg_cluster_allowed_variance_;

	float cfg_dist_threshold_;
	float cfg_valid_cluster_radius_;
	float cfg_laser_offset_;

};

#endif
