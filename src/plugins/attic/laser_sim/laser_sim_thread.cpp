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

#include "laser_sim_thread.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/Position3DInterface.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <utility>
#include <vector>

#define CFG_PREFIX "/plugins/LaserSim/"
#define NOISE 0.01f
#define BASE_DIST 2.0f
#define CLUSTER_LOC 340
#define CLUSTER_SIZE 2
#define CLUSTER_DIST 0.25

using namespace fawkes;
using namespace std;

/** @class MachinePositionerThread "machinepositioner_thread.h"
 * Reducing a laserscan to the relevant clusters
 * @author Daniel Ewert
 */

/** Constructor. */
LaserSim::LaserSim()
: Thread("MachinePositionerThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void
LaserSim::init()
{
	logger->log_info(name(), "Laser_Sim starts up");
	laser_if_ = blackboard->open_for_writing<Laser360Interface>("Laser sim");
}

bool
LaserSim::prepare_finalize_user()
{
	return true;
}

void
LaserSim::finalize()
{
	blackboard->close(laser_if_);
}

void
LaserSim::add_cluster_at(unsigned int angle_of_center, unsigned int size, float distance)
{
	// adding some noise
	switch (rand() % 50) {
	case 0: angle_of_center++; break;
	case 1: angle_of_center--; break;
	default: break;
	}

	for (unsigned int i = 0; i < size; ++i) {
		int angle = (i + angle_of_center) - size / 2;

		if (angle >= 360)
			angle -= 360;
		else if (angle < 0)
			angle += 360;
		laser_if_->set_distances(angle, add_noise(distance));
	}
}

void
LaserSim::loop()
{
	for (unsigned int i = 0; i < 360; ++i) {
		laser_if_->set_distances(i, add_noise(BASE_DIST));
	}
	add_cluster_at(CLUSTER_LOC, CLUSTER_SIZE, CLUSTER_DIST);
	// INTERESTING: Lasergui doesn't show the 0 value, and allows [0-360]
	// entries..
	laser_if_->write();
}

float
LaserSim::add_noise(float base)
{
	double noise = (drand48() * NOISE * 2) - NOISE;
	return (float)noise + base;
}
