/***************************************************************************
 *  gazsim_object_tracking.cpp - Thread simulates object tracking of
 *      workpieces, conveyor belts, and slides while providing curresponding
 *      target frames used for visual servoing in Gazebo
 *
 *  Created: Sun May 16 12:03:10 2021
 *  Copyright  2021  Matteo Tschesche
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

#include "gazsim_object_tracking_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <interfaces/Position3DInterface.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class ObjectTrackingThread "gazsim_object_tracking_thread.h"
 * Thread Simulates the Object Tracker in Gazebo
 * @author Matteo Tschesche
 */

/** Constructor. */
ObjectTrackingThread::ObjectTrackingThread()
: Thread("ObjectTrackingThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void ObjectTrackingThread::init()
{
	logger->log_info(name(),
	  "Initializing Simulation of the Workpiece (Puck) Positions");

	//read cofig values
	gps_topic_ = config->get_string("/gazsim/topics/gps");

	//subscribing to gazebo publisher
	localization_sub_ = gazebo_world_node->Subscribe(gps_topic_,
	                    &ObjectTrackingThread::on_localization_msg, this);

	//init tracking
	tracked_pucks_ = 0;
}

//updating Puck positions:
void ObjectTrackingThread::on_localization_msg(ConstPosePtr &msg)
{
	//if object name starts with puck:
	if(!std::string(msg->name()).rfind("puck", 0))
	{
		//is puck already tracked?
		for(int i = 0; i < tracked_pucks_; i++)
		{
			//if yes, update position
			if(!std::string(msg->name()).compare(puck_names_[i]))
			{
				puck_positions_[i][0] = msg->position().x();
				puck_positions_[i][1] = msg->position().y();
				puck_positions_[i][2] = msg->position().z();
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 "updated position");
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 std::to_string(positions_[i][0]).c_str());
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 std::to_string(positions_[i][1]).c_str());
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 std::to_string(positions_[i][2]).c_str());
				return;
			}
			//else start tracking...
		}

		if(tracked_pucks_ < 50)
		{
			puck_names_[tracked_pucks_] = std::string(msg->name());
			puck_positions_[tracked_pucks_][0] = msg->position().x();
			puck_positions_[tracked_pucks_][1] = msg->position().y();
			puck_positions_[tracked_pucks_][2] = msg->position().z();
			tracked_pucks_++;
			logger->log_info(std::string(msg->name()).c_str(),
			                 "is now tracked.");
		}
		else
		{
			logger->log_error(std::string(msg->name()).c_str(),
			                  "cannot be tracked! Array full!");
		}
	}
}
