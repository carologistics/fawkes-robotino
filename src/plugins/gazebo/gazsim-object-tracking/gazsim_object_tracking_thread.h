/***************************************************************************
 *  gazsim_object_tracking.h - Thread simulates object tracking of
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

#ifndef __PLUGINS_GAZSIM_OBJECT_TRACKING_THREAD_H_
#define __PLUGINS_GAZSIM_OBJECT_TRACKING_THREAD_H_

#include <string.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


namespace fawkes {
	class Position3DInterface;
}

class ObjectTrackingThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
public:
	ObjectTrackingThread();

	virtual void init();

private:
	//Subscriber to receive localization data from gazebo
	gazebo::transport::SubscriberPtr localization_sub_;
	std::string gps_topic_;
	gazebo::transport::SubscriberPtr factory_sub_;
	std::string factory_topic_;

	//handler function for incoming localization data messages
	void on_localization_msg(ConstPosePtr &msg);
	void on_factory_msg(ConstFactoryPtr &msg);

	//puck tracking data
	int tracked_pucks_;
	std::string puck_names_[50];
	double puck_positions_[50][3];

	//MPS tracking data
	std::string mps_names_[14] = {"M-BS",
	                              "M-RS1",
	                              "M-RS2",
	                              "M-CS1",
	                              "M-CS2",
	                              "M-DS",
	                              "M-SS",
	                              "C-BS",
	                              "C-RS1",
	                              "C-RS2",
	                              "C-CS1",
	                              "C-CS2",
	                              "C-DS",
	                              "C-SS"};
	double mps_positions_[14][3];
};

#endif
