/***************************************************************************
 *	gazsim_navgraph_generator_thread.h - Thread for generating the navgraph
 *without exploration phase
 *
 *	Created: Mon Feb 15 11:27:09 2016
 *	Copyright	2016	David Schmidt
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

#ifndef __PLUGINS_GAZSIM_NAVGRAPH_GENERATOR_THREAD_H_
#define __PLUGINS_GAZSIM_NAVGRAPH_GENERATOR_THREAD_H_

#include <core/threading/thread.h>
//#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>

// gazebo headers
#include <plugins/gazebo/aspect/gazebo.h>

class GazsimNavgraphGeneratorThread : public fawkes::Thread,
                                      // public fawkes::ClockAspect,
                                      public fawkes::LoggingAspect,
                                      public fawkes::ConfigurableAspect,
                                      public fawkes::BlackBoardAspect,
                                      public fawkes::BlockedTimingAspect,
                                      public fawkes::TransformAspect,
                                      public fawkes::GazeboAspect
{
public:
	GazsimNavgraphGeneratorThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	// controlling flags
	bool                                                       task_finished_;
	bool                                                       computation_is_running_;
	fawkes::NavGraphWithMPSGeneratorInterface::ComputeMessage *compute_msg_;

	// Subscribers to receive tag positions from gazebo
	std::vector<std::string>                      tags_;
	std::vector<std::string>                      related_mps_;
	std::vector<gazebo::transport::SubscriberPtr> subscriber_tags_;

	// navgraph generator interface
	std::string                                nav_gen_if_name_;
	fawkes::NavGraphWithMPSGeneratorInterface *nav_gen_if_;

	// list of poses of the tags
	std::map<int, gazebo::msgs::Pose> tag_msgs_;

	// handler function for incoming messages about the tag positions
	void on_tag_msg(ConstPosePtr &msg);

	// extract mpsID ordered by tagID
	std::map<int, std::string> mps_id_;
	void                       get_mpsID_by_tagID();

	// send station msg with pose information to navgraph generator
	void send_station_msg(int id, gazebo::msgs::Pose pose);
};

#endif
