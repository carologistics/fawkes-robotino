/***************************************************************************
 *  gazsim_tag_vision_plugin.h - Plugin provides ground-truth tag vision
 *
 *  Created: Thu Nov 05 20:20:10 2015
 *  Copyright  2015 Frederik Zwilling
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

#ifndef __PLUGINS_GAZSIM_TAG_VISION_THREAD_H_
#define __PLUGINS_GAZSIM_TAG_VISION_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/TagVisionInterface.h>
#include <plugins/gazebo/aspect/gazebo.h>

#include <map>
#include <vector>

// from Gazebo
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

namespace fawkes {
class Position3DInterface;
}

class TagVisionSimThread : public fawkes::Thread,
                           public fawkes::ClockAspect,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::BlockedTimingAspect,
                           public fawkes::GazeboAspect,
                           public fawkes::TransformAspect
{
public:
	TagVisionSimThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	// Subscriber to receive tag positions from gazebo
	gazebo::transport::SubscriberPtr tag_vision_sub_;

	// interfaces to publish the positions
	std::map<int, fawkes::Position3DInterface *> tag_pos_ifs_;
	// switch interface for enableing/disableing
	fawkes::TagVisionInterface *info_if_;

	// handler function for incoming messages about the machine light signals
	void on_tag_vision_msg(ConstPosesStampedPtr &msg);

	// copy of last msg to write the interface in the next loop
	gazebo::msgs::PosesStamped last_msg_;
	bool                       new_data_;

	// config values
	std::string gazebo_topic_;
	std::string tag_if_name_prefix_;
	std::string info_if_name_;
	std::string sim_frame_name_;
	std::string frame_name_;
	int         number_interfaces_;
	int         visibility_history_increase_per_update_;

	// assignment tag-id to interface-index
	std::map<int, int> map_id_if_;
};

#endif
