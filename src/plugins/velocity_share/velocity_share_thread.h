/***************************************************************************
 *  velocity_share_thread.h - Thread to gather position and velocity
 *  information and share it with other robots via robot_memory
 *
 *  Created: Sat Jan 20 16:55:11 2018
 *  Copyright  2018  Nicolas Limpert
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

#ifndef __PLUGINS_VELOCITY_SHARE_THREAD_H_
#define __PLUGINS_VELOCITY_SHARE_THREAD_H_

#include <aspect/logging.h>
#include <aspect/tf.h>
#include <config/change_handler.h>
#include <core/threading/thread.h>
#include <geometry_msgs/PoseStamped.h>
#include <interfaces/MotorInterface.h>
#include <interfaces/Position3DInterface.h>
#include <nav_msgs/Path.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>
#include <plugins/ros/aspect/ros.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf/types.h>
#include <utils/time/wait.h>
#include <velocity_share_msgs/RobotVelInfo.h>

class VelocityShareThread : public fawkes::Thread,
                            public fawkes::ClockAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::ConfigurationChangeHandler,
                            public fawkes::BlackBoardAspect,
                            public fawkes::ROSAspect,
                            public fawkes::TransformAspect,
                            public fawkes::RobotMemoryAspect
{
public:
	VelocityShareThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	// Implemented abstracts inherited from ConfigurationChangeHandler
	virtual void config_tag_changed(const char *new_tag);
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
	virtual void config_value_erased(const char *path);

	fawkes::TimeWait *time_wait_;
	float             update_rate_;
	bool              update_needed_;

	unsigned int cfg_max_cell_lookahead_count_;
	unsigned int cfg_number_of_segments_;

	// used to determine the priority between robots
	int robot_number_;

	void load_config();

	ros::Subscriber path_sub_;
	nav_msgs::Path  path_;
	ros::Time       now_;

	std::string    vel_share_pub_topic_;
	ros::Publisher vel_share_pub_;

	void pathCallback(const ros::MessageEvent<const nav_msgs::Path> &path);
};

#endif
