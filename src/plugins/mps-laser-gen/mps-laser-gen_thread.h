
/***************************************************************************
 *  mps-laser-gen_thread.h - mps-laser-gen
 *
 *  Plugin created: Thu Jun 30 21:54:46 2016

 *  Copyright  2016  Tim Niemueller
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

#ifndef __PLUGINS_MPS_LASER_GEN_THREAD_H_
#define __PLUGINS_MPS_LASER_GEN_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/LaserBoxFilterInterface.h>
#include <navgraph/aspect/navgraph.h>
#ifdef HAVE_ROS2
#	include <plugins/ros2/aspect/ros2.h>

#	include <rclcpp/rclcpp.hpp>
#	include <visualization_msgs/msg/marker_array.hpp>
#endif
#ifdef HAVE_ROS
#	include <plugins/ros/aspect/ros.h>
#	include <ros/publisher.h>
#endif

#include <Eigen/Geometry>
#include <string>

#define CFG_PREFIX "/plugins/mps_laser_gen/"

namespace fawkes {
}

class MPSLaserGenThread : public fawkes::Thread,
                          public fawkes::BlockedTimingAspect,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::NavGraphAspect,
#ifdef HAVE_ROS2
                          public fawkes::ROS2Aspect,
#endif
#ifdef HAVE_ROS
                          public fawkes::ROSAspect,
#endif
                          public fawkes::TransformAspect
{
	/// @cond INTERNAL
	class MPS
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Eigen::Vector2f center;
		Eigen::Vector2f corners[4];

		unsigned int closest_idx;
		unsigned int adjacent_1;
		unsigned int adjacent_2;

		float bearing;
	};
	/// @endcond

public:
	/** MPSLaserGenThread constructor
   */
	MPSLaserGenThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::Laser360Interface       *laser_if_;
	fawkes::LaserBoxFilterInterface *laser_box_filter_if_;
#ifdef HAVE_ROS
	ros::Publisher vispub_;
#endif
#ifdef HAVE_ROS2
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vispub_;
#endif
	std::string                mps_laser_gen_cfg_prefix;
	std::map<std::string, MPS> mpses;

	bool  cfg_enable_mps_laser_gen_;
	bool  cfg_enable_mps_box_filter_;
	float cfg_mps_length_;
	float cfg_mps_width_;

	void load_config();
};

#endif
