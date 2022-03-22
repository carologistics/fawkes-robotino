
/***************************************************************************
 *  com_thread.cpp - Arduino com thread
 *
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
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

#include "tf_thread.h"

#include <baseapp/run.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <interfaces/ArduinoInterface.h>
#include <interfaces/BatteryInterface.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/thread.hpp>
#include <libudev.h>
#include <unistd.h>

using namespace fawkes;

/** @class ArduinoTFThread "tf_thread.h"
 * Thread to share poses of the gripper via tf
 * @author Tim Niemueller, Nicolas Limpert
 */

/** Constructor. */
ArduinoTFThread::ArduinoTFThread(std::string &cfg_name, std::string &cfg_prefix)
: Thread("ArduinoTFThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE),
  TransformAspect(TransformAspect::DEFER_PUBLISHER),
  dyn_x_pub(nullptr),
  dyn_y_pub(nullptr),
  dyn_z_pub(nullptr)
{
	cfg_prefix_ = cfg_prefix;
	cfg_name_   = cfg_name;
}

/** Destructor. */
ArduinoTFThread::~ArduinoTFThread()
{
}

void
ArduinoTFThread::init()
{
	load_config();
	//    last_official_z_position_ = 0.;
	//    desired_end_z_pose_ = 0.;
	cur_x_ = 0.0;
	cur_y_ = 0.0;
	cur_z_ = 0.0;

	//-- initialize publisher objects
	tf_add_publisher(cfg_gripper_dyn_x_frame_id_.c_str());
	dyn_x_pub = tf_publishers[cfg_gripper_dyn_x_frame_id_];

	tf_add_publisher(cfg_gripper_dyn_y_frame_id_.c_str());
	dyn_y_pub = tf_publishers[cfg_gripper_dyn_y_frame_id_];

	tf_add_publisher(cfg_gripper_dyn_z_frame_id_.c_str());
	dyn_z_pub = tf_publishers[cfg_gripper_dyn_z_frame_id_];
	update();
}

void
ArduinoTFThread::finalize()
{
}

void
ArduinoTFThread::update()
{
	boost::mutex::scoped_lock lock(data_mutex_);
	fawkes::Time              now(clock);

	//    float d_mm = current_end_z_pose_ - desired_end_z_pose_;
	//    double d_s = now - end_time_point_;

	tf::Quaternion q(0.0, 0.0, 0.0);

	tf::Vector3 v_x(cur_x_, 0.0, 0.0);

	tf::Vector3 v_y(0.0, (cur_y_ - cfg_y_max_ / 2.), 0.0);

	tf::Vector3 v_z(0.0, 0.0, cur_z_);

	tf::Transform tf_pose_gripper_x(q, v_x);
	tf::Transform tf_pose_gripper_y(q, v_y);
	tf::Transform tf_pose_gripper_z(q, v_z);

	tf::StampedTransform stamped_transform_x(tf_pose_gripper_x,
	                                         now.stamp(),
	                                         cfg_gripper_origin_x_frame_id_,
	                                         cfg_gripper_dyn_x_frame_id_);
	tf::StampedTransform stamped_transform_y(tf_pose_gripper_y,
	                                         now.stamp(),
	                                         cfg_gripper_origin_y_frame_id_,
	                                         cfg_gripper_dyn_y_frame_id_);
	tf::StampedTransform stamped_transform_z(tf_pose_gripper_z,
	                                         now.stamp(),
	                                         cfg_gripper_origin_z_frame_id_,
	                                         cfg_gripper_dyn_z_frame_id_);
	dyn_x_pub->send_transform(stamped_transform_x);
	dyn_y_pub->send_transform(stamped_transform_y);
	dyn_z_pub->send_transform(stamped_transform_z);
}

//// interpolate the current z position based on an assumption of where the
/// gripper should be given a timeframe and a length
// void
// ArduinoTFThread::set_interpolation_interval(fawkes::Time &end_time_point,
// float desired_end_z_pose)
//{
//    boost::mutex::scoped_lock lock(data_mutex_);
//    end_time_point_ = end_time_point;
//    desired_end_z_pose_ = desired_end_z_pose;
//
////    // Stepper speed: 500 steps / second
////    // 2.5 revolutions per second
////    // 5mm per second
//
//
//}

void
ArduinoTFThread::set_position(float new_x_pos, float new_y_pos, float new_z_pos)
{
	boost::mutex::scoped_lock lock(data_mutex_);
	cur_x_ = new_x_pos;
	cur_y_ = new_y_pos;
	cur_z_ = new_z_pos;
	update();
}

void
ArduinoTFThread::load_config()
{
	cfg_gripper_frame_id_          = config->get_string(cfg_prefix_ + "/gripper_frame_id");
	cfg_gripper_origin_x_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_origin_x_frame_id");
	cfg_gripper_origin_y_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_origin_y_frame_id");
	cfg_gripper_origin_z_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_origin_z_frame_id");

	cfg_gripper_dyn_x_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_x_frame_id");
	cfg_gripper_dyn_y_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_y_frame_id");
	cfg_gripper_dyn_z_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_z_frame_id");

	cfg_x_max_ = config->get_float(cfg_prefix_ + "/x_max");
	cfg_y_max_ = config->get_float(cfg_prefix_ + "/y_max");
	cfg_z_max_ = config->get_float(cfg_prefix_ + "/z_max");
}
