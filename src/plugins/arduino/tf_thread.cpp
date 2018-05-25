
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
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <interfaces/BatteryInterface.h>
#include <interfaces/ArduinoInterface.h>

#include <unistd.h>

#include <libudev.h>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/thread.hpp>

using namespace fawkes;

/** @class ArduinoTFThread "tf_thread.h"
 * Thread to share poses of the gripper via tf
 * @author Tim Niemueller, Nicolas Limpert
 */

/** Constructor. */
ArduinoTFThread::ArduinoTFThread(std::string &cfg_name,
        std::string &cfg_prefix)
: Thread("ArduinoTFThread", Thread::OPMODE_WAITFORWAKEUP),
        BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE),
        TransformAspect(TransformAspect::ONLY_PUBLISHER, "gripper")
{
    cfg_prefix_ = cfg_prefix;
    cfg_name_ = cfg_name;
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
}

void
ArduinoTFThread::finalize()
{
}

void
ArduinoTFThread::loop()
{
    boost::mutex::scoped_lock lock(data_mutex_);
//    fawkes::Time now(clock);

//    float d_mm = current_end_z_pose_ - desired_end_z_pose_;
//    double d_s = now - end_time_point_;

    tf::Quaternion q(0.0, 0.0, 0.0);
    tf::Vector3 v(cur_x_,
                  (cur_y_ - cfg_y_max_ / 2.),
                  cur_z_);

    tf::Transform tf_pose_gripper(q, v);

    tf::StampedTransform stamped_transform(tf_pose_gripper, now.stamp(), cfg_gripper_frame_id_, cfg_gripper_dyn_frame_id_);
    tf_publisher->send_transform(stamped_transform);
}

//// interpolate the current z position based on an assumption of where the gripper should be given a timeframe and a length
//void
//ArduinoTFThread::set_interpolation_interval(fawkes::Time &end_time_point, float desired_end_z_pose)
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
}

void
ArduinoTFThread::load_config()
{
    cfg_gripper_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_frame_id");
    cfg_gripper_dyn_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_frame_id");
    cfg_x_max_ = config->get_float(cfg_prefix_ + "/x_max");
    cfg_y_max_ = config->get_float(cfg_prefix_ + "/y_max");
    cfg_z_max_ = config->get_float(cfg_prefix_ + "/z_max");
}
