/***************************************************************************
 *  velocity_share_thread.cpp - Thread to gather position information and share
 *  it with other robots via robot_memory
 *
 *  Created: Sat Jan 20 17:00:04 CET 2018
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

#include "velocity_share_thread.h"

#define CFG_PREFIX "/plugins/velocity_share/"

// the robot-memory collection of use
#define COLLECTION "memory.robots"

// the timeout value for new data
#define NEW_DATA_TIMEOUT 1.0

// required to determine whether we have new velocities or not
#define EPS 0.0001f

using namespace fawkes;

/** @class VelocityShareThread "velocity_share_thread.h"
 * Thread to gather position and planned velocity of the robot and
 * to store this info in robot_memory to be shared among
 * other robots.
 * @author Nicolas Limpert
 */

/** Constructor. */
VelocityShareThread::VelocityShareThread()
  : Thread("VelocityShareThread", Thread::OPMODE_CONTINUOUS),
    ConfigurationChangeHandler(CFG_PREFIX)
{
}

void
VelocityShareThread::init()
{
  config->add_change_handler(this);
  load_config();

  vel_share_pub_ = rosnode->advertise<velocity_share_msgs::RobotVelInfoStamped>(vel_share_pub_topic_, 100);

  now_ = ros::Time::now();

  time_wait_ = new TimeWait(clock, (long int)(1000000 * (1. / update_rate_)));

  update_needed_ = false;

  logger->log_info(name(), "Plugin initialized for robot: %i", robot_number_);
}

void
VelocityShareThread::finalize()
{
  //remove this robot from robot_memory
  mongo::BSONObjBuilder q;
  q.append("object", "robot");
  q.append("robot_number", robot_number_);
  robot_memory->remove(q.obj(), COLLECTION);
  delete time_wait_;
}

void
VelocityShareThread::loop()
{
  time_wait_->mark_start();
  // update time
  now_ = ros::Time::now();

    // first, create the query for this robot
    mongo::BSONObjBuilder query;
    query.append("object", "robot");
    query.append("robot_number", robot_number_);






    mongo::BSONObjBuilder update;
    update.append("$set", obj.obj());
    robot_memory->update(query.obj(), update.obj(), COLLECTION);
    update_needed_ = false;
  }

  //query other robot poses
  mongo::BSONObjBuilder q;
  q.append("object", "robot");
  QResCursor res = robot_memory->query(q.obj(), COLLECTION);
  while(res->more())
  {
    mongo::BSONObj robot = res->next();

    // if the found object is this robot, continue.
    if (robot.getField("robot_number").Number() == robot_number_) {
      continue;
    }


    double robotdate = robot.getField("timestamp").number();

    ros::Time ros_robottime(robotdate);

    // TODO: only publish on change!
    vel_share_pub_.publish(info);
  }

  time_wait_->wait();
}

/**
 * Check for equality of two floats based on the absolute difference compared to the
 * epsilon EPS.
 * @param a first float to compare.
 * @param b second float to compare.
 * @return bool on almost equality or not
 */
inline bool
VelocityShareThread::almost_equal(float a, float b)
{
  return (fabs(a - b) < EPS);
}

void VelocityShareThread::config_value_erased(const char *path) {}
void VelocityShareThread::config_tag_changed(const char *new_tag) {}
void VelocityShareThread::config_comment_changed(const Configuration::ValueIterator *v) {}

/**
 * Load config if value changed
 */
void
VelocityShareThread::config_value_changed(const Configuration::ValueIterator *v)
{
  load_config();
}

void VelocityShareThread::load_config()
{
  std::string prefix = CFG_PREFIX;

  // get robot-name
  try {
    std::string robotname_confpath = config->get_string(prefix + "robot_name_confpath");
    robot_number_ = atoi(config->get_string(robotname_confpath).c_str() + 2);
  } catch (Exception &e) {
    logger->log_error( name() , "Can't read the robot name. Is 'robot-name' specified?" );
  }

  // get update_rate
  try {
    update_rate_ = config->get_float(prefix + "update_rate");
  } catch (Exception &e) {
    update_rate_ = 2.;
    logger->log_error(name() , "Can't read update_rate. Setting default to %f Hz", update_rate_ );
  }

  // get robot_vel topic
  try {
    vel_share_pub_topic_ = config->get_string(prefix + "robot_vel_topic");
  } catch (Exception &e) {
    vel_share_pub_topic_ = "/robot_velocities";
    logger->log_error(name() , "Can't read robot_vel_topic name. Setting default to %s",
                      vel_share_pub_topic_.c_str() );
  }

  // recreate an object for this robot in robot_memory,
  // we do not know the data structure of the previous object so simply create a new one.
  mongo::BSONObjBuilder q;
  q.append("object", "robot");
  q.append("robot_number", robot_number_);
  robot_memory->remove(q.obj(), COLLECTION);

  mongo::BSONArrayBuilder path_array;
  path_array.append(0.0);
  path_array.append(0.0);

  mongo::BSONObjBuilder b;
  b.append("object", "robot");
  b.append("robot_number", robot_number_);
  b.appendArray("path", path_array.arr());
  b.appendNumber("timestamp", now_.toSec());
  robot_memory->insert(b.obj(), COLLECTION);
}
