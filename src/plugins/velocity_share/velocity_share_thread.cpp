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

  pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");
  motor_if_ = blackboard->open_for_reading<MotorInterface>("Robotino");

  now_ = ros::Time::now();
  // try to find robotname_ (this robot) in COLLECTION.
  mongo::BSONObjBuilder q;
  q.append("object", "robot");
  q.append("name", robotname_);
  QResCursor res = robot_memory->query(q.obj(), COLLECTION);

  if (!res->more()) {
    // If not available, add this robot
    logger->log_warn(name(), "%s not found in collection %s, creating a new object", robotname_.c_str(), COLLECTION);

    mongo::BSONObjBuilder b;
    b.append("object", "robot");
    b.append("name", robotname_);
    b.append("pose", mongo::fromjson("{x:0.0,y:0.0,ori:0.0}"));
    b.append("vel", mongo::fromjson("{x:0.0,y:0.0,z:0.0}"));
    b.appendNumber("timestamp", now_.toSec());

    robot_memory->insert(b.obj(), COLLECTION);
  }

  time_wait_ = new TimeWait(clock, (long int)(1000000 * (1. / update_rate_)));
  //  last_vel_ = new Time(clock);

  cur_vx_    = 0.0;
  cur_vy_    = 0.0;
  cur_vori_  = 0.0;
  last_vx_   = 0.0;
  last_vy_   = 0.0;
  last_vori_ = 0.0;
  update_needed_ = false;

  logger->log_info(name(), "Plugin initialized for robot: %s", robotname_.c_str());
}

void
VelocityShareThread::finalize()
{
  //remove this robot from robot_memory
  mongo::BSONObjBuilder q;
  q.append("object", "robot");
  q.append("name", robotname_);
  robot_memory->remove(q.obj(), COLLECTION);
  delete time_wait_;
  blackboard->close(pose_if_);
  blackboard->close(motor_if_);
}

void
VelocityShareThread::loop()
{
  time_wait_->mark_start();
  // update time
  now_ = ros::Time::now();

  if (motor_if_->has_writer()) {
    motor_if_->read();
    last_vx_   = cur_vx_;
    last_vy_   = cur_vy_;
    last_vori_ = cur_vori_;
    cur_vx_    = motor_if_->vx();
    cur_vy_    = motor_if_->vy();
    cur_vori_  = motor_if_->omega();

    if (!almost_equal(last_vx_, cur_vx_) ||
        !almost_equal(last_vy_, cur_vy_) ||
        !almost_equal(last_vori_, cur_vori_))
    {
      // at least one velocity changed, update!
      update_needed_ = true;
    }
  } else {
    logger->log_warn(name(), "Motor interface does not have a writer.");
  }

  if (update_needed_ && pose_if_->has_writer()) {
    pose_if_->read();
    motor_if_->read();

    double *r = pose_if_->rotation();
    tf::Quaternion pose_q(r[0], r[1], r[2], r[3]);

    // update current robot pose
    // first, create the query for this robot
    mongo::BSONObjBuilder query;
    query.append("object", "robot");
    query.append("name", robotname_);

    const char* target_frame = "map";
    const char* source_frame = "base_link";

    // next, transform the velocity vector to the map frame
    Time stamp(0,0);
    tf::Stamped<tf::Point> source_vel_point, target_vel_point;
    source_vel_point.stamp = stamp;
    source_vel_point.frame_id = source_frame;
    source_vel_point.setX(cur_vx_ * velocity_scale_);
    source_vel_point.setY(cur_vy_ * velocity_scale_);
    source_vel_point.setZ(0.0);
    target_vel_point.frame_id = target_frame;

    try {
      tf_listener->transform_point(target_frame, source_vel_point, target_vel_point);
    } catch (tf::ExtrapolationException &e) {
      logger->log_debug(name(), "Extrapolation error");
      return;
    }

    // then, create the data to be updated
    mongo::BSONObjBuilder pose;
    pose.append("x", pose_if_->translation(0));
    pose.append("y", pose_if_->translation(1));
    pose.append("ori", tf::get_yaw(pose_q));

    // velocities in x, y are linear. z is considered the angular velocity around z
    mongo::BSONObjBuilder vel;
    vel.append("x", target_vel_point.getX());
    vel.append("y", target_vel_point.getY());
    vel.append("z", cur_vori_);

    // add "pose" and "vel" to obj
    mongo::BSONObjBuilder obj;
    obj.append("pose", pose.obj());
    obj.append("vel", vel.obj());

    // do not make use of mongo::Date_t for easier conversion from and to ROS::Time
    obj.appendNumber("timestamp", now_.toSec() );

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
    if (robotname_.compare(robot.getField("name").String()) == 0) {
      continue;
    }

    mongo::BSONObj robot_pose = robot.getField("pose").Obj();
    mongo::BSONObj robot_vel = robot.getField("vel").Obj();
    std::string this_robot_name = robot.getField("name").String();

    // create quaternion from yaw
    float yaw = robot_pose.getField("ori").number();
    tf::Quaternion robot_ori = tf::create_quaternion_from_yaw(yaw);

    // velocity vector relative to map frame
    float vel_x = robot_vel.getField("x").number();
    float vel_y = robot_vel.getField("y").number();

    // fill data
    velocity_share_msgs::RobotVelInfoStamped info;

    double robotdate = robot.getField("timestamp").number();

    ros::Time ros_robottime(robotdate);
    info.header.stamp = ros_robottime;
    info.header.frame_id = "map";
    info.robotvelinfo.robot_name = this_robot_name;

    info.robotvelinfo.pose.position.x = robot_pose.getField("x").number();
    info.robotvelinfo.pose.position.y = robot_pose.getField("y").number();
    info.robotvelinfo.pose.position.z = 0.0;
    info.robotvelinfo.pose.orientation.x = robot_ori[0];
    info.robotvelinfo.pose.orientation.y = robot_ori[1];
    info.robotvelinfo.pose.orientation.z = robot_ori[2];
    info.robotvelinfo.pose.orientation.w = robot_ori[3];

    info.robotvelinfo.vel_endpoint.x = vel_x;
    info.robotvelinfo.vel_endpoint.y = vel_y;
    info.robotvelinfo.vel_endpoint.z = 0.0;

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
    robotname_ = config->get_string(robotname_confpath);
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

  // get velocity_scale
  try {
    velocity_scale_ = config->get_float(prefix + "velocity_scale");
  } catch (Exception &e) {
    velocity_scale_ = 1.;
    logger->log_error(name() , "Can't read velocity_scale. Setting default to %f", velocity_scale_);
  }

  // get robot_vel topic
  try {
    vel_share_pub_topic_ = config->get_string(prefix + "robot_vel_topic");
  } catch (Exception &e) {
    vel_share_pub_topic_ = "/robot_velocities";
    logger->log_error(name() , "Can't read robot_vel_topic name. Setting default to %s",
                      vel_share_pub_topic_.c_str() );
  }
}
