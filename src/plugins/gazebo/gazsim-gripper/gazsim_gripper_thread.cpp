
/***************************************************************************
 *  gazsim_gripper_thread.h - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:54:42 2015
 *  Copyright  2015 Frederik Zwilling
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

#include "gazsim_gripper_thread.h"

#include <boost/lexical_cast.hpp>
#include <config/change_handler.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/AX12GripperInterface.h>
#include <interfaces/ArduinoInterface.h>
#include <interfaces/JointInterface.h>
#include <interfaces/LedInterface.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <utils/math/angle.h>

#include <cmath>
#include <cstdarg>
#include <unistd.h>

using namespace fawkes;
using namespace gazebo;

/** @class GazsimGripperThread "gazsim_gripper_thread.h"
 * Thread simulates the Gripper in Gazebo
 *
 * @author Frederik Zwilling
 */

/** Constructor. */

GazsimGripperThread::GazsimGripperThread()
    : Thread("GazsimGripperThread", Thread::OPMODE_WAITFORWAKEUP),
      BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
      TransformAspect(TransformAspect::DEFER_PUBLISHER)
{
  set_name("GazsimGripperThread()");
}

void GazsimGripperThread::init() {
  logger->log_debug(name(),
                    "Initializing Simulation of the Light Front Plugin");

  gripper_if_name_ = config->get_string("/gazsim/gripper/if-name");
  arduino_if_name_ = config->get_string("/gazsim/gripper/arduino-if-name");
  robotino_sensor_if_name_ = config->get_string("/gazsim/gripper/robotino-sensor-if-name");
  cfg_prefix_ = config->get_string("/gazsim/gripper/cfg-prefix");

  set_gripper_pub_ = gazebonode->Advertise<msgs::Int>(
      config->get_string("/gazsim/topics/gripper"));
  set_conveyor_pub_ = gazebonode->Advertise<msgs::Int>(
      config->get_string("/gazsim/topics/conveyor"));
  gripper_has_puck_sub_ = gazebonode->Subscribe(
      config->get_string("/gazsim/topics/gripper-has-puck"),
      &GazsimGripperThread::on_has_puck_msg, this);

  // setup gripper if with default values
  gripper_if_ = blackboard->open_for_writing<AX12GripperInterface>(
      gripper_if_name_.c_str());
  gripper_if_->set_calibrated(true);
  gripper_if_->set_min_left(config->get_float(cfg_prefix_ + "left_min"));
  gripper_if_->set_max_left(config->get_float(cfg_prefix_ + "left_max"));
  gripper_if_->set_min_right(config->get_float(cfg_prefix_ + "right_min"));
  gripper_if_->set_max_right(config->get_float(cfg_prefix_ + "right_max"));
  gripper_if_->set_left_margin(config->get_float(cfg_prefix_ + "left_margin"));
  gripper_if_->set_right_margin(
      config->get_float(cfg_prefix_ + "right_margin"));
  gripper_if_->set_max_left_velocity(0);
  gripper_if_->set_max_right_velocity(0);
  gripper_if_->set_left_velocity(0);
  gripper_if_->set_right_velocity(0);
  gripper_if_->set_final(true);
  gripper_if_->write();

  cfg_prefix_ = "/arduino/";
  arduino_if_ =
      blackboard->open_for_writing<ArduinoInterface>(arduino_if_name_.c_str());
  arduino_if_->set_x_position(0);
  arduino_if_->set_y_position(arduino_if_->y_max() / 2.);
  arduino_if_->set_y_position(0);

  arduino_if_->set_final(true);
  arduino_if_->write();

  //setup robotino sensor interface for HavePuck detection
  robotino_sensor_if_ =
          blackboard->open_for_writing<RobotinoSensorInterface>(robotino_sensor_if_name_.c_str());

  tf_add_publisher("gripper_x_dyn");
  tf_add_publisher("gripper_y_dyn");
  tf_add_publisher("gripper_z_dyn");

  update_gripper_tfs(0,0,0);

  gripper_move_duration_ = 1;
  is_busy = false;
}

void GazsimGripperThread::finalize() { blackboard->close(gripper_if_);
                                       blackboard->close(arduino_if_);
                                       blackboard->close(robotino_sensor_if_);}

void GazsimGripperThread::loop() {
  // gripper_if_->set_final(__servo_if_left->is_final() &&
  // __servo_if_right->is_final());

    // initialize wait timer of movement of gripper is necessary
    if ( arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>() ||
        arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZRelMessage>() ){
        if (!is_busy){
            //logger->log_info(name(), "INIT WAIT FOR GRIPPER");
            gripper_cmd_start_time_ = fawkes::Time().in_sec();
            is_busy = true;
            return;
            // gripper movement in progress
        }else if (fawkes::Time().get_sec() - gripper_cmd_start_time_.get_sec() < gripper_move_duration_){
            //logger->log_info(name(), "WAITING FOR MOVEMENT %ld",fawkes::Time().get_sec() - gripper_cmd_start_time_.get_sec());
            return;
    // gripper movement finished
        } else { is_busy = false;}
    }


  while (!arduino_if_->msgq_empty()) {
    if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
      ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);
      update_gripper_tfs(msg->x(), msg->y(), msg->z());
      arduino_if_->set_x_position(msg->x());
      arduino_if_->set_y_position(msg->y());
      arduino_if_->set_z_position(msg->z());
    } else if (arduino_if_
                   ->msgq_first_is<ArduinoInterface::MoveXYZRelMessage>()) {
      ArduinoInterface::MoveXYZRelMessage *msg = arduino_if_->msgq_first(msg);
      update_gripper_tfs(arduino_if_->x_position() + msg->x(),
                         arduino_if_->y_position() + msg->y(),
                         arduino_if_->z_position() + msg->z());
      arduino_if_->set_x_position(arduino_if_->x_position() + msg->x());
      arduino_if_->set_y_position(arduino_if_->y_position() + msg->y());
      arduino_if_->set_z_position(arduino_if_->z_position() + msg->z());

      msgs::Int s;
      s.set_data(arduino_if_->z_position() + msg->z());
      set_conveyor_pub_->Publish(s);
      //    } else if (
      //    arduino_if_->msgq_first_is<ArduinoInterface::MoveUpwardsMessage>() )
      //    {
      //      ArduinoInterface::MoveUpwardsMessage *msg =
      //      arduino_if_->msgq_first(msg); arduino_if_->set_z_position(
      //      arduino_if_->z_position() - msg->num_mm() ); msgs::Int s;
      //      s.set_data( - msg->num_mm() );
      //      set_conveyor_pub_->Publish( s );
    }  else if (arduino_if_
                ->msgq_first_is<ArduinoInterface::OpenGripperMessage>()) {
                send_gripper_msg(1);
   }   else if (arduino_if_
                ->msgq_first_is<ArduinoInterface::CloseGripperMessage>()) {
                send_gripper_msg(0);
                gripper_if_->set_holds_puck(true);
   } else {
      logger->log_warn(name(), "Unknown Arduino message received");
    }
    arduino_if_->msgq_pop();
    arduino_if_->write();

  }

  logger->log_debug(name(),
                    "publish dyn TF");

  //publish tf
  fawkes::tf::StampedTransform transform;

  transform.frame_id = "gripper_x_origin";
  transform.child_frame_id = "gripper_x_dyn";
  transform.stamp = fawkes::Time();

  transform.setOrigin( fawkes::tf::Vector3( 0, 0, 0 ) );
  fawkes::tf::Quaternion q;
  q.setEuler(M_PI_2, M_PI_2, 0);
  transform.setRotation( q );

  tf_publisher->send_transform(transform);

//  transform.frame_id = "gripper_y_origin";
//  transform.child_frame_id = "gripper_y_dyn";

//  tf_publisher->send_transform(transform);

//  transform.frame_id = "gripper_z_origin";
//  transform.child_frame_id = "gripper_z_dyn";

//  tf_publisher->send_transform(transform);

}

void GazsimGripperThread::update_gripper_tfs(float x, float y, float z){

    tf::Transform t(tf::Quaternion(tf::Vector3(0, 0, 1), 0), tf::Vector3(x, y, z));
    tf_publishers["gripper_x_dyn"]->send_transform(t, fawkes::Time(), "gripper_x_origin", "gripper_x_dyn");
    tf_publishers["gripper_y_dyn"]->send_transform(t, fawkes::Time(), "gripper_y_origin", "gripper_y_dyn");
    tf_publishers["gripper_z_dyn"]->send_transform(t, fawkes::Time(), "gripper_z_origin", "gripper_z_dyn");

}

void GazsimGripperThread::send_gripper_msg(int value) {
  // send message to gazebo
  // 0 close, 1 open, 2 move
  msgs::Int msg;
  msg.set_data(value);
  set_gripper_pub_->Publish(msg);
}

void GazsimGripperThread::on_has_puck_msg(ConstIntPtr &msg) {
  // 1 means the gripper has a puck 0 not
    if (msg->data() > 0){
        robotino_sensor_if_->set_digital_in(0,false);
        robotino_sensor_if_->set_digital_in(1,true);
    }else{
        robotino_sensor_if_->set_digital_in(0,false);
        robotino_sensor_if_->set_digital_in(1,false);
    }
    robotino_sensor_if_->write();

  gripper_if_->set_holds_puck(msg->data() > 0);
  gripper_if_->write();
}
