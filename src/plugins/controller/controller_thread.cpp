
/***************************************************************************
 *  controller_thread.cpp - Robotino Controller Thread
 *
 *  Created: Fri May 07 09:01:14 2017
 *  Copyright  2017  Christoph Henke
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

#include "controller_thread.h"

using namespace fawkes;

/** @class ControllerThread "controller_thread.h"
 * Controlling the robot pose
 * @author Christoph Henke
 */

/** Contructor. */
ControllerThread::ControllerThread()
  : Thread("ControllerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Method for initialization
 */
void
ControllerThread::init()
{
  try {
	controller_if_ = blackboard->open_for_writing<ControllerInterface>("Controller");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open controller interface "
             "interface for writing", name());
    logger->log_error(name(), e);
    throw;
  }

  // Initialize controller parameters
  kp_ = 0.;
  ki_ = 0.;
  kd_ = 0.;

  // Initialize control parameters
  robot_frame_ = "";
  target_frame_ = "";
  x_offset_ = 0.;
  y_offset_ = 0.;
  ori_offset_ = 0.;
  x_error_ = 0.;
  y_error_ = 0.;
  ori_error_ = 0.;
}

/** Method for finalization
 */
void
ControllerThread::finalize()
{
  // close interfaces
  try {
    blackboard->close(controller_if_);
  } catch (Exception& e) {
    logger->log_error(name(), "Closing interface failed!");
    logger->log_error(name(), e);
  }
}

/** Method for setting the current controller parameters
 */
void
ControllerThread::set_pid_parameters(const float& kp, const float& ki, const float& kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

/** Method for setting the current controller parameters
 */
void
ControllerThread::set_control_target(const std::string& robot_frame, const std::string& target_frame,
		 const float& x_offset, const float& y_offset, const float& ori_offset,
		 const float& x_error, const float& y_error, const float& ori_error)
{
  robot_frame_ = robot_frame;
  target_frame_ = target_frame;
  x_offset_ = x_offset;
  y_offset_ = y_offset;
  ori_offset_ = ori_offset;
  x_error_ = x_error;
  y_error_ = y_error;
  ori_error_ = ori_error;
}

/** Method for writing the current controller settings into the controller interface
 */
void
ControllerThread::write_current_control(const std::string& robot_frame, const std::string& target_frame,
		 const float& x_offset, const float& y_offset, const float& ori_offset,
		 const float& x_error, const float& y_error, const float& ori_error,
		 const float& kp, const float& ki, const float& kd)
{
	//Set the current controller settings to running
	controller_if_->set_controller_running(true);
	controller_if_->set_robot_frame(robot_frame.c_str());
	controller_if_->set_target_frame(target_frame.c_str());
	controller_if_->set_x_offset(x_offset);
	controller_if_->set_y_offset(y_offset);
	controller_if_->set_ori_offset(ori_offset);
	controller_if_->set_x_error(x_error);
	controller_if_->set_y_error(y_error);
	controller_if_->set_ori_error(ori_error);
	controller_if_->set_kp(kp);
	controller_if_->set_ki(ki);
	controller_if_->set_kd(kd);
	controller_if_->write();
}

void
ControllerThread::loop()
{
  while (! controller_if_->msgq_empty()) {

    if (ControllerInterface::PidControlMessage *msg = controller_if_->msgq_first_safe(msg)) {
      logger->log_info(name(), "PID control message received");
      // Writing the last called service and the according parameters into the blackboard
      write_current_control(msg->robot_frame(), msg->target_frame(),
    		  msg->x_offset(), msg->y_offset(), msg->ori_offset(),
			  msg->x_error(), msg->y_error(), msg->ori_error(),
			  msg->kp(), msg->ki(), msg->kd());
      controller_if_->set_controller_running(true);
      controller_if_->set_robot_frame(msg->robot_frame());
      controller_if_->write();
      // Setting the controller parameters
      set_pid_parameters(msg->kp(), msg->ki(), msg->kd());
      // Setting the controller target
      set_control_target(msg->robot_frame(), msg->target_frame(),
    		  msg->x_offset(), msg->y_offset(), msg->ori_offset(),
			  msg->x_error(), msg->y_error(), msg->ori_error());
    }
    controller_if_->msgq_pop();
  } // while
}

