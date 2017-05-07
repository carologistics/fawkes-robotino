
/***************************************************************************
 *  controller_thread.h - Robotino Controller Thread
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
 
#ifndef __CONTROLLER_THREAD_H_
#define __CONTROLLER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <plugins/ros/aspect/ros.h>

#include <interfaces/ControllerInterface.h>

#include <ros/ros.h>

namespace fawkes {
  class ControllerInterface;
}

class ControllerThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ROSAspect
{
 public:
  ControllerThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 // methods for setting the controller and its parameters
 void set_pid_parameters(const float& kp, const float& ki, const float& kd);

 // methods for setting the target parameters
 void set_control_target(const std::string& robot_frame, const std::string& target_frame,
		 const float& x_offset, const float& y_offset, const float& ori_offset,
		 const float& x_error, const float& y_error, const float& ori_error);

 // method for writing the current controller settings into the controller interface
 void write_current_control(const std::string& robot_frame, const std::string& target_frame,
 		 const float& x_offset, const float& y_offset, const float& ori_offset,
 		 const float& x_error, const float& y_error, const float& ori_error,
		 const float& kp, const float& ki, const float& kd);

 private:

  // Controller interface
  fawkes::ControllerInterface *controller_if_;

  // Control parameters
  std::string robot_frame_;
  std::string target_frame_;
  float x_offset_;
  float y_offset_;
  float ori_offset_;
  float x_error_;
  float y_error_;
  float ori_error_;

  // Controller parameters
  float kp_;
  float ki_;
  float kd_;

};

#endif /* __ROS_DYNAMIC_RECONFIGURE_THREAD_H_ */
