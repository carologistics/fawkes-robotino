
/***************************************************************************
 *  gazsim_gripper_thread.h - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:45:09 2015
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

#ifndef __PLUGINS_GAZSIM_GRIPPER_THREAD_H_
#define __PLUGINS_GAZSIM_GRIPPER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>
#include <interfaces/DynamixelServoInterface.h>
#include <utils/time/time.h>
#include <plugins/gazebo/aspect/gazebo.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>

namespace fawkes {
  class AX12GripperInterface;
  class LedInterface;
  class JointInterface;
}

class GazsimGripperThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::GazeboAspect
{
 public:
  GazsimGripperThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::AX12GripperInterface *gripper_if_;

  std::string  gripper_if_name_;
  std::string  cfg_prefix_;

  //Publisher to sent msgs to gazebo
  gazebo::transport::PublisherPtr set_gripper_pub_;

  void send_gripper_msg(int value);
};

#endif
