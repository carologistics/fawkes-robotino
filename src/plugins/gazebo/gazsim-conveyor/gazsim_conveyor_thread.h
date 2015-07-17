
/***************************************************************************
 *  gazsim_conveyor_thread.h - Plugin used to simulate a conveyor vision
 *
 *  Created: Fri Jul 10 11:27:12 2015
 *  Copyright  2015 Randolph Maaßen
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

#ifndef __PLUGINS_GAZSIM_CONVEYOR_THREAD_H_
#define __PLUGINS_GAZSIM_CONVEYOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>
#include <interfaces/Position3DInterface.h>
#include <utils/time/time.h>
#include <plugins/gazebo/aspect/gazebo.h>

#include <llsf_msgs/ConveyorVisionResult.pb.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>

#define MAX_LOOP_COUNT_TO_INVISIBLE 5

typedef const boost::shared_ptr<llsf_msgs::ConveyorVisionResult const> ConstConveyorVisionResultPtr;

class GazsimConveyorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::GazeboAspect
{
 public:
  GazsimConveyorThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::Position3DInterface *pos_if_;

  std::string  pos_if_name_;
  std::string  cfg_prefix_;
  
  gazebo::transport::SubscriberPtr conveyor_vision_sub_;
  void on_conveyor_vision_msg(ConstConveyorVisionResultPtr &msg);
  
  int32_t loopcount_;
  
  //copy of last msg to write the interface in the next loop
  llsf_msgs::ConveyorVisionResult last_msg_;
  bool new_data_; 
};

#endif
