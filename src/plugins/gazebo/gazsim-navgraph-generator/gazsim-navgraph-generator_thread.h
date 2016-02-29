/***************************************************************************
 *  gazsim-navgraph-generator_thread.h - Thread for generating the navgraph without exploration phase
 *
 *  Created: Mon Feb 15 11:27:09 2016
 *  Copyright  2016  David Schmidt
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

#ifndef __PLUGINS_GAZSIM_NAVGRAPH_GENERATOR_THREAD_H_
#define __PLUGINS_GAZSIM_NAVGRAPH_GENERATOR_THREAD_H_

#include <core/threading/thread.h>
//#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
//#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>

/*
namespace fawkes {
  class Position3DInterface;
}
*/

//gazebo headers
#include <plugins/gazebo/aspect/gazebo.h>
//#include <gazebo/transport/TransportTypes.hh>

class GazsimNavgraphGeneratorThread
: public fawkes::Thread,
//public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
//public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  GazsimNavgraphGeneratorThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
 private:
  //Subscriber to receive tag_01 position from gazebo
  std::string tag_01_;
  gazebo::transport::SubscriberPtr subscriber_tag_01_;

  //copy of last msg to write the interface in the next loop
  gazebo::msgs::PosesStamped last_msg_;
  bool new_data_;

  //handler function for incoming messages about the machine light signals
  void on_tag_vision_msg(ConstPosesStampedPtr &msg);
};

#endif
