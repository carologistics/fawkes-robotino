/***************************************************************************
 *  gazsim_llsf_control_plugin.h - Plugin controls the llsf simulation
 *
 *  Created: Mon Sep 23 17:12:33 2013
 *  Copyright  2013 Frederik Zwilling
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

#ifndef __PLUGINS_GAZSIM_LLSF_CONTROL_THREAD_H_
#define __PLUGINS_GAZSIM_LLSF_CONTROL_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <interfaces/Position3DInterface.h>
#include <protobuf_msgs/GameState.pb.h>
#include <string.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


namespace fawkes {
  class Position3DInterface;
}

typedef const boost::shared_ptr<llsf_msgs::GameState const> ConstGameStatePtr;
typedef const boost::shared_ptr<llsf_msgs::SetGameState const> ConstSetGameStatePtr;

class LlsfControlSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  LlsfControlSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  //suscribers for gazebo nodes (refbox messages forwarded by gazsim-llsfrbcomm)
  gazebo::transport::SubscriberPtr game_state_sub_;
  //Publisher to start the refbox
  gazebo::transport::PublisherPtr set_game_state_pub_;

  //config values
  bool start_game_automatically_;
  float time_to_wait_before_start_;
  bool post_game_simulation_shutdown_;
  float time_to_wait_before_shutdown_;
  std::string fawkes_path_;
  std::string simulation_shutdown_script_;
  
  //handler functions
  void on_game_state_msg(ConstGameStatePtr &msg);

  //helper variables
  float start_time_;
  bool start_sent_;
  bool shutdown_initiated_;
  float shutdown_initiated_time_;
};

#endif
