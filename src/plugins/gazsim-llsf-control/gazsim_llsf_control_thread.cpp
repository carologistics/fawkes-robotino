/***************************************************************************
 *  gazsim_llsf_control_plugin.cpp - Plugin controls the llsf simulation
 *
 *  Created: Thu Oct 03 13:14:33 2013
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

#include "gazsim_llsf_control_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <stdlib.h>

#include <interfaces/Position3DInterface.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class LlsfControlSimThread "gazsim_llsf_control_thread.h"
 * Thread controls the llsf simulation
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
LlsfControlSimThread::LlsfControlSimThread()
  : Thread("LlsfControlSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void LlsfControlSimThread::init()
{
  logger->log_debug(name(), "Initializing LLSF-Control Plugin");
  
  //create subscriber
  game_state_sub_ = gazebo_world_node->Subscribe(std::string("~/LLSFRbSim/GameState/"), &LlsfControlSimThread::on_game_state_msg, this);

  //Create Publisher
  set_game_state_pub_ = gazebo_world_node->Advertise<llsf_msgs::SetGameState>("~/LLSFRbSim/SetGameState/");

  //read config values
  post_game_simulation_shutdown_ = config->get_bool("/gazsim/llsf-control/simulation-shutdown-after-game");
  fawkes_path_ = config->get_string("/gazsim/llsf-control/fawkes-path");
  simulation_shutdown_script_ = config->get_string("/gazsim/llsf-control/simulation-shutdown-script");
  start_game_automatically_ = config->get_bool("/gazsim/llsf-control/start-game-automatically");
  time_to_wait_before_start_ = config->get_float("/gazsim/llsf-control/time-to-wait-before-start");
  time_to_wait_before_shutdown_ = config->get_float("/gazsim/llsf-control/time-to-wait-before-shutdown");

  start_sent_ = false;
  start_time_ = clock->now().in_sec();
  shutdown_initiated_ = false;
}

void LlsfControlSimThread::finalize()
{
}

void LlsfControlSimThread::loop()
{
  if(!start_sent_ && (clock->now().in_sec() - start_time_) > time_to_wait_before_start_)
  {
    //let the refbox start the game
    llsf_msgs::SetGameState msg;
    msg.set_state(llsf_msgs::GameState::RUNNING);
    set_game_state_pub_->Publish(msg);

    start_sent_ = true;
  }

  if(shutdown_initiated_ && (clock->now().in_sec() - shutdown_initiated_time_) > time_to_wait_before_shutdown_)
  {
    logger->log_info(name(), "shutting down, ttwbs: %f, now: %f, sit: %f", time_to_wait_before_shutdown_, clock->now().in_sec(), shutdown_initiated_time_);
      std::string command = fawkes_path_ + simulation_shutdown_script_;
      system(command.c_str());
  }
}

void LlsfControlSimThread::on_game_state_msg(ConstGameStatePtr &msg)
{
  //logger->log_info(name(), "Got GameState message");
  if(msg->phase() == llsf_msgs::GameState::POST_GAME)
  {
    if(post_game_simulation_shutdown_)
    {
      //countdown for simulation shutdown
      shutdown_initiated_time_ = clock->now().in_sec();
      shutdown_initiated_ = true;
    }
  }
}
