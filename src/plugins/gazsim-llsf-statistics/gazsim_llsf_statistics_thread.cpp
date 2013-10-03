/***************************************************************************
 *  gazsim_llsf_statistics_plugin.cpp - Plugin generates a statistic about
 *     a game
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

#include "gazsim_llsf_statistics_thread.h"

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

/** @class LlsfStatisticsSimThread "gazsim_llsf_statistics_thread.h"
 * Thread generates statistics of a llsf game in gazebo
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
LlsfStatisticsSimThread::LlsfStatisticsSimThread()
  : Thread("LlsfStatisticsSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void LlsfStatisticsSimThread::init()
{
  logger->log_debug(name(), "Initializing LLSF-Statistics Plugin");
  
  //create subscriber
  game_state_sub_ = gazebo_world_node->Subscribe(std::string("~/LLSFRbSim/GameState/"), &LlsfStatisticsSimThread::on_game_state_msg, this);

  //read config values

  //init statistics
  exp_points_ = 0;
  prod_points_ = 0;
  written_ = false;
}

void LlsfStatisticsSimThread::finalize()
{
}

void LlsfStatisticsSimThread::loop()
{
 
}

void LlsfStatisticsSimThread::on_game_state_msg(ConstGameStatePtr &msg)
{
  //logger->log_info(name(), "Got GameState message");
  if(msg->phase() == llsf_msgs::GameState::EXPLORATION)
  {
    exp_points_ = msg->points();
  }
  else if(msg->phase() == llsf_msgs::GameState::PRODUCTION)
  {
    prod_points_ = msg->points() - exp_points_;
  }
  else if(msg->phase() == llsf_msgs::GameState::POST_GAME)
  {
    if(!written_)
    {
      //TODO: write statistics to file/DB

      written_ = true;
    }
  }
}
