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

#include <aspect/logging.h>
#include <interfaces/Position3DInterface.h>
#include <tf/types.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

void
LlsfControlSimThread::init()
{
	logger->log_debug(name(), "Initializing LLSF-Control Plugin");

	// create subscriber
	game_state_sub_ = gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/game-state"),
	                                               &LlsfControlSimThread::on_game_state_msg,
	                                               this);

	// Create Publisher
	set_game_state_pub_ = gazebo_world_node->Advertise<llsf_msgs::SetGameState>(
	  config->get_string("/gazsim/topics/set-game-state"));
	set_game_phase_pub_ = gazebo_world_node->Advertise<llsf_msgs::SetGamePhase>(
	  config->get_string("/gazsim/topics/set-game-phase"));
	set_team_name_pub_ = gazebo_world_node->Advertise<llsf_msgs::SetTeamName>(
	  config->get_string("/gazsim/topics/set-team-name"));

	// read config values
	post_game_simulation_shutdown_ =
	  config->get_bool("/gazsim/llsf-control/simulation-shutdown-after-game");
	team_cyan_name_    = config->get_string("/gazsim/llsf-control/team-cyan-name");
	team_magenta_name_ = config->get_string("/gazsim/llsf-control/team-magenta-name");
	fawkes_path_       = config->get_string("/gazsim/llsf-control/fawkes-path");
	simulation_shutdown_script_ =
	  config->get_string("/gazsim/llsf-control/simulation-shutdown-script");
	start_game_automatically_  = config->get_bool("/gazsim/llsf-control/start-game-automatically");
	time_to_wait_before_start_ = config->get_float("/gazsim/llsf-control/time-to-wait-before-start");
	time_to_wait_before_set_team_ =
	  config->get_float("/gazsim/llsf-control/time-to-wait-before-set-team");
	time_to_wait_before_shutdown_ =
	  config->get_float("/gazsim/llsf-control/time-to-wait-before-shutdown");

	start_sent_ = false;
	team_sent_  = false;
	start_time_.set_clock(clock);
	start_time_.stamp();
	logger->log_info(name(), "Startup");
	shutdown_initiated_ = false;
}

void
LlsfControlSimThread::finalize()
{
}

void
LlsfControlSimThread::loop()
{
	fawkes::Time now(clock);
	if (!team_sent_ && (now - &start_time_) > time_to_wait_before_set_team_) {
		logger->log_info(name(), "Setting teams");
		// set team names
		llsf_msgs::SetTeamName msg_team_cyan;
		msg_team_cyan.set_team_name(team_cyan_name_);
		msg_team_cyan.set_team_color(llsf_msgs::Team::CYAN);
		set_team_name_pub_->Publish(msg_team_cyan);

		llsf_msgs::SetTeamName msg_team_magenta;
		msg_team_magenta.set_team_name(team_magenta_name_);
		msg_team_magenta.set_team_color(llsf_msgs::Team::MAGENTA);
		set_team_name_pub_->Publish(msg_team_magenta);

		// start setup phase
		llsf_msgs::SetGameState msg_state;
		msg_state.set_state(llsf_msgs::GameState::RUNNING);
		set_game_state_pub_->Publish(msg_state);
		team_sent_ = true;
	}
	if (!start_sent_ && (now - &start_time_) > time_to_wait_before_start_) {
		logger->log_info(name(), "Starting game");
		// let the refbox start the game
		llsf_msgs::SetGamePhase msg_phase;
		msg_phase.set_phase(llsf_msgs::GameState::EXPLORATION);
		set_game_phase_pub_->Publish(msg_phase);

		start_sent_ = true;
	}

	if (shutdown_initiated_ && (now - &shutdown_initiated_time_) > time_to_wait_before_shutdown_) {
		logger->log_info(name(), "shutting down");
		std::string command = fawkes_path_ + simulation_shutdown_script_;
		int         schnurz = system(command.c_str());
		// just avoid warning that the return value of system() is ignored
		schnurz++;
	}
}

void
LlsfControlSimThread::on_game_state_msg(ConstGameStatePtr &msg)
{
	// logger->log_info(name(), "Got GameState message");
	if (msg->phase() == llsf_msgs::GameState::POST_GAME) {
		if (post_game_simulation_shutdown_ && !shutdown_initiated_) {
			logger->log_info(name(), "set shutdown timer %f", time_to_wait_before_shutdown_);
			// countdown for simulation shutdown
			shutdown_initiated_time_.set_clock(clock);
			shutdown_initiated_time_.stamp();
			shutdown_initiated_ = true;
		}
	}
}
