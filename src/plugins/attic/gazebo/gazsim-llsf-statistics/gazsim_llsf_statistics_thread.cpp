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

#include <aspect/logging.h>
#include <interfaces/Position3DInterface.h>
#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>
#include <tf/types.h>
#include <utils/misc/string_conversions.h>

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
using namespace mongo;

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

void
LlsfStatisticsSimThread::init()
{
	logger->log_debug(name(), "Initializing LLSF-Statistics Plugin");

	// create subscriber
	game_state_sub_ = gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/game-state"),
	                                               &LlsfStatisticsSimThread::on_game_state_msg,
	                                               this);

	// read config values
	db_name_       = config->get_string("/gazsim/llsf-statistics/db-name");
	collection_    = config->get_string("/gazsim/llsf-statistics/collection");
	namespace_     = db_name_ + "." + collection_;
	configuration_ = config->get_string("/gazsim/llsf-statistics/configuration-name");
	replay_        = config->get_string("/gazsim/llsf-statistics/log");
	run_           = config->get_uint("/gazsim/llsf-statistics/run");

	// init statistics
	exp_points_cyan_     = 0;
	prod_points_cyan_    = 0;
	exp_points_magenta_  = 0;
	prod_points_magenta_ = 0;
	written_             = false;

	mongodb_ = mongodb_client;
}

void
LlsfStatisticsSimThread::finalize()
{
	write_statistics();
}

void
LlsfStatisticsSimThread::loop()
{
}

void
LlsfStatisticsSimThread::on_game_state_msg(ConstGameStatePtr &msg)
{
	// logger->log_info(name(), "Got GameState message");
	if (msg->phase() == llsf_msgs::GameState::EXPLORATION) {
		exp_points_magenta_ = msg->points_magenta();
		exp_points_cyan_    = msg->points_cyan();
	} else if (msg->phase() == llsf_msgs::GameState::PRODUCTION) {
		prod_points_magenta_ = msg->points_magenta() - exp_points_magenta_;
		prod_points_cyan_    = msg->points_cyan() - exp_points_cyan_;
	} else if (msg->phase() == llsf_msgs::GameState::POST_GAME) {
		write_statistics();
	}
}

void
LlsfStatisticsSimThread::write_statistics()
{
	if (!written_) {
		written_ = true;

		// get refbox game summery from ninja bash script
		std::string refbox_log_file = replay_ + "/refbox.log";
		std::string command         = StringConversions::resolve_path(
                            config->get_string("/gazsim/llsf-statistics/get-refbox-summary-script"))
		                      + " " + refbox_log_file;
		logger->log_info(name(), "command %s\n", command.c_str());
		FILE *      bash_output      = popen(command.c_str(), "r");
		std::string refbox_score_log = "";
		if (bash_output) {
			logger->log_info(name(), "shuffeling");
			char buffer[100];
			while (!feof(bash_output)) {
				if (fgets(buffer, 100, bash_output) == NULL) {
					break;
				}
				refbox_score_log += buffer;
			}
			fclose(bash_output);
		}
		logger->log_info(name(), "summery %s\n", refbox_score_log.c_str());

		BSONObjBuilder builder;
		builder.append("configuration", configuration_.c_str());
		builder.append("run", run_);
		builder.append("exp-points-cyan", exp_points_cyan_);
		builder.append("prod-points-cyan", prod_points_cyan_);
		builder.append("total-points-cyan", exp_points_cyan_ + prod_points_cyan_);
		builder.append("exp-points-magenta", exp_points_magenta_);
		builder.append("prod-points-magenta", prod_points_magenta_);
		builder.append("total-points-magenta", exp_points_magenta_ + prod_points_magenta_);
		builder.append("logs_and_replay", replay_.c_str());
		builder.append("refbox_game_summery", refbox_score_log.c_str());
		BSONObj entry = builder.obj();
		mongodb_->insert(namespace_.c_str(), entry);
	}
}
