/***************************************************************************
 *  time_sync.cpp - syncs the simulation time with fawkes by sending
 *                the current real time factor
 *
 *  Created: Tue Sep 24 14:10:51 2013
 *  Copyright  2013  Frederik Zwilling
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
#include <boost/bind.hpp>
#include <gazebo.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include <physics/physics.hh>
#include <llsf_msgs/SimTimeSync.pb.h>
#include <cmath>

#include "time_sync.h"

using namespace gazebo;

TimeSync::TimeSync(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  gazebo_node_ = gazebo_node;
  world_ = world;

  //create publisher
  this->time_sync_pub_ = gazebo_node_->Advertise<llsf_msgs::SimTimeSync>("~/gazsim/time-sync/");
  
  //init variables
  last_real_time_ = 0.0;
  last_sim_time_ = 0.0;
}

TimeSync::~TimeSync()
{
}

void TimeSync::send_time_sync()
{
  double sim_time = world_->GetSimTime().Double();
  double real_time = world_->GetRealTime().Double();

  llsf_msgs::SimTimeSync msg;

  llsf_msgs::Time* time = msg.mutable_sim_time();
  time->set_sec(sim_time); //automatically rounded to integer
  time->set_nsec((sim_time - time->sec()) * 1000000000.f);

  //Calculate real time factor (did not find it in gazebo api)
  double real_time_factor = (sim_time - last_sim_time_) / (real_time - last_real_time_);
  msg.set_real_time_factor(real_time_factor);

  msg.set_paused(!world_->GetRunning());
  time_sync_pub_->Publish(msg);

  last_sim_time_ = sim_time;
  last_real_time_ = real_time;
}
