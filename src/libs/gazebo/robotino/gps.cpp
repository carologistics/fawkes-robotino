/***************************************************************************
 *  gps.cpp - Provides ground Truth position
 *
 *  Created: Mon Jul 29 17:33:31 2013
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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "gps.h"
#include "config.h"

using namespace gazebo;

Gps::Gps(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
Gps::~Gps()
{
}

void Gps::init()
{
  printf("Initialize Gps \n");
  last_sent_time_ = model->GetWorld()->GetSimTime().Double();
}

void Gps::create_publishers()
{
  this->gps_pub_ = this->node->Advertise<msgs::Pose>("~/RobotinoSim/Gps/");
}

void Gps::create_subscribers()
{
}

void Gps::update()
{
  //Send position information to Fawkes
  double time = model->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / 5.0))
  {
    last_sent_time_ = time;
    send_position();
  }
}

void Gps::send_position()
{
  if(gps_pub_->HasConnections())
  {
    //build message
    msgs::Pose posMsg;
    posMsg.mutable_position()->set_x(this->model->GetWorldPose().pos.x);
    posMsg.mutable_position()->set_y(this->model->GetWorldPose().pos.y);
    posMsg.mutable_position()->set_z(this->model->GetWorldPose().pos.z);
    posMsg.mutable_orientation()->set_x(this->model->GetWorldPose().rot.x);
    posMsg.mutable_orientation()->set_y(this->model->GetWorldPose().rot.y);
    posMsg.mutable_orientation()->set_z(this->model->GetWorldPose().rot.z);
    posMsg.mutable_orientation()->set_w(this->model->GetWorldPose().rot.w);

    //send
    gps_pub_->Publish(posMsg);
  }
}
