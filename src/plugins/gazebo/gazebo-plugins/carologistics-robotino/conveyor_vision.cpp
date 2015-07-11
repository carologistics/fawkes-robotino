/***************************************************************************
 *  puck_detection.cpp - provides puck-positions
 *
 *  Created: Thu Aug 29 10:05:15 2013
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
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "conveyor_vision.h"
#include <llsf_msgs/ConveyorVisionResult.pb.h>
#include "../llsf/data_table.h"
#include "config.h"
#include "libs/utils/math/common.h"

using namespace gazebo;

/** Constructor
 * @param model Model to the gazebo model of the plugin
 * @param node Transport node to publish and subscribe messages on
 */
ConveyorVision::ConveyorVision(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
ConveyorVision::~ConveyorVision()
{
}

void ConveyorVision::init()
{
  printf("Initialize PuckDetection \n");
  table_ = LlsfDataTable::get_table();
}

void ConveyorVision::create_publishers()
{
  this->puck_position_pub_ = this->node->Advertise<llsf_msgs::PuckDetectionResult>("~/RobotinoSim/ConveyorVisionResult/");
}

void ConveyorVision::create_subscribers()
{
  //no subscribers
}

void ConveyorVision::update()
{
  //Send position information to Fawkes
  double time = model->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / PUCK_DETECTION_SEND_FREQUENCY))
  {
    last_sent_time_ = time;
    send_puck_positions();
  }
}

void ConveyorVision::send_puck_positions()
{
  //send ground truth puck positions to fawkes
  //fawkes chooses which of them are too far away to detect
  
  //build Protobuf Message
  llsf_msgs::ConveyorVisionResult msg;
  llsf_msgs::Pose3D *pose = msg.positions();
  gazebo::math::Pose bot_pose = model->GetWorldPose();
  for(Machine machine: table_->get_machines())
  {
    if(fawkes::point_dist(machine.x,machine.y,bot_pose.pos[0],bot_pose.pos[1]) < 1.5)
    {
      gazebo::math::Pose machine_pose(machine.x,machine.y,0,0,0,0);
      machine_pose = machine_pose - bot_pose;
      pose->set_x(machine_pose.pos[0]);
      pose->set_y(machine_pose.pos[2]);
      pose->set_z(0);
      pose->set_ori_x(0);
      pose->set_ori_y(0);
      pose->set_ori_z(0);
      pose->set_ori_w(0);
      break;
    }
  }
  pose->mutable_timestamp()->set_sec(0);
  pose->mutable_timestamp()->set_nsec(0);
  //send it
  conveyor_position_pub_->Publish(msg);
}
