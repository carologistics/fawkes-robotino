/***************************************************************************
 *  gyro.cpp - provides gyro values
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
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "gyro.h"

using namespace gazebo;

Gyro::Gyro(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
Gyro::~Gyro()
{
}

void Gyro::init()
{
  printf("Initialize Gyro \n");
}

void Gyro::create_publishers()
{
  this->gyro_pub_ = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/Gyro/");
}

void Gyro::create_subscribers()
{

}

void Gyro::update()
{
  //Send gyro information to Fawkes
  send_gyro();
}

void Gyro::send_gyro()
{
  if(gyro_pub_->HasConnections())
  {
    //Read gyro from simulation
    float roll = this->model->GetWorldPose().rot.GetAsEuler().x;
    float pitch = this->model->GetWorldPose().rot.GetAsEuler().y;
    float yaw = this->model->GetWorldPose().rot.GetAsEuler().z;

    //build message
    msgs::Vector3d gyroMsg;
    gyroMsg.set_x(roll);
    gyroMsg.set_y(pitch);
    gyroMsg.set_z(yaw);

    //send
    gyro_pub_->Publish(gyroMsg);
  }
}
