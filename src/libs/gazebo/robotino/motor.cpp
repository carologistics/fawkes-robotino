/***************************************************************************
 *  motor.cpp - provides motor functionality
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
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "motor.h"

using namespace gazebo;

Motor::Motor(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
Motor::~Motor()
{
}

void Motor::init()
{
  printf("Initialize Motor\n");
  //initialize movement commands:
  vx_ = 0.0;
  vy_ = 0.0;
  vomega_ = 0.0;
}

void Motor::create_publishers()
{
}

void Motor::create_subscribers()
{
  this->motor_move_sub_ = this->node->Subscribe(std::string("~/RobotinoSim/MotorMove/"), &Motor::on_motor_move_msg, this);
}

void Motor::update()
{
  //Apply movement command
  float x,y;
  float yaw = this->model->GetWorldPose().rot.GetAsEuler().z;
  //foward part
  x = cos(yaw) * vx_;
  y = sin(yaw) * vx_;
  //sideways part
  x += cos(yaw + 3.1415926f / 2) * vy_;
  y += sin(yaw + 3.1415926f / 2) * vy_;
  // Apply velocity to the model.
  this->model->SetLinearVel(math::Vector3(x, y, 0));
  this->model->SetAngularVel(math::Vector3(0, 0, vomega_));
}

void Motor::on_motor_move_msg(ConstVector3dPtr &msg)
{
  //printf("Got MotorMove Msg!!! %f %f %f\n", msg->x(), msg->y(), msg->z());
  //Transform relative motion into ablosulte motion
  vx_ = msg->x();
  vy_ = msg->y();
  vomega_ = msg->z();
}
