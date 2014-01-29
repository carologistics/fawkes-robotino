/***************************************************************************
 *  frontCamera.cpp - Provides laser sensor data
 *
 *  Created: Thu Sep 19 12:46:43 2013
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
#include <string.h>
#include <gazebo/transport/transport.hh>
#include "simDevice.h"
#include "frontCamera.h"
#include "config.h"

using namespace gazebo;

FrontCamera::FrontCamera(physics::ModelPtr model, transport::NodePtr node, sensors::SensorPtr sensorPtr)
 : SimDevice(model, node)
{
  this->parent_sensor_ = boost::dynamic_pointer_cast<sensors::CameraSensor>(sensorPtr);
}
FrontCamera::~FrontCamera()
{
}

void FrontCamera::init()
{
  printf("Initialize FrontCamera \n");
  last_sent_time_ = model->GetWorld()->GetSimTime().Double();
}

void FrontCamera::create_publishers()
{
}

void FrontCamera::create_subscribers()
{
}

void FrontCamera::update()
{
  double time = model->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / FRONT_CAMERA_SEND_FREQUENCY))
  {
    last_sent_time_ = time;
    process_image();
  }
}

void FrontCamera::process_image()
{
  //printf("Processing Image \n");
}
