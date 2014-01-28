/***************************************************************************
 *  laserSensor.cpp - Provides laser sensor data
 *
 *  Created: Wed Aug 07 18:22:45 2013
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
#include <math.h>
#include <string.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "laserSensor.h"

using namespace gazebo;

LaserSensor::LaserSensor(physics::ModelPtr model, transport::NodePtr node, sensors::SensorPtr sensorPtr)
 : SimDevice(model, node)
{
  this->parent_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorPtr);
}
LaserSensor::~LaserSensor()
{
}

void LaserSensor::init()
{
  printf("Initialize LaserSensor \n");

  //Register OnNewLaserScans function
  this->new_laser_scans_connection_ = this->parent_sensor_->GetLaserShape()->ConnectNewLaserScans(boost::bind(&LaserSensor::on_new_laser_scans, this));
}

void LaserSensor::create_publishers()
{
  this->laser_pub_ = this->node->Advertise<msgs::LaserScan>("~/RobotinoSim/LaserSensor/");
}

void LaserSensor::create_subscribers()
{
}

void LaserSensor::update()
{
  //sending the laser scans happens in OnNewLaserScans()
}

void LaserSensor::on_new_laser_scans()
{
  if(laser_pub_->HasConnections())
  {
    //Get relevant data
    int numRays = parent_sensor_->GetRangeCount();
    float angleMin = parent_sensor_->GetAngleMin().Radian();
    float angleMax = parent_sensor_->GetAngleMax().Radian();
    float angleStep = parent_sensor_->GetAngleResolution();
    float rangeMin = parent_sensor_->GetRangeMin();
    float rangeMax = parent_sensor_->GetRangeMax();
    

    //create Protobuf message
    msgs::LaserScan laserMsg;
    laserMsg.set_frame("/base_laser");
    laserMsg.set_angle_min(angleMin);
    laserMsg.set_angle_max(angleMax);
    laserMsg.set_angle_step(angleStep);
    laserMsg.set_range_min(rangeMin);
    laserMsg.set_range_max(rangeMax);
    for(int i = 0; i < numRays; i++)
    {
    
      laserMsg.add_ranges(parent_sensor_->GetRange(i));
      //laserMsg.add_intensities(-1);//I think I don't need the intensity
      //printf("Ray number %d range %f\n", i, parent_sensor_->GetRange(i));
    }
    //dummy for world pose
    laserMsg.mutable_world_pose()->mutable_position()->set_x(0);
    laserMsg.mutable_world_pose()->mutable_position()->set_y(0);
    laserMsg.mutable_world_pose()->mutable_position()->set_z(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_x(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_y(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_z(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_w(0);
 
    //send message
    laser_pub_->Publish(laserMsg);
    }
}
