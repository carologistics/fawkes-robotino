/***************************************************************************
 *  frontCamera.h - provides a image stream of the front camera
 *
 *  Created: Thu Sep 19 12:43:12 2013
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
#include <sensors/SensorTypes.hh>
#include <sensors/CameraSensor.hh>

namespace gazebo
{
  /*
    This class gets image data from the mounted front camera and sends it to fawkes

    The most config values for the sensor are located in the model.sdf of the hokuyo sensor.
  */
  class FrontCamera : public SimDevice
  {
    public: 
    
    //Constructor
    FrontCamera(physics::ModelPtr, transport::NodePtr, sensors::SensorPtr sensorPtr);

    //Destructor
    ~FrontCamera();


    virtual void init();
    virtual void create_publishers();
    virtual void create_subscribers();
    virtual void update();


    //what happens if the sensor has camera image
    void process_image();

    private:
    
    //connection of the sensor
    event::ConnectionPtr new_camera_connection_;

    //Pointer to the camera
    sensors::CameraSensorPtr parent_sensor_;

    //Publisher for communication to fawkes
    //transport::PublisherPtr laser_pub_;

  };
}
