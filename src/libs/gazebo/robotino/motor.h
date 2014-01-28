/***************************************************************************
 *  motor.h - provides motor functionality
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

namespace gazebo
{
/*
   This class displayes messages form Fawkes in the Gazebo console
 */
  class Motor: public SimDevice
  {
  public:

    //Constructor
    Motor(physics::ModelPtr, transport::NodePtr);
    //Destructor
    ~Motor();
  
    virtual void init();
    virtual void create_publishers();
    virtual void create_subscribers();
    virtual void update();

  private:
        
    //Functions for recieving Messages (registerd via suscribers)
    void on_motor_move_msg(ConstVector3dPtr &msg);

    //Suscriber for MotorMove Interfaces from Fawkes
    transport::SubscriberPtr motor_move_sub_;


    //current movement commands:
    float vx_, vy_, vomega_;  
  };
}
