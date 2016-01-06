/***************************************************************************
 *  bridge_thread.cpp - Thread to print the robot's position to the log
 *
  *  Created: Thu Aug  2015
 *  Copyright  2015 Mostafa Gomaa
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

#include "bridge_thread.h"
 
#include <tf/types.h>
#include <interfaces/Position3DInterface.h>

using namespace fawkes;

/** @class BridgeThread "bridge_thread.h"
 * Thread to print robot position to log.
 * @author Tim Niemueller
 */

/** Constructor. */
BridgeThread::BridgeThread()
  : Thread("BridgeThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
  
}

/** Destructor. */
BridgeThread::~BridgeThread()
{
}


void
BridgeThread::init()
{
//  pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");
//  world_ =new World("f");
 logger-> log_info("I CAN SEE THE WORLD","asddas");
 // time_var_=new Time(clock);
 // time_var_->stamp();

  proc_ = std::make_shared<BridgeBlackBoardProcessor> (logger, config, blackboard);

  fawkes_bridge_manager_=std::make_shared<GenericBridgeManager> (clock);
  fawkes_bridge_manager_->register_processor(proc_);

  web_server_=websocketpp::lib::make_shared<Web_server>(logger, fawkes_bridge_manager_);
  web_server_->run(6060);
}

void
BridgeThread::finalize()
{
 // blackboard->close(pose_if_);
  // delete proc_;
  // delete fawkes_bridge_manager_;
  // delete web_server_;//when do i actually need to teminate it..noty sure yet
}

void
BridgeThread::loop()
{
   fawkes_bridge_manager_->loop();
  // if (pose_if_->has_writer()) {
  //   pose_if_->read();
  //   double *r = pose_if_->rotation();
  //   tf::Quaternion pose_q(r[0], r[1], r[2], r[3]);
  //   logger->log_info(name(), "Pose: (%f,%f,%f)", pose_if_->translation(0),
  //                    pose_if_->translation(1), tf::get_yaw(pose_q));
  // } else {
  //   logger->log_warn(name(), "No writer for pose interface");
  // }

  // //  proc_->getAgentState();

  //Palying around with time

  // time_var_->stamp();
  // fawkes::Time now(clock);
  // now.stamp();
 
  //   std::cout<< "time_is" << now.in_msec() <<std::endl;
  //   std::cout<< "time_is" << (*time_var_).in_msec() <<std::endl;
    
  //   std::cout<< "time_is" << now.in_msec() - (*time_var_).in_msec() <<std::endl;
    
  //   std::cout<< "0000000000000000000000000000000000000"  <<std::endl;

}
