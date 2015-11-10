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
 
#include "dispatcher.h"
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
//  proc_  = new BridgeProcessor(clips_env_mgr, logger, config, blackboard);
//  world_ =new World("f");
 logger-> log_info("I CAN SEE THE WORLD","asddas");

  websocketpp::lib::shared_ptr<Web_server> web_server=websocketpp::lib::make_shared<Web_server>();
  web_server->run(6060);
}

void
BridgeThread::finalize()
{
 // blackboard->close(pose_if_);
  //delete dispatcher_;
  delete web_server;
}

void
BridgeThread::loop()
{
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
  
}
