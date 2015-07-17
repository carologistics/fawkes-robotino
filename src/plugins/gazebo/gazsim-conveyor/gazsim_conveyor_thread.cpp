
/***************************************************************************
 *  gazsim_conveyor_thread.h - Plugin used to simulate a conveyor vision
 *
 *  Created: Fri Jul 10 11:27:12 2015
 *  Copyright  2015 Randolph Maaßen
 *
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

#include "gazsim_conveyor_thread.h"

#include <utils/math/angle.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/wait_condition.h>
#include <boost/lexical_cast.hpp>
#include <config/change_handler.h>

#include <cstdarg>
#include <cmath>
#include <unistd.h>

using namespace fawkes;
using namespace gazebo;

/** @class GazsimConveyorThread "gazsim_conveyor_thread.h"
 * Thread simulates the Conveyor Vision in Gazebo
 *
 * @author Frederik Zwilling
 */

/** Constructor. */

GazsimConveyorThread::GazsimConveyorThread()
  : Thread("GazsimConveyorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
  set_name("GazsimConveyorThread()");
  loopcount_ = 0;
}


void
GazsimConveyorThread::init()
{
  logger->log_debug(name(), 
		    "Initializing Simulation of the Conveyor Vision Plugin");
  
  //conveyor_if_name_ = "conveyor_0";//config->get_string("/gazsim/gripper/if-name");
  pos_if_name_ = "mps_conveyor";
  cfg_prefix_ = "";//config->get_string("/gazsim/gripper/cfg-prefix");

  //setup Position3DInterface if with default values
  pos_if_ = blackboard->open_for_writing<Position3DInterface>(pos_if_name_.c_str());
  
  conveyor_vision_sub_ = gazebonode->Subscribe("~/RobotinoSim/ConveyorVisionResult/", &GazsimConveyorThread::on_conveyor_vision_msg, this);
  
}

void
GazsimConveyorThread::finalize()
{
  blackboard->close(pos_if_);
}

void
GazsimConveyorThread::loop()
{
  pos_if_->set_frame("base_conveyor");
  if(new_data_)
  {
    double trans[] = {last_msg_.positions().x(), last_msg_.positions().y(), last_msg_.positions().z()};
    double rot[] = {last_msg_.positions().ori_x(), last_msg_.positions().ori_y(), last_msg_.positions().ori_z(), last_msg_.positions().ori_w()};
    pos_if_->set_translation(trans);
    pos_if_->set_rotation(rot);
    pos_if_->set_visibility_history(loopcount_);
    pos_if_->write();
    new_data_=false;
  }
  else if((loopcount_ - pos_if_->visibility_history()) > MAX_LOOP_COUNT_TO_INVISIBLE)
  {
    pos_if_->set_visibility_history(-1);
    pos_if_->write();
    loopcount_ = -1;
  }
  loopcount_++;
}

void
GazsimConveyorThread::on_conveyor_vision_msg(ConstConveyorVisionResultPtr &msg)
{
  last_msg_.CopyFrom(*msg);
  new_data_ = true;
}
