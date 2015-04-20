
/***************************************************************************
 *  gazsim_gripper_thread.h - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:54:42 2015
 *  Copyright  2015 Frederik Zwilling
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

#include "gazsim_gripper_thread.h"

#include <utils/math/angle.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/scoped_rwlock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/AX12GripperInterface.h>
#include <interfaces/LedInterface.h>
#include <interfaces/JointInterface.h>
#include <boost/lexical_cast.hpp>
#include <config/change_handler.h>

#include <cstdarg>
#include <cmath>
#include <unistd.h>

using namespace fawkes;

/** @class GazsimGripperThread "gazsim_gripper_thread.h"
 * Thread simulates the Gripper in Gazebo
 *
 * @author Frederik Zwilling
 */

/** Constructor. */

GazsimGripperThread::GazsimGripperThread()
  : Thread("GazsimGripperThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
  set_name("GazsimGripperThread()");
}


void
GazsimGripperThread::init()
{
  logger->log_debug(name(), 
		    "Initializing Simulation of the Light Front Plugin");
  
  gripper_if_name_ = config->get_string("/gazsim/gripper/if-name");
  cfg_prefix_ = config->get_string("/gazsim/gripper/cfg-prefix");

  //setup gripper if with default values
  gripper_if_ = blackboard->open_for_writing<AX12GripperInterface>(gripper_if_name_.c_str());
  gripper_if_->set_calibrated(true);
  gripper_if_->set_min_left(config->get_float(cfg_prefix_ + "left_min"));
  gripper_if_->set_max_left(config->get_float(cfg_prefix_ + "left_max"));
  gripper_if_->set_min_right(config->get_float(cfg_prefix_ + "right_min"));
  gripper_if_->set_max_right(config->get_float(cfg_prefix_ + "right_max"));
  gripper_if_->set_left_margin(config->get_float(cfg_prefix_ + "left_margin"));
  gripper_if_->set_right_margin(config->get_float(cfg_prefix_ + "right_margin"));
  gripper_if_->set_max_left_velocity(0);
  gripper_if_->set_max_right_velocity(0);
  gripper_if_->set_left_velocity(0);
  gripper_if_->set_right_velocity(0);
  gripper_if_->write();
}

void
GazsimGripperThread::finalize()
{
  blackboard->close(gripper_if_);
}

void
GazsimGripperThread::loop()
{
  // gripper_if_->set_final(__servo_if_left->is_final() && __servo_if_right->is_final());

  // process interface messages
  while (! gripper_if_->msgq_empty() ) {
    if (gripper_if_->msgq_first_is<AX12GripperInterface::CalibrateMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::GotoMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::TimedGotoMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::ParkMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::SetEnabledMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::SetVelocityMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::OpenMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::CloseMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::CloseLoadMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::Open_AngleMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::SetMarginMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::CenterMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::StopLeftMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::StopRightMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::StopMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else if (gripper_if_->msgq_first_is<AX12GripperInterface::FlushMessage>()) {
      logger->log_warn(name(), "%s is not implemented in the simulation.",
                       gripper_if_->msgq_first()->type());
    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    gripper_if_->msgq_pop();
    gripper_if_->write();
  }
}
