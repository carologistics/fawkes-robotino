
/***************************************************************************
 *  skiller_motor_state_thread.cpp - Indicate skiller & motor state through LED
 *
 *  Created: Fri Jun 14 15:07:42 2019
 *  Copyright  2019  Morian Sonnet
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

#include "skiller_motor_state_thread.h"

#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/SkillerInterface.h>

using namespace fawkes;

/** @class SkillerMotorStateThread "act_thread.h"
 * Set LED according to motor state.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerMotorStateThread::SkillerMotorStateThread()
    : Thread("SkillerMotorStateThread", Thread::OPMODE_WAITFORWAKEUP),
      BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT) {}

  cfg_skiller_ifid_ = config->get_string("/skiller_state/skiller-interface-id");
  cfg_rsens_ifid_ = config->get_string("/skiller_state/sensor-interface-id");
  cfg_digital_out_green_ = config->get_uint("/skiller_state/digital-out-green");
  cfg_digital_out_red_ = config->get_uint("/skiller_state/digital-out-red");
void SkillerMotorStateThread::init() {
  cfg_digital_out_yellow_ =
      config->get_uint("/skiller_state/digital-out-yellow");
  cfg_timeout_ = fawkes::Time(config->get_float("/skiller_state/timeout"));

  skiller_if_ =
      blackboard->open_for_reading<SkillerInterface>(cfg_skiller_ifid_.c_str());
  rsens_if_ = blackboard->open_for_reading<RobotinoSensorInterface>(
      cfg_rsens_ifid_.c_str());

  disable(cfg_digital_out_red_);
  disable(cfg_digital_out_green_);
  disable(cfg_digital_out_yellow_);
}

void SkillerMotorStateThread::finalize() {
  blackboard->close(skiller_if_);
  blackboard->close(rsens_if_);
}

void SkillerMotorStateThread::loop() {
  if (rsens_if_->has_writer() && skiller_if_->has_writer()) {
    rsens_if_->read();
    skiller_if_->read();
    SkillerInterface::SkillStatusEnum status = skiller_if_->status();
    if (status == SkillerInterface::S_RUNNING) {
      enable(cfg_digital_out_yellow_);
    } else {
      disable(cfg_digital_out_yellow_);
    }
    if (status == SkillerInterface::S_FINAL) {
      final_time_ = fawkes::Time();
      enable(cfg_digital_out_green_);
    }
    if (status == SkillerInterface::S_FAILED) {
      failed_time_ = fawkes::Time();
      enable(cfg_digital_out_red_);
    }

    if (fawkes::Time() - final_time_ > cfg_timeout_) {
      disable(cfg_digital_out_green_);
    }
    if (fawkes::Time() - failed_time_ > cfg_timeout_) {
      disable(cfg_digital_out_red_);
    }
  }
}

void SkillerMotorStateThread::enable(unsigned int output) {
  if (!rsens_if_->is_digital_out(output - 1)) {
    RobotinoSensorInterface::SetDigitalOutputMessage *msg =
        new RobotinoSensorInterface::SetDigitalOutputMessage(output, true);
    rsens_if_->msgq_enqueue(msg);
  }
  return;
}

void SkillerMotorStateThread::disable(unsigned int output) {
  if (rsens_if_->is_digital_out(output - 1)) {
    RobotinoSensorInterface::SetDigitalOutputMessage *msg =
        new RobotinoSensorInterface::SetDigitalOutputMessage(output, false);
    rsens_if_->msgq_enqueue(msg);
  }
  return;
}
