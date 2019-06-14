
/***************************************************************************
 *  skiller_state_thread.cpp - Indicate skiller state through LED
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

#include "skiller_state_thread.h"

#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/SkillerInterface.h>

using namespace fawkes;

/** @class SkillerStateThread "act_thread.h"
 * Set LED according to motor state.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerStateThread::SkillerStateThread()
    : Thread("SkillerStateThread", Thread::OPMODE_WAITFORWAKEUP),
      BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT), final_ctr_(0),
      failed_ctr_(0) {}

void SkillerStateThread::init() {
  cfg_skiller_ifid_ = config->get_string("/skiller_state/skiller-interface-id");
  cfg_rsens_ifid_ = config->get_string("/skiller_state/sensor-interface-id");
  cfg_digital_out_green_ = config->get_uint("/skiller_state/digital-out-green");
  cfg_digital_out_red_ = config->get_uint("/skiller_state/digital-out-red");
  cfg_digital_out_yellow_ =
      config->get_uint("/skiller_state/digital-out-yellow");
  cfg_timeout_ = config->get_uint("/skiller_state/timeout");

  skiller_if_ =
      blackboard->open_for_reading<SkillerInterface>(cfg_skiller_ifid_.c_str());
  rsens_if_ = blackboard->open_for_reading<RobotinoSensorInterface>(
      cfg_rsens_ifid_.c_str());

  disable(cfg_digital_out_red_);
  disable(cfg_digital_out_green_);
  disable(cfg_digital_out_yellow_);
}

void SkillerStateThread::finalize() {
  blackboard->close(skiller_if_);
  blackboard->close(rsens_if_);
}

void SkillerStateThread::loop() {
  if (rsens_if_->has_writer() && skiller_if_->has_writer()) {
    SkillerInterface::SkillStatusEnum status = skiller_if_->status();
    if (status == SkillerInterface::S_RUNNING) {
      enable(cfg_digital_out_yellow_);
    } else {
      disable(cfg_digital_out_yellow_);
    }
    if (status == SkillerInterface::S_FINAL) {
      final_ctr_ = cfg_timeout_;
      enable(cfg_digital_out_green_);
    }
    if (status == SkillerInterface::S_FAILED) {
      failed_ctr_ = cfg_timeout_;
      enable(cfg_digital_out_red_);
    }

    --final_ctr_;
    --failed_ctr_;

    if (final_ctr_ <= 0) {
      final_ctr_ = 0;
      disable(cfg_digital_out_green_);
    }
    if (failed_ctr_ <= 0) {
      failed_ctr_ = 0;
      disable(cfg_digital_out_red_);
    }
  }
}

void SkillerStateThread::enable(unsigned int output) {
  if (!rsens_if_->is_digital_out(output - 1)) {
    RobotinoSensorInterface::SetDigitalOutputMessage *msg =
        new RobotinoSensorInterface::SetDigitalOutputMessage(output, true);
    rsens_if_->msgq_enqueue(msg);
  }
  return;
}

void SkillerStateThread::disable(unsigned int output) {
  if (rsens_if_->is_digital_out(output - 1)) {
    RobotinoSensorInterface::SetDigitalOutputMessage *msg =
        new RobotinoSensorInterface::SetDigitalOutputMessage(output, false);
    rsens_if_->msgq_enqueue(msg);
  }
  return;
}
