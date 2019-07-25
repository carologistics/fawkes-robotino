
/***************************************************************************
 *  gazsim_sensing_thread.cpp - Plugin used to simulate sensing actions
 *
 *  Created: Mon Jul 15 18:54:42 2019
 *  Copyright  2019 Daniel Habering
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

#include "gazsim_sensing_thread.h"

#include <cmath>
#include <cstdarg>
#include <unistd.h>

using namespace fawkes;
using namespace gazebo;

/** @class GazsimSensingThread "gazsim_sensing_thread.cpp"
 * Thread simulates sensing actions in Gazebo
 *
 * @author Daniel Habering
 */

/** Constructor. */

GazsimSensingThread::GazsimSensingThread()
    : Thread("GazsimSensingThread", Thread::OPMODE_WAITFORWAKEUP),
      BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
      TransformAspect(TransformAspect::DEFER_PUBLISHER) {
  set_name("GazsimSensingThread()");
}

void GazsimSensingThread::init() {
  logger->log_debug(name(),
                    "Initializing Simulation of Sensing Actions Plugin");

  std::string sens_if_name = config->get_string("/gazsim/sensing/if-name");

  gazebo_sensing_pub_ = gazebonode->Advertise<msgs::Request>(
      config->get_string("/gazsim/topics/gripper-sensing-pub"));
  gazebo_sensing_feedback_sub_ = gazebonode->Subscribe(
      config->get_string("/gazsim/topics/gripper-sensing-feedback-sub"),
      &GazsimSensingThread::on_response_msg, this);
  // setup gripper if with default values
  sens_if_ = blackboard->open_for_writing<GazsimSensingInterface>(
 	   sens_if_name.c_str());
  sens_if_->set_busy(false);
  sens_if_->set_ret("");
  sens_if_->set_last_sensed("null");
  sens_if_->write();

}

void GazsimSensingThread::finalize() {
  blackboard->close(sens_if_);
}

void GazsimSensingThread::loop() {
  if (sens_if_->msgq_first_is<GazsimSensingInterface::GripperCalibratedMessage>()) {
    sens_if_->set_msgid(sens_if_->msgq_first<GazsimSensingInterface::GripperCalibratedMessage>()->id());
    send_gazebo_sensing_request("gripper_calibrated");
    
    sens_if_->msgq_pop();
  }
  if (sens_if_->msgq_first_is<GazsimSensingInterface::RealsenseActivatedMessage>()) {
    sens_if_->set_msgid(sens_if_->msgq_first<GazsimSensingInterface::RealsenseActivatedMessage>()->id());
    send_gazebo_sensing_request("realsense_activated");
    
    sens_if_->msgq_pop();
  }
  if (sens_if_->msgq_first_is<GazsimSensingInterface::ActivateRealsenseMessage>()) {
    sens_if_->set_msgid(sens_if_->msgq_first<GazsimSensingInterface::ActivateRealsenseMessage>()->id());
    send_gazebo_sensing_request("activate_realsense");

    sens_if_->msgq_pop();
  }
  if (sens_if_->msgq_first_is<GazsimSensingInterface::HoldingMessage>()) {
    sens_if_->set_msgid(sens_if_->msgq_first<GazsimSensingInterface::HoldingMessage>()->id());
    send_gazebo_sensing_request("gripper_holding");
    sens_if_->msgq_pop();
  }
  if (sens_if_->msgq_first_is<GazsimSensingInterface::WorkpieceCheckMessage>()) {
    GazsimSensingInterface::WorkpieceCheckMessage* msg = sens_if_->msgq_first<GazsimSensingInterface::WorkpieceCheckMessage>();
    sens_if_->set_msgid(msg->id());
    send_gazebo_sensing_request("workpiece",msg->mps());
    sens_if_->msgq_pop();
  }
}

void GazsimSensingThread::send_gazebo_sensing_request(std::string request, std::string data, double dbl_data) {
  msgs::Request msg;
  msg.set_request(request);
  msg.set_id(1);
  msg.set_data(data);
  msg.set_dbl_data(dbl_data);
  gazebo_sensing_pub_->Publish(msg);
  sens_if_->set_busy(true);
  sens_if_->set_ret("");
  sens_if_->set_last_sensed("");
  sens_if_->write();
}

void GazsimSensingThread::on_response_msg(ConstResponsePtr &msg) {
  sens_if_->set_ret(msg->response().c_str());
  sens_if_->set_busy(false);
  sens_if_->set_last_sensed(msg->request().c_str());
  sens_if_->write();
}

