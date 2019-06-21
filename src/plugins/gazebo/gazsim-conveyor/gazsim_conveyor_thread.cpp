
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

#include <boost/lexical_cast.hpp>
#include <config/change_handler.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/ConveyorPoseInterface.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#include <cmath>
#include <cstdarg>
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
      BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
      fawkes::TransformAspect(fawkes::TransformAspect::BOTH, "conveyor_pose"),
      new_data_(false) {
  set_name("GazsimConveyorThread()");
  loopcount_ = 0;
}

void GazsimConveyorThread::init() {
  logger->log_debug(name(),
                    "Initializing Simulation of the Conveyor Vision Plugin");
  const std::string if_prefix =
      config->get_string("plugins/conveyor_pose/if/prefix") + "/";
  frame_name_ = config->get_string("/realsense/frame_id");
  conveyor_frame_id_ =
      config->get_string("plugins/conveyor_pose/conveyor_frame_id");

  cfg_if_prefix_ = config->get_string(CFG_PREFIX "/if/prefix");
  if (cfg_if_prefix_.back() != '/')
    cfg_if_prefix_.append("/");

  // setup ConveyorPoseInterface if with default values
  pos_if_ = blackboard->open_for_writing<ConveyorPoseInterface>(
      (cfg_if_prefix_ + "status").c_str());
  switch_if_ = blackboard->open_for_writing<SwitchInterface>(
      config->get_string("/gazsim/conveyor/switch-if-name").c_str());

  conveyor_vision_sub_ = gazebonode->Subscribe(
      "~/RobotinoSim/ConveyorVisionResult/",
      &GazsimConveyorThread::on_conveyor_vision_msg, this);
}

void GazsimConveyorThread::finalize() {
  blackboard->close(pos_if_);
  blackboard->close(switch_if_);
}

void GazsimConveyorThread::loop() {

  pos_if_->set_frame(frame_name_.c_str());
  if (pos_if_->msgq_first_is<ConveyorPoseInterface::StopICPMessage>()){
    pos_if_->msgq_pop();
  }
  if (new_data_ &&
      pos_if_->msgq_first_is<ConveyorPoseInterface::RunICPMessage>()) {
    new_data_ = false;

    ConveyorPoseInterface::RunICPMessage *msg =
        pos_if_->msgq_first<ConveyorPoseInterface::RunICPMessage>();
    // write to interface
    // swap the axis' because the cam_conveyor frame has the z-axis facing
    // foward
    double trans[] = {last_msg_.positions().x(), last_msg_.positions().y(),
                      last_msg_.positions().z()};
    if (strcmp(pos_if_->tostring_MPS_TARGET(msg->mps_target_to_set()),
               "SLIDE") == 0) {
      trans[1] -= shelf_offset_x;
    }
    double rot[] = {
        last_msg_.positions().ori_x(), last_msg_.positions().ori_y(),
        last_msg_.positions().ori_z(), last_msg_.positions().ori_w()};

    pos_if_->set_translation(trans);
    pos_if_->set_rotation(rot);
    pos_if_->set_euclidean_fitness(rand() % 100);
    pos_if_->set_msgid(msg->id());
    pos_if_->write();

    // publishe tf
    fawkes::tf::StampedTransform transform;

    transform.frame_id = "base_link";
    transform.child_frame_id = conveyor_frame_id_;
    transform.stamp = fawkes::Time();

    transform.setOrigin(fawkes::tf::Vector3(trans[0], trans[1], trans[2]));
    fawkes::tf::Quaternion q(rot[0], rot[1], rot[2], rot[3]);
    transform.setRotation(q);

    tf_publisher->send_transform(transform);
    pos_if_->msgq_pop();
  }

  loopcount_++;
}

void GazsimConveyorThread::on_conveyor_vision_msg(
    ConstConveyorVisionResultPtr &msg) {
  last_msg_.CopyFrom(*msg);
  new_data_ = true;
}
