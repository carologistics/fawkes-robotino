
/***************************************************************************
 *  clips-motor-switch-thread.cpp - Switch motor from CLIPS
 *
 *  Created: Thu Apr 25 12:31:51 2013
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "clips-motor-switch-thread.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/SwitchInterface.h>

#include <clipsmm.h>

using namespace fawkes;

/** @class ClipsMotorSwitchThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsMotorSwitchThread::ClipsMotorSwitchThread(std::string &env_name)
  : Thread("ClipsMotorSwitchThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSAspect(env_name.c_str(), /* create */ true, /* excl */ false)
{
}


/** Destructor. */
ClipsMotorSwitchThread::~ClipsMotorSwitchThread()
{
}


void
ClipsMotorSwitchThread::init()
{
  cfg_iface_id_ = config->get_string("/clips-motor-switch/interface-id");
  switch_if_ =
    blackboard->open_for_reading<SwitchInterface>(cfg_iface_id_.c_str());

  MutexLocker lock(clips.objmutex_ptr());

  clips->add_function("motor-enable", sigc::slot<void>(sigc::mem_fun(*this, &ClipsMotorSwitchThread::clips_motor_enable)));
  clips->add_function("motor-disable", sigc::slot<void>(sigc::mem_fun(*this, &ClipsMotorSwitchThread::clips_motor_enable)));

}


void
ClipsMotorSwitchThread::finalize()
{
  blackboard->close(switch_if_);
}


void
ClipsMotorSwitchThread::loop()
{
}


void
ClipsMotorSwitchThread::clips_motor_enable()
{
  SwitchInterface::EnableSwitchMessage *msg =
    new SwitchInterface::EnableSwitchMessage();
  switch_if_->msgq_enqueue(msg);
}

void
ClipsMotorSwitchThread::clips_motor_disable()
{
  SwitchInterface::DisableSwitchMessage *msg =
    new SwitchInterface::DisableSwitchMessage();
  switch_if_->msgq_enqueue(msg);
}
