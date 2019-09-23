
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
#include <interfaces/MotorInterface.h>

#include <clipsmm.h>

using namespace fawkes;

/** @class ClipsMotorSwitchThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsMotorSwitchThread::ClipsMotorSwitchThread()
: Thread("ClipsMotorSwitchThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("motor-switch"),
  CLIPSFeatureAspect(this)
{
}

/** Destructor. */
ClipsMotorSwitchThread::~ClipsMotorSwitchThread()
{
	envs_.clear();
}

void
ClipsMotorSwitchThread::init()
{
	cfg_iface_id_ = config->get_string("/clips-motor-switch/interface-id");
	motor_if_     = blackboard->open_for_reading<MotorInterface>(cfg_iface_id_.c_str());
}

void
ClipsMotorSwitchThread::finalize()
{
	blackboard->close(motor_if_);
}

void
ClipsMotorSwitchThread::clips_context_init(const std::string &          env_name,
                                           LockPtr<CLIPS::Environment> &clips)
{
	envs_[env_name] = clips;

	clips->add_function(
	  "motor-enable",
	  sigc::slot<void>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMotorSwitchThread::clips_motor_enable), env_name)));
	clips->add_function(
	  "motor-disable",
	  sigc::slot<void>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMotorSwitchThread::clips_motor_disable), env_name)));
}

void
ClipsMotorSwitchThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
}

void
ClipsMotorSwitchThread::loop()
{
}

void
ClipsMotorSwitchThread::clips_motor_enable(std::string env_name)
{
	try {
		MotorInterface::SetMotorStateMessage *msg =
		  new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_ENABLED);
		motor_if_->msgq_enqueue(msg);
	} catch (Exception &e) {
		logger->log_warn(name(), "Cannot enable motor");
	}
}

void
ClipsMotorSwitchThread::clips_motor_disable(std::string env_name)
{
	try {
		MotorInterface::SetMotorStateMessage *msg =
		  new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_DISABLED);
		motor_if_->msgq_enqueue(msg);
	} catch (Exception &e) {
		logger->log_warn(name(), "Cannot disable motor");
	}
}
