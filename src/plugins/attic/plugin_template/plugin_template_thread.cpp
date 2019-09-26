/***************************************************************************
 *  plugin_template_thread.cpp - template thread
 *
 *  Created: Mi 23. Mai 17:44:14 CEST 2012
 *  Copyright  2012 Daniel Ewert
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

#include "plugin_template_thread.h"

#include <interfaces/MotorInterface.h>

#include <cmath>

#define FORWARD_SPEED 1.0f
#define SIDEWARD_SPEED 1.0f
#define ROTATIONAL_SPEED 1.0f

using namespace fawkes;

/** @class PluginTemplateThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped
 * course
 * @author Daniel Ewert
 */

/** Constructor. */
PluginTemplateThread::PluginTemplateThread()
: Thread("PluginTemplateThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void
PluginTemplateThread::init()
{
	logger->log_info(name(), "Plugin Template starts up");
	motor_if_ = blackboard->open_for_reading<MotorInterface>("Robotino");
}

bool
PluginTemplateThread::prepare_finalize_user()
{
	stop();
	return true;
}

void
PluginTemplateThread::finalize()
{
	blackboard->close(motor_if_);
}

void
PluginTemplateThread::send_transrot(float vx, float vy, float omega)
{
	MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(vx, vy, omega);
	motor_if_->msgq_enqueue(msg);
}

void
PluginTemplateThread::stop()
{
	send_transrot(0., 0., 0.);
}

void
PluginTemplateThread::loop()
{
	send_transrot(FORWARD_SPEED, SIDEWARD_SPEED, ROTATIONAL_SPEED);
	logger->log_info(name(), "Driving madly!!");
}
