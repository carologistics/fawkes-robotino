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

#include "omnivision_activator_thread.h"

#include <cmath>

using namespace fawkes;

/** @class OmnivisionActivatorThread "omnivision_activator_thread.h"
 * Activates the Omnivision
 * @author Johannes Rothe
 */

/** Constructor. */
OmnivisionActivatorThread::OmnivisionActivatorThread()
: Thread("OmnivisionActivatorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void
OmnivisionActivatorThread::init()
{
	switch_if_ = blackboard->open_for_reading<SwitchInterface>("omnivisionSwitch");
	SwitchInterface::EnableSwitchMessage *msg = new SwitchInterface::EnableSwitchMessage();
	switch_if_->msgq_enqueue(msg);
	logger->log_info(name(), "Activating Omnivision");
}

void
OmnivisionActivatorThread::finalize()
{
	SwitchInterface::DisableSwitchMessage *msg = new SwitchInterface::DisableSwitchMessage();
	switch_if_->msgq_enqueue(msg);

	blackboard->close(switch_if_);
}

void
OmnivisionActivatorThread::loop()
{
}
