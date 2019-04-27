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

#include "robotino_wm_tester_thread.h"

#include <interfaces/RobotinoWorldModelInterface.h>

#include <cmath>
#include <map>
#include <stdlib.h>

using namespace fawkes;

/** @class RobotinoWorldModelThread "robotino_worldmodel_thread.h"
 * Testing if the worldmodel works
 * @author Daniel Ewert
 */

/** Constructor. */
RobotinoWMTesterThread::RobotinoWMTesterThread()
: Thread("RobotinoWorldModelThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void
RobotinoWMTesterThread::init()
{
	wm_if_      = blackboard->open_for_writing<RobotinoWorldModelInterface>("Model fll");
	wm_ext1_if_ = blackboard->open_for_writing<RobotinoWorldModelInterface>("Model fll ext1");
	wm_ext2_if_ = blackboard->open_for_writing<RobotinoWorldModelInterface>("Model fll ext2");
	time_       = clock->now();
	state_      = 0;
	type_       = 0;
}

bool
RobotinoWMTesterThread::prepare_finalize_user()
{
	return true;
}

void
RobotinoWMTesterThread::finalize()
{
	blackboard->close(wm_if_);
	blackboard->close(wm_ext1_if_);
	blackboard->close(wm_ext2_if_);
}

void
RobotinoWMTesterThread::loop()
{
	if (clock->elapsed(&time_) > 5) {
		apply_random_change();
		time_ = clock->now();
	}
	wm_if_->write();
	wm_ext1_if_->write();
	wm_ext2_if_->write();
}

void
RobotinoWMTesterThread::apply_random_change()
{
	int machine       = rand() % 13;
	int interface     = rand() % 3;
	int state_or_type = rand() % 2;
	if (state_or_type == 0) {
		RobotinoWorldModelInterface::machine_state_t state;
		switch (state_) {
		case 0: state = RobotinoWorldModelInterface::EMPTY; break;
		case 1: state = RobotinoWorldModelInterface::S0_ONLY; break;
		case 2: state = RobotinoWorldModelInterface::S1_ONLY; break;
		case 3:
		default: state = RobotinoWorldModelInterface::S1_S2; break;
		}

		switch (interface) {
		case 0: wm_if_->set_machine_states(machine, state); break;
		case 1: wm_ext1_if_->set_machine_states(machine, state); break;
		case 2: wm_ext2_if_->set_machine_states(machine, state); break;
		}
		state_++;
		state_ %= 4;
	} else {
		RobotinoWorldModelInterface::machine_type_t type;
		switch (type_) {
		case 0: type = RobotinoWorldModelInterface::TEST; break;
		case 1: type = RobotinoWorldModelInterface::M1_2; break;
		case 2: type = RobotinoWorldModelInterface::M1; break;
		case 3:
		default: type = RobotinoWorldModelInterface::M2; break;
		}
		switch (interface) {
		case 0: wm_if_->set_machine_types(machine, type); break;
		case 1: wm_ext1_if_->set_machine_types(machine, type); break;
		case 2: wm_ext2_if_->set_machine_types(machine, type); break;
		}
		type_++;
		type_ %= 4;
	}
}
