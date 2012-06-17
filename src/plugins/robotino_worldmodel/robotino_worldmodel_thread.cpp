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

#include "robotino_worldmodel_thread.h"

#include <interfaces/RobotinoWorldModelInterface.h>

#include <cmath>
#include <map>

using namespace fawkes;

/** @class RobotinoWorldModelThread "robotino_worldmodel_thread.h"
 * Testing if the worldmodel works
 * @author Daniel Ewert
 */

/** Constructor. */
RobotinoWorldModelThread::RobotinoWorldModelThread()
		: Thread("RobotinoWorldModelThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void RobotinoWorldModelThread::init()
{
	logger->log_info(name(), "Plugin Template starts up");
	wm_if_ = blackboard->open_for_reading<RobotinoWorldModelInterface>(
			"Model fll");
	wm_ext1_if_ = blackboard->open_for_reading<RobotinoWorldModelInterface>(
			"Model fll ext1");
	wm_ext2_if_ = blackboard->open_for_reading<RobotinoWorldModelInterface>(
			"Model fll ext2");
	wm_merged_if_ = blackboard->open_for_writing<RobotinoWorldModelInterface>(
			"Model fll changed");
	wm_if_data_.update_worldmodel(wm_if_);
	wm_ext1_if_data_.update_worldmodel(wm_ext1_if_);
	wm_ext2_if_data_.update_worldmodel(wm_ext2_if_);

}

bool RobotinoWorldModelThread::prepare_finalize_user()
{
	return true;
}

void RobotinoWorldModelThread::finalize()
{
	blackboard->close(wm_if_);
	blackboard->close(wm_ext1_if_);
	blackboard->close(wm_ext2_if_);
	blackboard->close(wm_merged_if_);
}

void RobotinoWorldModelThread::write_state_changes(
		std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states)
{
	for (std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t>::iterator it =
			changed_machine_states.begin(); it != changed_machine_states.end();
			++it)
	{
		wm_merged_if_->set_machine_states(it->first, it->second);
	}

}

void RobotinoWorldModelThread::write_type_changes(
		std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types)
{
	for (std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t>::iterator it =
			changed_machine_types.begin(); it != changed_machine_types.end();
			++it)
	{
		wm_merged_if_->set_machine_types(it->first, it->second);
	}

}

void RobotinoWorldModelThread::publish_changes()
{
	wm_merged_if_->write();
	wm_if_data_.update_worldmodel(wm_if_);
	wm_ext1_if_data_.update_worldmodel(wm_ext1_if_);
	wm_ext2_if_data_.update_worldmodel(wm_ext2_if_);

}

void RobotinoWorldModelThread::loop()
{
	/*
	 * The changes of all all Interfaces are merged into each other. Shoild work since
	 * different robots are not expected to perceive different changes at the same machine.
	 * It's ugly anyways..
	 */
	std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states =
			wm_if_data_.get_changed_states(wm_if_);
	std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states_ext1 =
			wm_ext1_if_data_.get_changed_states(wm_ext1_if_);
	std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states_ext2 =
			wm_ext2_if_data_.get_changed_states(wm_ext2_if_);
	changed_machine_states.insert(changed_machine_states_ext1.begin(),
			changed_machine_states_ext1.end());
	changed_machine_states.insert(changed_machine_states_ext2.begin(),
			changed_machine_states_ext2.end());
	std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types =
			wm_if_data_.get_changed_types(wm_if_);
	std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types_ext1 =
			wm_ext1_if_data_.get_changed_types(wm_ext1_if_);
	std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types_ext2 =
			wm_ext2_if_data_.get_changed_types(wm_ext2_if_);
	changed_machine_types.insert(changed_machine_types_ext1.begin(),
			changed_machine_types_ext1.end());
	changed_machine_types.insert(changed_machine_types_ext2.begin(),
			changed_machine_types_ext2.end());
	if (!(changed_machine_states.empty() && changed_machine_types.empty()))
	{
		if (!changed_machine_states.empty())
		{
			write_state_changes(changed_machine_states);
		}
		if (!changed_machine_types.empty())
		{
			write_type_changes(changed_machine_types);
		}
		publish_changes();
	}

}

