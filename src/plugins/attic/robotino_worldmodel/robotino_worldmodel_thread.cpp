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

#define CFG_PREFIX "/plugins/robotinoworldmodel/"

using namespace fawkes;

/** @class RobotinoWorldModelThread "robotino_worldmodel_thread.h"
 * Testing if the worldmodel works
 * @author Daniel Ewert
 */

/** Constructor. */
RobotinoWorldModelThread::RobotinoWorldModelThread()
: Thread("RobotinoWorldModelThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void
RobotinoWorldModelThread::init()
{
	logger->log_info(name(), "Plugin Template starts up");
	wm_if_data_.set_logger(logger);
	wm_ext_if_data_.set_logger(logger);
	wm_if_ = blackboard->open_for_reading<RobotinoWorldModelInterface>(
	  config->get_string(CFG_PREFIX "local_model").c_str());
	wm_ext_if_ = blackboard->open_for_reading<RobotinoWorldModelInterface>(
	  config->get_string(CFG_PREFIX "ext_model").c_str());
	wm_changed_if_ =
	  blackboard->open_for_writing<RobotinoWorldModelInterface>("Model fll changes_only");
	wm_merged_if_ = blackboard->open_for_writing<RobotinoWorldModelInterface>("Model fll merged");
	wm_if_data_.update_worldmodel(wm_if_);
	wm_ext_if_data_.update_worldmodel(wm_ext_if_);
}

bool
RobotinoWorldModelThread::prepare_finalize_user()
{
	return true;
}

void
RobotinoWorldModelThread::finalize()
{
	blackboard->close(wm_if_);
	blackboard->close(wm_ext_if_);
	blackboard->close(wm_merged_if_);
	blackboard->close(wm_changed_if_);
}

void
RobotinoWorldModelThread::write_state_changes(
  RobotinoWorldModelInterface *                                    wm_if,
  std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states)
{
	for (std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t>::iterator it =
	       changed_machine_states.begin();
	     it != changed_machine_states.end();
	     ++it) {
		wm_if->set_machine_states(it->first, it->second);
	}
}

void
RobotinoWorldModelThread::write_type_changes(
  RobotinoWorldModelInterface *                                   wm_if,
  std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types)
{
	for (std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t>::iterator it =
	       changed_machine_types.begin();
	     it != changed_machine_types.end();
	     ++it) {
		wm_if->set_machine_types(it->first, it->second);
	}
}

void
RobotinoWorldModelThread::publish_changes()
{
	wm_changed_if_->write();
	wm_merged_if_->write();
	wm_if_data_.update_worldmodel(wm_if_);
	wm_ext_if_data_.update_worldmodel(wm_ext_if_);
}

void
RobotinoWorldModelThread::loop()
{
	/*
   * The changes of all all Interfaces are merged into each other. Should work
   * since different robots are not expected to perceive different changes at
   * the same machine. It's ugly anyways..
   */
	wm_if_->read();
	wm_ext_if_->read();
	std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states =
	  wm_if_data_.get_changed_states(wm_if_);
	std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changed_machine_states_ext =
	  wm_ext_if_data_.get_changed_states(wm_ext_if_);
	changed_machine_states.insert(changed_machine_states_ext.begin(),
	                              changed_machine_states_ext.end());
	std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types =
	  wm_if_data_.get_changed_types(wm_if_);
	std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changed_machine_types_ext =
	  wm_ext_if_data_.get_changed_types(wm_ext_if_);
	changed_machine_types.insert(changed_machine_types_ext.begin(), changed_machine_types_ext.end());
	if (!(changed_machine_states.empty() && changed_machine_types.empty())) {
		wipe_worldmodel(wm_changed_if_);
		if (!changed_machine_states.empty()) {
			write_state_changes(wm_changed_if_, changed_machine_states);
			write_state_changes(wm_merged_if_, changed_machine_states);
		}
		if (!changed_machine_types.empty()) {
			write_type_changes(wm_changed_if_, changed_machine_types);
			write_type_changes(wm_merged_if_, changed_machine_types);
		}
		publish_changes();
	}
}

void
RobotinoWorldModelThread::merge_worldmodels(fawkes::RobotinoWorldModelInterface *wm_sink_if,
                                            fawkes::RobotinoWorldModelInterface *wm_source_if)
{
	for (uint32_t i = 0; i < wm_source_if->maxlenof_machine_states(); ++i) {
		wm_sink_if->set_machine_states(i, wm_source_if->machine_states(i));
	}
	for (uint32_t i = 0; i < wm_source_if->maxlenof_machine_types(); ++i) {
		wm_sink_if->set_machine_types(i, wm_source_if->machine_types(i));
	}
	wm_sink_if->set_express_machine(wm_source_if->express_machine());
}

void
RobotinoWorldModelThread::wipe_worldmodel(RobotinoWorldModelInterface *wm_if)
{
	for (unsigned int i = 0; i < wm_if->maxlenof_machine_states(); ++i) {
		wm_if->set_machine_states(i, RobotinoWorldModelInterface::STATE_UNKNOWN);
		wm_if->set_machine_types(i, RobotinoWorldModelInterface::TYPE_UNKNOWN);
	}
}
