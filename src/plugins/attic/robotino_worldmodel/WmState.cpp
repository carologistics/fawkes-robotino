/*
 * WmState.cpp
 *
 *  Created on: 17.06.2012
 *      Author: daniel
 */

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

#include "WmState.h"

#include <algorithm>

using namespace fawkes;

WmState::WmState()
{
}

WmState::~WmState()
{
	// TODO Auto-generated destructor stub
}

void
WmState::set_logger(fawkes::Logger *logger)
{
	logger_ = logger;
}

/**
 * Copies the values of the given interface.
 */
void
WmState::update_worldmodel(RobotinoWorldModelInterface *wm_if)
{
	for (unsigned int i = 0; i < wm_if->maxlenof_machine_states(); ++i) {
		machine_states_[i] = wm_if->machine_states(i);
	}
	for (unsigned int i = 0; i < wm_if->maxlenof_machine_types(); ++i) {
		machine_types_[i] = wm_if->machine_types(i);
	}
	express_machine = wm_if->express_machine();
}

/**
 * Returns the differences in states between the given Interface and the
 * internal data
 */
std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t>
WmState::get_changed_states(RobotinoWorldModelInterface *wm_if)
{
	std::map<uint32_t, RobotinoWorldModelInterface::machine_state_t> changes;
	for (uint32_t i = 0; i < wm_if->maxlenof_machine_states(); ++i) {
		if (wm_if->machine_states(i) != machine_states_[i]) {
			logger_->log_debug("WmState",
			                   "State of %i changed from %i to %i",
			                   i,
			                   machine_states_[i],
			                   wm_if->machine_states(i));
			changes[i] = wm_if->machine_states(i);
		}
	}
	return changes;
}

/**
 * Returns the differences in states between the given Interface and the
 * internal data
 */
std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t>
WmState::get_changed_types(RobotinoWorldModelInterface *wm_if)
{
	std::map<uint32_t, RobotinoWorldModelInterface::machine_type_t> changes;
	for (uint32_t i = 0; i < wm_if->maxlenof_machine_types(); ++i) {
		if (wm_if->machine_types(i) != machine_types_[i]) {
			changes[i] = wm_if->machine_types(i);
			logger_->log_debug("WmState",
			                   "Type of %i changed from %i to %i",
			                   i,
			                   machine_types_[i],
			                   wm_if->machine_types(i));
		}
	}

	return changes;
}
