/*
 * WmState.h
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

#ifndef WMSTATE_H_
#define WMSTATE_H_

#include <interfaces/RobotinoWorldModelInterface.h>
#include <logging/logger.h>

#include <map>

class WmState
{
public:
	WmState();
	void set_logger(fawkes::Logger *logger);
	virtual ~WmState();

	void update_worldmodel(fawkes::RobotinoWorldModelInterface *wm_if);
	std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_state_t>
	get_changed_states(fawkes::RobotinoWorldModelInterface *wm_if);
	std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_type_t>
	get_changed_types(fawkes::RobotinoWorldModelInterface *wm_if);

private:
	fawkes::RobotinoWorldModelInterface::machine_state_t machine_states_[13];
	fawkes::RobotinoWorldModelInterface::machine_type_t  machine_types_[13];
	u_int32_t                                            express_machine;
	fawkes::Logger *                                     logger_;
};

#endif /* WMSTATE_H_ */
