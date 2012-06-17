/*
 * WmState.h
 *
 *  Created on: 17.06.2012
 *      Author: daniel
 */

#ifndef WMSTATE_H_
#define WMSTATE_H_

#include <interfaces/RobotinoWorldModelInterface.h>
#include <map>

class WmState
{
public:
	WmState();
	virtual ~WmState();

	void update_worldmodel(fawkes::RobotinoWorldModelInterface* wm_if);
	std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_state_t> get_changed_states(
			fawkes::RobotinoWorldModelInterface *wm_if);
	std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_type_t> get_changed_types(
			fawkes::RobotinoWorldModelInterface *wm_if);

private:
	fawkes::RobotinoWorldModelInterface::machine_state_t* machine_states_;
	fawkes::RobotinoWorldModelInterface::machine_type_t* machine_types_;
	u_int32_t express_machine;

};

#endif /* WMSTATE_H_ */
