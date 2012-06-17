/*
 * WmState.h
 *
 *  Created on: 17.06.2012
 *      Author: daniel
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
	void set_logger(fawkes::Logger* logger);
	virtual ~WmState();

	void update_worldmodel(fawkes::RobotinoWorldModelInterface* wm_if);
	std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_state_t> get_changed_states(
			fawkes::RobotinoWorldModelInterface *wm_if);
	std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_type_t> get_changed_types(
			fawkes::RobotinoWorldModelInterface *wm_if);

private:
	fawkes::RobotinoWorldModelInterface::machine_state_t machine_states_[13];
	fawkes::RobotinoWorldModelInterface::machine_type_t machine_types_[13];
	u_int32_t express_machine;
	fawkes::Logger* logger_;

};

#endif /* WMSTATE_H_ */
