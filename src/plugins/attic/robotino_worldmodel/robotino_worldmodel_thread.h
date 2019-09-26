/***************************************************************************
 *  robotino_worldmodel_thread.h - empty example
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

#ifndef __PLUGINS_ROBOTINO_WORLDMODEL_THREAD_H_
#define __PLUGINS_ROBOTINO_WORLDMODEL_THREAD_H_

#include "WmState.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <interfaces/RobotinoWorldModelInterface.h>

#include <map>
#include <string>
#include <vector>

class RobotinoWorldModelThread : public fawkes::Thread,
                                 public fawkes::BlockedTimingAspect,
                                 public fawkes::LoggingAspect,
                                 public fawkes::ConfigurableAspect,
                                 public fawkes::BlackBoardAspect
{
public:
	RobotinoWorldModelThread();

	virtual void init();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void publish_changes();
	void write_state_changes(fawkes::RobotinoWorldModelInterface *wm_if,
	                         std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_state_t>
	                           changed_machine_states);
	void write_type_changes(
	  fawkes::RobotinoWorldModelInterface *                                   wm_if,
	  std::map<uint32_t, fawkes::RobotinoWorldModelInterface::machine_type_t> changed_machine_types);
	void merge_worldmodels(fawkes::RobotinoWorldModelInterface *wm_sink_if,
	                       fawkes::RobotinoWorldModelInterface *wm_source_if);
	void wipe_worldmodel(fawkes::RobotinoWorldModelInterface *wm_if);

private:
	fawkes::RobotinoWorldModelInterface *wm_if_;
	fawkes::RobotinoWorldModelInterface *wm_ext_if_;
	fawkes::RobotinoWorldModelInterface *wm_merged_if_;
	fawkes::RobotinoWorldModelInterface *wm_changed_if_;

	WmState wm_if_data_;
	WmState wm_ext_if_data_;
};

#endif
