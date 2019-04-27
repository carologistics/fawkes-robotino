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

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <interfaces/RobotinoWorldModelInterface.h>

#include <map>
#include <string>
#include <vector>

class RobotinoWMTesterThread : public fawkes::Thread,
                               public fawkes::BlockedTimingAspect,
                               public fawkes::LoggingAspect,
                               public fawkes::ConfigurableAspect,
                               public fawkes::BlackBoardAspect,
                               public fawkes::ClockAspect
{
public:
	RobotinoWMTesterThread();

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
	void apply_random_change();

private:
	fawkes::RobotinoWorldModelInterface *wm_if_;
	fawkes::RobotinoWorldModelInterface *wm_ext1_if_;
	fawkes::RobotinoWorldModelInterface *wm_ext2_if_;

	fawkes::Time time_;
	int          state_;
	int          type_;
};

#endif
