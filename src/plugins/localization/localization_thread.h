/***************************************************************************
 *  localization_thread.h - empty example
 *
 *  Created: Do 31. Mai 00:19:28 CEST 2012
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

#ifndef __PLUGINS_LOCALIZATION_THREAD_H_
#define __PLUGINS_LOCALIZATION_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>

namespace fawkes {
class Position3DInterface;
class Laser360Interface;
}

class LocalizationThread: public fawkes::Thread,
public fawkes::BlockedTimingAspect,
public fawkes::LoggingAspect,
public fawkes::ConfigurableAspect,
public fawkes::BlackBoardAspect
{

public:
	LocalizationThread();

	virtual void init();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void run() {
		Thread::run();
	}

private:
	fawkes::Laser360Interface *laser_if_;
	fawkes::Position3DInterface *pos3d_if_;

};

#endif
