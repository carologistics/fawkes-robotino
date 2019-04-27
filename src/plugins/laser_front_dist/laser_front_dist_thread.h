
/***************************************************************************
 *  laser_front_dist_thread.h - laser_front_dist
 *
 *  Plugin created: Thu Jun 23 22:35:32 2016

 *  Copyright  2016  Frederik Zwilling
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

#ifndef __PLUGINS_LASER_FRONT_DISTTHREAD_H_
#define __PLUGINS_LASER_FRONT_DISTTHREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>

#include <string>

namespace fawkes {
class Laser360Interface;
class Position3DInterface;
} // namespace fawkes

class LaserFrontDistThread : public fawkes::Thread,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ClockAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::TransformAspect
{
public:
	LaserFrontDistThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	// Define class member variables here
	fawkes::Laser360Interface *  if_laser_;
	fawkes::Position3DInterface *if_result_;
	std::string                  frame_;
	int                          beams_used_;
	std::string                  target_frame_;
};

#endif
