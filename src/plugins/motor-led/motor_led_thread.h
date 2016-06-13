
/***************************************************************************
 *  motor_led_thread.h - Motor LED thread
 *
 *  Created: Fri Apr 15 15:04:53 2016
 *  Copyright  2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_MOTOR_LED_MOTOR_LED_THREAD_H_
#define __PLUGINS_MOTOR_LED_MOTOR_LED_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

namespace fawkes {
	class MotorInterface;
	class RobotinoSensorInterface;
}

class MotorLedThread
: public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::BlockedTimingAspect,
	public fawkes::BlackBoardAspect
{
 public:
	MotorLedThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	std::string  cfg_motor_ifid_;
	std::string  cfg_rsens_ifid_;
	unsigned int cfg_digital_out_;

	fawkes::MotorInterface          *motor_if_;
	fawkes::RobotinoSensorInterface *rsens_if_;
};


#endif
