
/***************************************************************************
 *  motor_led_thread.cpp - Motor LED thread
 *
 *  Created: Fri Apr 15 15:07:42 2016
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

#include "motor_led_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/RobotinoSensorInterface.h>

using namespace fawkes;

/** @class MotorLedThread "act_thread.h"
 * Set LED according to motor state.
 * @author Tim Niemueller
 */

/** Constructor. */
MotorLedThread::MotorLedThread()
	: Thread("MotorLedThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}


void
MotorLedThread::init()
{
	cfg_motor_ifid_    = config->get_string("/motor-led/motor-interface-id");
	cfg_rsens_ifid_    = config->get_string("/motor-led/sensor-interface-id");
	cfg_digital_out_   = config->get_uint("/motor-led/digital-out");

	motor_if_ = blackboard->open_for_reading<MotorInterface>(cfg_motor_ifid_.c_str());
	rsens_if_ = blackboard->open_for_reading<RobotinoSensorInterface>(cfg_rsens_ifid_.c_str());
}


void
MotorLedThread::finalize()
{
	blackboard->close(motor_if_);
	blackboard->close(rsens_if_);
}


void
MotorLedThread::loop()
{
	if (rsens_if_->has_writer()) {
		motor_if_->read();
		rsens_if_->read();

		// -1: to map port names to interface positions

		if (motor_if_->motor_state() == MotorInterface::MOTOR_DISABLED &&
		    ! rsens_if_->is_digital_out(cfg_digital_out_ - 1))
		{
			RobotinoSensorInterface::SetDigitalOutputMessage *msg =
				new RobotinoSensorInterface::SetDigitalOutputMessage(cfg_digital_out_, true);
			rsens_if_->msgq_enqueue(msg);
		} else if (motor_if_->motor_state() == MotorInterface::MOTOR_ENABLED &&
		           rsens_if_->is_digital_out(cfg_digital_out_ - 1))
		{
			RobotinoSensorInterface::SetDigitalOutputMessage *msg =
				new RobotinoSensorInterface::SetDigitalOutputMessage(cfg_digital_out_, false);
			rsens_if_->msgq_enqueue(msg);
		}
	}
}
