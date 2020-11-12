
/***************************************************************************
 *  skiller_motor_state_thread.cpp - Indicate skiller & motor state through LED
 *
 *  Created: Fri Jun 14 15:07:42 2019
 *  Copyright  2019  Morian Sonnet
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

#include "skiller_motor_state_thread.h"

#define CFG_PREFIX std::string("/skiller_motor_state")

#include <core/threading/mutex_locker.h>
#include <interfaces/MotorInterface.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/SkillerInterface.h>

using namespace fawkes;

/** @class SkillerMotorStateThread "act_thread.h"
 * Set LED according to motor state.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerMotorStateThread::SkillerMotorStateThread()
: Thread("SkillerMotorStateThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("skiller_motor_state"),
  final_time_(0, 0),
  failed_time_(0, 0),
  timeout_wait_condition_(&timeout_wait_mutex_),
  motor_if_changed_flag_(false),
  skiller_if_changed_flag_(false)
{
}

void
SkillerMotorStateThread::init()
{
	// first rest all thos config values
	cfg_skiller_ifid_       = config->get_string(CFG_PREFIX + "/skiller-interface-id");
	cfg_rsens_ifid_         = config->get_string(CFG_PREFIX + "/sensor-interface-id");
	cfg_motor_ifid_         = config->get_string(CFG_PREFIX + "/motor-interface-id");
	cfg_digital_out_green_  = config->get_uint(CFG_PREFIX + "/digital-out-green");
	cfg_digital_out_red_    = config->get_uint(CFG_PREFIX + "/digital-out-red");
	cfg_digital_out_yellow_ = config->get_uint(CFG_PREFIX + "/digital-out-yellow");
	cfg_digital_out_motor_  = config->get_uint(CFG_PREFIX + "/digital-out-motor");
	cfg_timeout_            = fawkes::Time(config->get_float(CFG_PREFIX + "/timeout"));

	// now open the interfaces which we need definetly, thus skiller and
	// robotinosensor
	skiller_if_ = blackboard->open_for_reading<SkillerInterface>(cfg_skiller_ifid_.c_str());
	rsens_if_   = blackboard->open_for_reading<RobotinoSensorInterface>(cfg_rsens_ifid_.c_str());

	// the skiller interface must also be listened to
	bbil_add_data_interface(skiller_if_);

	// finally disable all the outputs
	disable(cfg_digital_out_red_);
	disable(cfg_digital_out_green_);
	disable(cfg_digital_out_yellow_);

	// if the motor state output is also wished for, open the interface,
	// add it to the listener and disable the output
	if (cfg_digital_out_motor_) {
		motor_if_ = blackboard->open_for_reading<MotorInterface>(cfg_motor_ifid_.c_str());
		bbil_add_data_interface(motor_if_);
		disable(cfg_digital_out_motor_);
	}

	// finally register this thread as a blackboard listener
	blackboard->register_listener(this);

	// activate coalesce wake up, as one loop does all the work to do right now
	this->set_coalesce_wakeups(true);
}

void
SkillerMotorStateThread::finalize()
{
	// important: first deregister, then close
	blackboard->unregister_listener(this);

	// now close all opened interfaces
	blackboard->close(skiller_if_);
	blackboard->close(rsens_if_);
	if (cfg_digital_out_motor_)
		blackboard->close(motor_if_);
}

void
SkillerMotorStateThread::loop()
{
	// mandatory to have the robotino sensor interface writer
	if (!rsens_if_->has_writer())
		return; // forget it
	rsens_if_->read();

	// handle change in the skiller interface if necessary
	// by setting the correct signal lamps
	if (skiller_if_changed_flag_) { // check first the easy things
		if (skiller_if_->has_writer()) {
			// first clear the flag
			// if any change in data happens after the inital data change (which
			// raised the flag) and after the flag was cleared, but before the read(),
			// the flag is raised again, even though no newer data is available. this
			// leads to one additional run of handle_change_skiller, which is ok
			skiller_if_changed_flag_ = false;
			skiller_if_->read();
			signal_skiller_change();
		}
	}

	// handle change in the motor interface if necessary
	// by setting the correct signal lamps
	if (cfg_digital_out_motor_) {
		if (motor_if_changed_flag_) {
			if (motor_if_->has_writer()) {}
			motor_if_changed_flag_ = false;
			motor_if_->read();
			signal_motor_change();
		}
	}

	// while no changes need to be handled and we still need to reset some LEDs
	while (!motor_if_changed_flag_ && !skiller_if_changed_flag_
	       && (!final_time_.is_zero() || !failed_time_.is_zero())) {
		unsigned int  digital_output_to_reset;
		fawkes::Time *timeout_at;
		// first decide which lamp should be reset first
		get_next_switchoff(timeout_at, digital_output_to_reset);
		// now wait until the right moment to disable the led
		// However, we need to react on changes in the skiller ot motor interface
		// Thus, use interruptable timeout
		if (interruptable_timeout(timeout_at))
			break; // break, as it was interrupted

		rsens_if_->read(); // always reread sensor interface after waking up

		// sweet, my alarm woke me up, now let's turn off the LED
		disable(digital_output_to_reset);
		*timeout_at = fawkes::Time(0, 0); // don't need to check this timeout anymore
	}

	// ok, either all output with timeout are turned off now
	// then let's sleep now until someone wake us up
	// *OR* the timeouts got interrupted by some data change we have to handle
	// let's take a super quick nap now, as another wake up is already pending
}

// set the correct lights corresponding to the skiller interface
void
SkillerMotorStateThread::signal_skiller_change()
{
	SkillerInterface::SkillStatusEnum status = skiller_if_->status();
	if (status == SkillerInterface::S_RUNNING) {
		enable(cfg_digital_out_yellow_);
	} else {
		disable(cfg_digital_out_yellow_);
	}
	if (status == SkillerInterface::S_FINAL) {
		final_time_ = fawkes::Time() + cfg_timeout_;
		enable(cfg_digital_out_green_);
	}
	if (status == SkillerInterface::S_FAILED) {
		failed_time_ = fawkes::Time() + cfg_timeout_;
		enable(cfg_digital_out_red_);
	}
}

// set the correct lights corresponding to the motor interface
void
SkillerMotorStateThread::signal_motor_change()
{
	if (motor_if_->motor_state() == MotorInterface::MOTOR_DISABLED) {
		enable(cfg_digital_out_motor_);
	} else {
		disable(cfg_digital_out_motor_);
	}
}

void
SkillerMotorStateThread::enable(unsigned int output)
{
	if (!rsens_if_->is_digital_out(output - 1)) {
		RobotinoSensorInterface::SetDigitalOutputMessage *msg =
		  new RobotinoSensorInterface::SetDigitalOutputMessage(output, true);
		rsens_if_->msgq_enqueue(msg);
	}
	return;
}

void
SkillerMotorStateThread::disable(unsigned int output)
{
	if (rsens_if_->is_digital_out(output - 1)) {
		RobotinoSensorInterface::SetDigitalOutputMessage *msg =
		  new RobotinoSensorInterface::SetDigitalOutputMessage(output, false);
		rsens_if_->msgq_enqueue(msg);
	}
	return;
}

// function to decide which lamp should be reset first
// @param wait_until, this is the pointer to the time at which the lamp should
// be reset
// @oaram digital_output_to_reset the corresponding signal lamp output
void
SkillerMotorStateThread::get_next_switchoff(fawkes::Time *&wait_until,
                                            unsigned int & digital_output_to_reset)
{
	// first determine which LED needs to be reset earliest
	wait_until              = &failed_time_;
	digital_output_to_reset = 0;
	if (!final_time_.is_zero()) { // still need to reset final led
		wait_until              = &final_time_;
		digital_output_to_reset = cfg_digital_out_green_;
	}
	if (!failed_time_.is_zero()) { // still need to reset failed led
		if (*wait_until >= failed_time_) {
			wait_until              = &failed_time_;
			digital_output_to_reset = cfg_digital_out_red_;
		}
	}
}

// @return True if the timeout was interrupted
bool
SkillerMotorStateThread::interruptable_timeout(fawkes::Time *wait_until)
{
	// transform fawkes::Time to format for timer
	long int wait_until_sec = wait_until->get_sec(), wait_until_nsec = wait_until->get_nsec();

	fawkes::MutexLocker scoped_timeout_mutex(timeout_wait_mutex_);
	// use this mutex to make sure that the interface
	// flags cannot be changed between checking them a
	// last time and starting the interrupt
	// otherwise, the following calling series could happen:
	// we check that the flags are both off-> Ok, let's continue bed preparation
	// The blackboard changes, hence the flags get set and the
	// timeout_wait_condition is woken up The timeout is not running yet, hence
	// the wakeup doesn't have effect Now we start the wait_conditon, as the wake
	// up call already happened, no one is bothering us
	// => We don't react fast enough on skiller/motor changes!
	//
	// With mutex: the flags setting as well as the wakeup call is blocked
	// until the wait_condition started, thus the wakeup call will definetely
	// interrupt our sleep
	// => We react on changes of the skiller/motor interface on time.
	if (motor_if_changed_flag_ || skiller_if_changed_flag_)
		return true;

	// see WaitCondition::abstimed_wait()
	return timeout_wait_condition_.abstimed_wait(wait_until_sec, wait_until_nsec);
}

void
SkillerMotorStateThread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	if (*interface == *skiller_if_) {
		fawkes::MutexLocker scoped_timeout_mutex(timeout_wait_mutex_);
		skiller_if_changed_flag_ = true;
	} else { // must be motor interface then
		fawkes::MutexLocker scoped_timeout_mutex(timeout_wait_mutex_);
		motor_if_changed_flag_ = true;
	}
	timeout_wait_condition_.wake_all();
	wakeup();
}
