
/***************************************************************************
 *  ax12_gripper_thread.cpp - AX12A Gripper unit act thread
 *
 *  Created: Sun Mar 29 16:11:47 2015
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *                  2015  Nicolas Limpert
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

#include "ax12_gripper_thread.h"

#include <config/change_handler.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/scoped_rwlock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/AX12GripperInterface.h>
#include <interfaces/JointInterface.h>
#include <interfaces/LedInterface.h>
#include <utils/math/angle.h>

#include <boost/lexical_cast.hpp>
#include <cmath>
#include <cstdarg>
#include <unistd.h>

using namespace fawkes;

/** @class GripperAX12AThread "ax12_gripper_thread.h"
 * Gripper act thread for AX12A Gripper.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts with the controller of the AX12 Gripper.
 * @author Tim Niemueller
 */
/** Constructor.
 * @param gripper_cfg_prefix gripper plugin configuration prefix
 */

GripperAX12AThread::GripperAX12AThread(std::string &gripper_cfg_prefix)
: Thread("GripperAX12AThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
#ifdef HAVE_TF
  TransformAspect(TransformAspect::ONLY_PUBLISHER, std::string("GRIPPER AX12A").c_str()),
#endif
  BlackBoardInterfaceListener("GripperAX12AThread(%s)", gripper_cfg_prefix.c_str()),
  ConfigurationChangeHandler(gripper_cfg_prefix.c_str())
{
	set_name("GripperAX12AThread()");

	__gripper_cfg_prefix = gripper_cfg_prefix;
	//  __ptu_name           = ptu_name;

	// __ax12a = NULL;
}

void
GripperAX12AThread::init()
{
	// __last_left = __last_right = 0.f;
	float init_left_velocity  = 0.f;
	float init_right_velocity = 0.f;

	config->add_change_handler(this);

	load_config();

	load_left_pending  = false;
	load_right_pending = false;

	bool left_servo_found = false, right_servo_found = false, z_servo_found = false;

	__servo_if_right = blackboard->open_for_reading<DynamixelServoInterface>(
	  (__cfg_driver_prefix + "/" + __cfg_right_servo_id).c_str());
	__servo_if_left = blackboard->open_for_reading<DynamixelServoInterface>(
	  (__cfg_driver_prefix + "/" + __cfg_left_servo_id).c_str());

	__servo_if_left->read();
	__servo_if_right->read();

	right_servo_found = __servo_if_right->has_writer();
	left_servo_found  = __servo_if_left->has_writer();
	if (!(left_servo_found && right_servo_found)) {
		throw Exception("Left and/or right and/or z-servo not found: left: %i  "
		                "right: %i, z: %i",
		                left_servo_found,
		                right_servo_found,
		                z_servo_found);
	}

	joystick_if_ =
	  blackboard->open_for_reading<JoystickInterface>("Joystick", __cfg_ifid_joystick_.c_str());

	DynamixelServoInterface::SetSpeedMessage *vel_left =
	  new DynamixelServoInterface::SetSpeedMessage((unsigned int)(__cfg_max_speed * 0x3ff));
	DynamixelServoInterface::SetSpeedMessage *vel_right =
	  new DynamixelServoInterface::SetSpeedMessage((unsigned int)(__cfg_max_speed * 0x3ff));
	__servo_if_left->msgq_enqueue(vel_left);
	__servo_if_right->msgq_enqueue(vel_right);

	// Enable PreventAlarmShutdown on both servos
	DynamixelServoInterface::SetPreventAlarmShutdownMessage *prevent_left_msg =
	  new DynamixelServoInterface::SetPreventAlarmShutdownMessage();
	DynamixelServoInterface::SetPreventAlarmShutdownMessage *prevent_right_msg =
	  new DynamixelServoInterface::SetPreventAlarmShutdownMessage();
	prevent_left_msg->set_enable_prevent_alarm_shutdown(false);
	prevent_right_msg->set_enable_prevent_alarm_shutdown(false);
	__servo_if_left->msgq_enqueue(prevent_left_msg);
	__servo_if_right->msgq_enqueue(prevent_right_msg);

	DynamixelServoInterface::SetTorqueLimitMessage *torque_left_msg =
	  new DynamixelServoInterface::SetTorqueLimitMessage();
	DynamixelServoInterface::SetTorqueLimitMessage *torque_right_msg =
	  new DynamixelServoInterface::SetTorqueLimitMessage();
	torque_left_msg->set_torque_limit(1023);
	torque_right_msg->set_torque_limit(1023);
	__servo_if_left->msgq_enqueue(torque_left_msg);
	__servo_if_right->msgq_enqueue(torque_right_msg);

	// Enable autorecovery for left and right servo
	DynamixelServoInterface::SetAutorecoverEnabledMessage *recover_left_msg =
	  new DynamixelServoInterface::SetAutorecoverEnabledMessage();
	DynamixelServoInterface::SetAutorecoverEnabledMessage *recover_right_msg =
	  new DynamixelServoInterface::SetAutorecoverEnabledMessage();
	recover_left_msg->set_autorecover_enabled(false);
	recover_right_msg->set_autorecover_enabled(false);
	__servo_if_left->msgq_enqueue(recover_left_msg);
	__servo_if_right->msgq_enqueue(recover_right_msg);

	// If you have more than one interface: catch exception and close them!
	std::string bbid = "Gripper AX12";
	__gripper_if     = blackboard->open_for_writing<AX12GripperInterface>(bbid.c_str());
	__gripper_if->set_calibrated(true);
	__gripper_if->set_min_left(__cfg_left_min);
	__gripper_if->set_max_left(__cfg_left_max);
	__gripper_if->set_min_right(__cfg_right_min);
	__gripper_if->set_max_right(__cfg_right_max);
	__gripper_if->set_left_margin(__cfg_left_margin);
	__gripper_if->set_right_margin(__cfg_right_margin);
	__gripper_if->set_max_left_velocity(0); //__ax12a->get_max_supported_speed(__cfg_left_servo_id));
	__gripper_if->set_max_right_velocity(
	  0); //__ax12a->get_max_supported_speed(__cfg_right_servo_id));
	__gripper_if->set_left_velocity(init_left_velocity);
	__gripper_if->set_right_velocity(init_right_velocity);
	__gripper_if->write();

	__led_if = blackboard->open_for_writing<LedInterface>(bbid.c_str());

	std::string leftid = __cfg_gripper_name + " left";
	__leftjoint_if     = blackboard->open_for_writing<JointInterface>(leftid.c_str());
	__leftjoint_if->set_position(__last_left);
	__leftjoint_if->set_velocity(init_left_velocity);
	__leftjoint_if->write();

	std::string rightid = __cfg_gripper_name + " right";
	__rightjoint_if     = blackboard->open_for_writing<JointInterface>(rightid.c_str());
	__rightjoint_if->set_position(__last_right);
	__rightjoint_if->set_velocity(init_right_velocity);
	__rightjoint_if->write();

	if (__cfg_goto_zero_start) {
		goto_gripper(__cfg_left_start, __cfg_right_start);
	}

	bbil_add_message_interface(__gripper_if);
	bbil_add_message_interface(__leftjoint_if);
	bbil_add_message_interface(__rightjoint_if);
	blackboard->register_listener(this);

	cur_torque_ = 1.0;

	// initialize motion_start_timestamp
	motion_start_timestamp_ = fawkes::Time();
	slap_left_pending_      = false;
	slap_right_pending_     = false;

#ifdef USE_TIMETRACKER
	__tt.reset(new TimeTracker());
	__tt_count        = 0;
	__ttc_read_sensor = __tt->add_class("Read Sensor");
#endif
}

void
GripperAX12AThread::finalize()
{
	// make sure that no servo continues movement
	stop_motion();
	blackboard->unregister_listener(this);
	blackboard->close(__gripper_if);
	blackboard->close(__led_if);
	blackboard->close(__leftjoint_if);
	blackboard->close(__rightjoint_if);
	blackboard->close(__servo_if_right);
	blackboard->close(__servo_if_left);
	config->rem_change_handler(this);
}

void
GripperAX12AThread::loop()
{
	// __gripper_if->set_final(__wt->is_final());

	if (!cfg_mutex_.try_lock()) {
		logger->log_info(name(), "Skipping loop(), mutex locked");
		return; // If I cant lock it its locked already (config is changing/reinit
		        // in progress)
	} else {
		// read data from interfaces
		__servo_if_left->read();
		__servo_if_right->read();

		fawkes::Time now(clock);

		// load is given in values from 0 - 1023 is ccw load, 1024 - 2047 is cw load
		// but we are only interested in overall load
		if (load_left_pending && (__servo_if_left->load() & 0x3ff) >= (__cfg_max_load * 0x3ff)) {
			DynamixelServoInterface::StopMessage *stop_message =
			  new DynamixelServoInterface::StopMessage();
			__servo_if_left->msgq_enqueue(stop_message);
			load_left_pending = false;
		}
		if (load_right_pending && (__servo_if_right->load() & 0x3ff) >= (__cfg_max_load * 0x3ff)) {
			DynamixelServoInterface::StopMessage *stop_message =
			  new DynamixelServoInterface::StopMessage();
			__servo_if_right->msgq_enqueue(stop_message);
			load_right_pending = false;
		}
		if (center_pending && __servo_if_left->is_final() && __servo_if_right->is_final()) {
			center_pending = false;
		}

		bool is_final = now > (motion_start_timestamp_ + 0.5) && __servo_if_left->is_final()
		                && __servo_if_right->is_final() && !z_alignment_pending;

		if (slap_left_pending_) {
			// the left servo is overriding the middle when it crosses the
			// angle values from less than 0 to greater than 0
			bool left_done = __servo_if_left->angle() >= 0.;
			is_final &= left_done;
			slap_left_pending_ = !left_done;
		}
		if (slap_right_pending_) {
			// the right servo is overriding the middle when it crosses the
			// angle values from greater than 0 to less than 0
			bool right_done = __servo_if_right->angle() <= 0.;
			is_final &= right_done;
			slap_right_pending_ = !right_done;
		}

		__gripper_if->set_final(is_final);

		// if the torque is disabled we have to track the position of the servos
		// and constantly send it as goal poses.
		// Otherwise resetting the torque_limit and trying to go to a new pose
		// leads to a full speed move of the servos.
		// This is probably a bug in the AX12 firmware.
		if (cur_torque_ < 1.0) {
			bool is_final = (__servo_if_left->speed() & 0x3FF) < 32;
			is_final &= (__servo_if_right->speed() & 0x3FF) < 32;
			is_final &= !z_alignment_pending;
			is_final &= now > (motion_start_timestamp_ + 0.5);
			__gripper_if->set_final(is_final);
			if (is_final == false) {
				goto_gripper(__servo_if_left->angle(), __servo_if_right->angle());
			}
		}

		while (!__gripper_if->msgq_empty()) {
			if (__gripper_if->msgq_first_is<AX12GripperInterface::CalibrateMessage>()) {
				// ignored

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::GotoMessage>()) {
				AX12GripperInterface::GotoMessage *msg = __gripper_if->msgq_first(msg);

				goto_gripper(msg->left(), msg->right());
				__gripper_if->set_msgid(msg->id());
				__gripper_if->set_final(false);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::TimedGotoMessage>()) {
				AX12GripperInterface::TimedGotoMessage *msg = __gripper_if->msgq_first(msg);

				goto_gripper_timed(msg->left(), msg->right(), msg->time_sec());
				__gripper_if->set_msgid(msg->id());
				__gripper_if->set_final(false);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::ParkMessage>()) {
				AX12GripperInterface::ParkMessage *msg = __gripper_if->msgq_first(msg);

				goto_gripper(0, 0);
				__gripper_if->set_msgid(msg->id());
				__gripper_if->set_final(false);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetEnabledMessage>()) {
				AX12GripperInterface::SetEnabledMessage *msg = __gripper_if->msgq_first(msg);

				set_enabled(msg->is_enabled());

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetVelocityMessage>()) {
				AX12GripperInterface::SetVelocityMessage *msg = __gripper_if->msgq_first(msg);

				if (msg->left_velocity() > __gripper_if->max_left_velocity()) {
					logger->log_warn(name(),
					                 "Desired left velocity %f too high, max is %f",
					                 msg->left_velocity(),
					                 __gripper_if->max_left_velocity());
				} else if (msg->right_velocity() > __gripper_if->max_right_velocity()) {
					logger->log_warn(name(),
					                 "Desired right velocity %f too high, max is %f",
					                 msg->right_velocity(),
					                 __gripper_if->max_right_velocity());
				} else {
					set_velocities(msg->left_velocity(), msg->right_velocity());
				}

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::OpenMessage>()) {
				AX12GripperInterface::OpenMessage *msg = __gripper_if->msgq_first(msg);

				set_torque(1.0);
				goto_gripper(__cfg_left_open_angle + msg->offset(), __cfg_right_open_angle + msg->offset());

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::ModifyOpeningAngleByMessage>()) {
				AX12GripperInterface::ModifyOpeningAngleByMessage *msg = __gripper_if->msgq_first(msg);
				const float angle_difference                           = msg->angle_difference();

				set_torque(1.0);
				goto_gripper(__servo_if_left->angle() - angle_difference,
				             __servo_if_right->angle() + angle_difference);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::CloseMessage>()) {
				AX12GripperInterface::CloseMessage *msg = __gripper_if->msgq_first(msg);

				goto_gripper(__cfg_left_close_angle, __cfg_right_close_angle);
				// set_move_load_pending(true);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::CloseLoadMessage>()) {
				AX12GripperInterface::CloseLoadMessage *msg = __gripper_if->msgq_first(msg);

				goto_gripper_load(__cfg_left_close_load_angle, __cfg_right_close_load_angle);
				// set_move_load_pending(true);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::Open_AngleMessage>()) {
				AX12GripperInterface::Open_AngleMessage *msg        = __gripper_if->msgq_first(msg);
				float                                    angle_part = msg->angle() / 2.;
				goto_gripper(-angle_part, angle_part);

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetMarginMessage>()) {
				AX12GripperInterface::SetMarginMessage *msg = __gripper_if->msgq_first(msg);

				set_margins(msg->left_margin(), msg->right_margin());
				__gripper_if->set_left_margin(msg->left_margin());
				__gripper_if->set_right_margin(msg->right_margin());

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::CenterMessage>()) {
				// The target_opening_angle has to be smaller than the opening_angle
				// because the servos tend to open a little when only the opening_angle
				// is used.
				float target_opening_angle_per_servo =
				  (get_opening_angle() - __cfg_center_angle_correction_amount) / 2.;

				goto_gripper(__servo_if_left->angle()
				               - (__servo_if_left->angle() + target_opening_angle_per_servo),
				             __servo_if_right->angle()
				               - (__servo_if_right->angle() - target_opening_angle_per_servo));
				center_pending = true;

				motion_start_timestamp_ = fawkes::Time(clock);

				__gripper_if->set_final(false);
			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetTorqueMessage>()) {
				AX12GripperInterface::SetTorqueMessage *msg = __gripper_if->msgq_first(msg);

				motion_start_timestamp_ = fawkes::Time(clock);
				set_torque(msg->torque());

			} else if (__gripper_if->msgq_first_is<AX12GripperInterface::SlapMessage>()) {
				AX12GripperInterface::SlapMessage *msg = __gripper_if->msgq_first(msg);

				if (msg->slapmode() == AX12GripperInterface::SlapMode::LEFT) {
					// open the gripper and disable torque on the left
					DynamixelServoInterface::SetTorqueLimitMessage *torque_left_msg =
					  new DynamixelServoInterface::SetTorqueLimitMessage();
					DynamixelServoInterface::SetTorqueLimitMessage *torque_right_msg =
					  new DynamixelServoInterface::SetTorqueLimitMessage();
					torque_left_msg->set_torque_limit(0);
					torque_right_msg->set_torque_limit(1023);
					__servo_if_left->msgq_enqueue(torque_left_msg);
					__servo_if_right->msgq_enqueue(torque_right_msg);

					goto_gripper(__cfg_left_open_angle, __cfg_right_open_angle);

					motion_start_timestamp_ = fawkes::Time(clock);
					slap_left_pending_      = true;

				} else if (msg->slapmode() == AX12GripperInterface::SlapMode::RIGHT) {
					DynamixelServoInterface::SetTorqueLimitMessage *torque_left_msg =
					  new DynamixelServoInterface::SetTorqueLimitMessage();
					DynamixelServoInterface::SetTorqueLimitMessage *torque_right_msg =
					  new DynamixelServoInterface::SetTorqueLimitMessage();
					torque_left_msg->set_torque_limit(1023);
					torque_right_msg->set_torque_limit(0);
					__servo_if_left->msgq_enqueue(torque_left_msg);
					__servo_if_right->msgq_enqueue(torque_right_msg);
					// open the gripper and disable torque on the right
					goto_gripper(__cfg_left_open_angle, __cfg_right_open_angle);

					motion_start_timestamp_ = fawkes::Time(clock);
					slap_right_pending_     = true;

				} else {
					logger->log_error(name(), "SlapMessage received but no side given.");
				}

			} else {
				logger->log_warn(name(), "Unknown message received");
			}

			__gripper_if->msgq_pop();
		}

		joystick_if_->read();

		if (joystick_if_->pressed_buttons() & JoystickInterface::BUTTON_13) {
			set_torque(1);
			goto_gripper(__cfg_left_open_angle, __cfg_right_open_angle);
		} else if (joystick_if_->pressed_buttons() & JoystickInterface::BUTTON_12) {
			set_torque(0);
			//        goto_gripper(__cfg_left_close_angle, __cfg_right_close_angle);
		}
		__gripper_if->set_angle(get_opening_angle());
		__gripper_if->set_holds_puck(holds_puck());

		unsigned int left_load, right_load;
		get_loads(left_load, right_load);
		__gripper_if->set_left_load(left_load & 0x400 ? -((int)left_load & 0x3ff) : (int)left_load);
		__gripper_if->set_right_load(right_load & 0x400 ? -((int)right_load & 0x3ff) : (int)right_load);
		__gripper_if->write();
	}
	cfg_mutex_.unlock();
}

bool
GripperAX12AThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	if (message->is_of_type<AX12GripperInterface::StopLeftMessage>()) {
		stop_left();
		return false; // do not enqueue StopMessage
	} else if (message->is_of_type<AX12GripperInterface::StopRightMessage>()) {
		stop_right();
		return false; // do not enqueue StopMessage
	} else if (message->is_of_type<AX12GripperInterface::StopMessage>()) {
		stop_motion();
		return false; // do not enqueue StopMessage
	} else if (message->is_of_type<AX12GripperInterface::FlushMessage>()) {
		stop_motion();
		logger->log_info(name(), "Flushing message queue");
		__gripper_if->msgq_flush();
		return false;
	} else {
		logger->log_info(name(), "Received message of type %s, enqueueing", message->type());
		return true;
	}
	return true;
}

/** Enable or disable servo.
 * @param enabled true to enable servos, false to turn them off
 */
void
GripperAX12AThread::set_enabled(bool enabled)
{
	DynamixelServoInterface::SetEnabledMessage *ena_left =
	  new DynamixelServoInterface::SetEnabledMessage(enabled);
	DynamixelServoInterface::SetEnabledMessage *ena_right =
	  new DynamixelServoInterface::SetEnabledMessage(enabled);
	__servo_if_right->msgq_enqueue(ena_left);
	__servo_if_left->msgq_enqueue(ena_right);
}

/** Stop currently running left motion. */
void
GripperAX12AThread::stop_left()
{
	DynamixelServoInterface::StopMessage *stop_message = new DynamixelServoInterface::StopMessage();
	__servo_if_left->msgq_enqueue(stop_message);
}

/** Stop currently running right motion. */
void
GripperAX12AThread::stop_right()
{
	DynamixelServoInterface::StopMessage *stop_message = new DynamixelServoInterface::StopMessage();
	__servo_if_left->msgq_enqueue(stop_message);
}

/** Stop currently running motion. */
void
GripperAX12AThread::stop_motion()
{
	stop_left();
	stop_right();
}

/** Goto desired left/right values.
 * @param left left in radians
 * @param right right in radians
 */
void
GripperAX12AThread::goto_gripper(float left, float right)
{
	__target_left                                   = left;
	__target_right                                  = right;
	DynamixelServoInterface::GotoMessage *goto_left = new DynamixelServoInterface::GotoMessage(left);
	DynamixelServoInterface::GotoMessage *goto_right =
	  new DynamixelServoInterface::GotoMessage(right);

	__servo_if_left->msgq_enqueue(goto_left);
	__servo_if_right->msgq_enqueue(goto_right);
}

/** Goto desired left/right values until given max_load (by config).
 * @param left left in radians
 * @param right right in radians
 */
void
GripperAX12AThread::goto_gripper_load(float left, float right)
{
	goto_gripper(left, right);
	load_left_pending  = true;
	load_right_pending = true;
}

/** Goto desired left/right values in a specified time.
 * @param left left in radians
 * @param right right in radians
 * @param time_sec time when to reach the desired left/right values
 */
void
GripperAX12AThread::goto_gripper_timed(float left, float right, float time_sec)
{
	DynamixelServoInterface::TimedGotoMessage *goto_timed_left =
	  new DynamixelServoInterface::TimedGotoMessage(time_sec, left);
	DynamixelServoInterface::TimedGotoMessage *goto_timed_right =
	  new DynamixelServoInterface::TimedGotoMessage(time_sec, right);

	__servo_if_left->msgq_enqueue(goto_timed_left);
	__servo_if_right->msgq_enqueue(goto_timed_right);
}

/** Set desired velocities.
 * @param left_vel left velocity
 * @param right_vel right velocity
 */
void
GripperAX12AThread::set_velocities(float left_vel, float right_vel)
{
	DynamixelServoInterface::SetVelocityMessage *left_vel_message =
	  new DynamixelServoInterface::SetVelocityMessage(left_vel);
	DynamixelServoInterface::SetVelocityMessage *right_vel_message =
	  new DynamixelServoInterface::SetVelocityMessage(right_vel);

	__servo_if_left->msgq_enqueue(left_vel_message);
	__servo_if_right->msgq_enqueue(right_vel_message);
}

// /** Set desired velocities normalized.
//  * @param left_vel left velocity ( 0 - 1 )
//  * @param right_vel right velocity ( 0 - 1 )
//  */
// void
// GripperAX12AThread::set_velocities_normalized(float left_vel, float
// right_vel)
// {
//   return set_velocities(left_vel * __max_left_speed, right_vel *
//   __max_right_speed);
// }

/** Get current loads.
 */
void
GripperAX12AThread::get_loads(unsigned int &left, unsigned int &right)
{
	left  = __servo_if_left->load();
	right = __servo_if_right->load();
}

/** Get current velocities.
 * @param left_vel upon return contains current left velocity
 * @param right_vel upon return contains current right velocity
 */
void
GripperAX12AThread::get_velocities(float &left_vel, float &right_vel)
{
	left_vel  = __servo_if_left->goal_speed();
	right_vel = __servo_if_right->goal_speed();
}

/** Set desired velocities.
 * @param left_margin left margin
 * @param right_margin right margin
 */
void
GripperAX12AThread::set_margins(float left_margin, float right_margin)
{
	if (left_margin > 0.0) {
		DynamixelServoInterface::SetMarginMessage *margin_left_message =
		  new DynamixelServoInterface::SetMarginMessage(left_margin);
		__servo_if_left->msgq_enqueue(margin_left_message);
		__left_margin = left_margin;
	}
	if (right_margin > 0.0) {
		DynamixelServoInterface::SetMarginMessage *margin_right_message =
		  new DynamixelServoInterface::SetMarginMessage(right_margin);
		__servo_if_right->msgq_enqueue(margin_right_message);
		__right_margin = right_margin;
	}
	//__logger->log_warn(name(), "Margins set to %f, %f", __left_margin,
	//__right_margin);
}

/** Set desired torque in a range of 0 - 1.
 * @param torque torque from 0 - 1
 */
void
GripperAX12AThread::set_torque(float torque)
{
	if (torque >= 0. && torque <= 1.0) {
		set_torque_left((unsigned int)(torque * 1023));
		set_torque_right((unsigned int)(torque * 1023));
	}
}

/** Set desired left servo torque in a range of 0 - 1.
 * @param torque torque from 0 - 1
 */
void
GripperAX12AThread::set_torque_left(unsigned int torque)
{
	if (torque >= 0 && torque <= 1023) {
		cur_torque_ = torque;
		logger->log_debug(name(), "Set left torque to %u", torque);

		DynamixelServoInterface::SetTorqueLimitMessage *torque_left_message =
		  new DynamixelServoInterface::SetTorqueLimitMessage(torque);
		__servo_if_left->msgq_enqueue(torque_left_message);
	} else {
		logger->log_error(name(), "Unable to set left torque to %u - value out of range!", torque);
	}
}

/** Set desired left servo torque in a range of 0 - 1.
 * @param torque torque from 0 - 1
 */
void
GripperAX12AThread::set_torque_right(unsigned int torque)
{
	if (torque >= 0 && torque <= 1023) {
		cur_torque_ = torque;
		logger->log_debug(name(), "Set right torque to %u", torque);

		DynamixelServoInterface::SetTorqueLimitMessage *torque_right_message =
		  new DynamixelServoInterface::SetTorqueLimitMessage(torque);
		__servo_if_right->msgq_enqueue(torque_right_message);
	} else {
		logger->log_error(name(), "Unable to set right torque to %u - value out of range!", torque);
	}
}

/** Get left/right value.
 * @param left upon return contains the current left value
 * @param right upon return contains the current right value
 */
void
GripperAX12AThread::get_gripper(float &left, float &right)
{
	left  = __servo_if_left->angle();
	right = __servo_if_right->angle();
}

/** Check if motion is final.
 * @return true if motion is final, false otherwise
 */
bool
GripperAX12AThread::is_final()
{
	float left, right;
	get_gripper(left, right);

	return ((fabs(left - __target_left) <= __left_margin)
	        && (fabs(right - __target_right) <= __right_margin))
	       || (!__servo_if_left->is_final() && !__servo_if_right->is_final());
}

/** Check if Gripper is enabled.
 * @return true if torque is enabled for both servos, false otherwise
 */
bool
GripperAX12AThread::is_enabled()
{
	return __servo_if_left->is_enabled() && __servo_if_right->is_enabled();
}

/** Get opening angle of gripper
 * @return angle as float
 */
float
GripperAX12AThread::get_opening_angle()
{
	return -__servo_if_left->angle() + __servo_if_right->angle();
}

/** Check if gripper holds a puck.
 * @return true if load and opening angle indicate that
 * the gripper probably holds a puck
 */
bool
GripperAX12AThread::holds_puck()
{
	return get_opening_angle() >= __cfg_angle_for_holds_puck_min
	       && get_opening_angle() <= __cfg_angle_for_holds_puck_max;
}

/** Handle config changes
 */
void GripperAX12AThread::config_value_erased(const char *path){};
void GripperAX12AThread::config_tag_changed(const char *new_tag){};
void GripperAX12AThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v){};
void
GripperAX12AThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
	if (cfg_mutex_.try_lock()) {
		try {
			load_config();
		} catch (fawkes::Exception &e) {
			logger->log_error(name(), e);
		}
	}
	cfg_mutex_.unlock();
}

void
GripperAX12AThread::load_config()
{
	logger->log_info(name(), "load config");
	// Note: due to the use of auto_ptr and RefPtr resources are automatically
	// freed on destruction, therefore no special handling is necessary in init()
	// itself!
	__cfg_driver_prefix  = config->get_string((__gripper_cfg_prefix + "driver_prefix").c_str());
	__cfg_left_servo_id  = config->get_string((__gripper_cfg_prefix + "left_servo_id").c_str());
	__cfg_right_servo_id = config->get_string((__gripper_cfg_prefix + "right_servo_id").c_str());
	// __cfg_cw_compl_margin  = config->get_uint((__gripper_cfg_prefix +
	// "cw_compl_margin").c_str());
	// __cfg_ccw_compl_margin = config->get_uint((__gripper_cfg_prefix +
	// "ccw_compl_margin").c_str());
	// __cfg_cw_compl_slope   = config->get_uint((__gripper_cfg_prefix +
	// "cw_compl_slope").c_str());
	// __cfg_ccw_compl_slope  = config->get_uint((__gripper_cfg_prefix +
	// "ccw_compl_slope").c_str());
	__cfg_left_min        = config->get_float((__gripper_cfg_prefix + "left_min").c_str());
	__cfg_left_max        = config->get_float((__gripper_cfg_prefix + "left_max").c_str());
	__cfg_right_min       = config->get_float((__gripper_cfg_prefix + "right_min").c_str());
	__cfg_right_max       = config->get_float((__gripper_cfg_prefix + "right_max").c_str());
	__cfg_left_torque     = config->get_float((__gripper_cfg_prefix + "left_torque").c_str());
	__cfg_right_torque    = config->get_float((__gripper_cfg_prefix + "right_torque").c_str());
	__cfg_goto_zero_start = config->get_bool((__gripper_cfg_prefix + "goto_zero_start").c_str());
	__cfg_turn_off        = config->get_bool((__gripper_cfg_prefix + "turn_off").c_str());

	__cfg_left_margin      = config->get_float((__gripper_cfg_prefix + "left_margin").c_str());
	__cfg_right_margin     = config->get_float((__gripper_cfg_prefix + "right_margin").c_str());
	__cfg_left_start       = config->get_float((__gripper_cfg_prefix + "left_start").c_str());
	__cfg_right_start      = config->get_float((__gripper_cfg_prefix + "right_start").c_str());
	__cfg_left_open_angle  = config->get_float((__gripper_cfg_prefix + "left_open").c_str());
	__cfg_left_close_angle = config->get_float((__gripper_cfg_prefix + "left_close").c_str());
	__cfg_left_close_load_angle =
	  config->get_float((__gripper_cfg_prefix + "left_close_load").c_str());
	__cfg_right_open_angle  = config->get_float((__gripper_cfg_prefix + "right_open").c_str());
	__cfg_right_close_angle = config->get_float((__gripper_cfg_prefix + "right_close").c_str());
	__cfg_right_close_load_angle =
	  config->get_float((__gripper_cfg_prefix + "right_close_load").c_str());
	__cfg_max_speed  = config->get_float((__gripper_cfg_prefix + "max_speed").c_str());
	__cfg_max_load   = config->get_float((__gripper_cfg_prefix + "max_load").c_str());
	__cfg_max_torque = config->get_float((__gripper_cfg_prefix + "max_torque").c_str());
	__cfg_load_for_holds_puck =
	  config->get_uint((__gripper_cfg_prefix + "load_for_holds_puck_threshold").c_str());
	__cfg_angle_for_holds_puck_min =
	  config->get_float((__gripper_cfg_prefix + "angle_for_holds_puck_min").c_str());
	__cfg_angle_for_holds_puck_max =
	  config->get_float((__gripper_cfg_prefix + "angle_for_holds_puck_max").c_str());
	__cfg_center_angle_correction_amount =
	  config->get_float((__gripper_cfg_prefix + "center_angle_correction_amount").c_str());
	__cfg_ifid_joystick_ = config->get_string(__gripper_cfg_prefix + "joystick_interface_id");

#ifdef HAVE_TF
	__cfg_publish_transforms =
	  config->get_bool((__gripper_cfg_prefix + "publish_transforms").c_str());
#endif

#ifdef HAVE_TF
	if (__cfg_publish_transforms) {
		// float left_trans_x  =
		//     config->get_float((__gripper_cfg_prefix + "left_trans_x").c_str());
		// float left_trans_y  =
		//     config->get_float((__gripper_cfg_prefix + "left_trans_y").c_str());
		// float left_trans_z  =
		//     config->get_float((__gripper_cfg_prefix + "left_trans_z").c_str());
		// float right_trans_x =
		//     config->get_float((__gripper_cfg_prefix + "right_trans_x").c_str());
		// float right_trans_y =
		//     config->get_float((__gripper_cfg_prefix + "right_trans_y").c_str());
		// float right_trans_z =
		//     config->get_float((__gripper_cfg_prefix + "right_trans_z").c_str());

		__cfg_gripper_name = config->get_string((__gripper_cfg_prefix + "name").c_str());

		std::string frame_id_prefix = std::string("/") + __cfg_gripper_name;
		try {
			frame_id_prefix = config->get_string((__gripper_cfg_prefix + "frame_id_prefix").c_str());
		} catch (Exception &e) {
		} // ignore, use default

		__cfg_base_frame = frame_id_prefix + "/base";
		__cfg_left_link  = frame_id_prefix + "/left";
		__cfg_right_link = frame_id_prefix + "/right";

		// __translation_left.setValue(left_trans_x, left_trans_y, left_trans_z);
		// __translation_right.setValue(right_trans_x, right_trans_y,
		// right_trans_z);
	}
#endif
}
