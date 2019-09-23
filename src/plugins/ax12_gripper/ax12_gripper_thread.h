
/***************************************************************************
 *  ax12_gripper_thread.h - AX12A Gripper unit act thread
 *
 *  Created: Sun Mar 29 16:11:36 2015
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

#ifndef __PLUGINS_GRIPPER_ROBOTIS_AX12_THREAD_H_
#define __PLUGINS_GRIPPER_ROBOTIS_AX12_THREAD_H_

#ifdef HAVE_TF
#	include <aspect/tf.h>
#endif
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <config/change_handler.h>
#include <core/threading/thread.h>
#include <interfaces/DynamixelServoInterface.h>
#include <interfaces/JoystickInterface.h>
#include <utils/time/time.h>

#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <memory>
#include <string>

namespace fawkes {
class AX12GripperInterface;
class LedInterface;
class JointInterface;
class ReadWriteLock;
class WaitCondition;
} // namespace fawkes

class RobotisAX12A;

class GripperAX12AThread : public fawkes::Thread,
                           public fawkes::ClockAspect,
                           public fawkes::BlockedTimingAspect,
#ifdef HAVE_TF
                           public fawkes::TransformAspect,
#endif
                           public fawkes::BlackBoardInterfaceListener,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::ConfigurationChangeHandler

{
public:
	GripperAX12AThread(std::string &gripper_cfg_prefix);

	virtual void init();
	/* virtual bool prepare_finalize_user(); */
	virtual void finalize();
	virtual void loop();

	// For BlackBoardInterfaceListener
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message *  message) throw();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::AX12GripperInterface *   __gripper_if;
	fawkes::LedInterface *           __led_if;
	fawkes::JointInterface *         __leftjoint_if;
	fawkes::JointInterface *         __rightjoint_if;
	fawkes::DynamixelServoInterface *__servo_if_left;
	fawkes::DynamixelServoInterface *__servo_if_right;
	fawkes::JoystickInterface *      joystick_if_;

	/* fawkes::RefPtr<RobotisAX12A> __ax12a; */
	std::string __gripper_cfg_prefix;
	std::string __cfg_gripper_name;
	std::string __cfg_driver_prefix;
	std::string __cfg_left_servo_id;
	std::string __cfg_right_servo_id;
	/* unsigned char __cfg_left_servo_id; */
	/* unsigned char __cfg_right_servo_id; */
	/* unsigned int __cfg_cw_compl_margin; */
	/* unsigned int __cfg_ccw_compl_margin; */
	/* unsigned int __cfg_cw_compl_slope; */
	/* unsigned int __cfg_ccw_compl_slope; */
	bool         __cfg_goto_zero_start;
	bool         __cfg_turn_off;
	float        __cfg_left_min;
	float        __cfg_left_max;
	float        __cfg_right_min;
	float        __cfg_right_max;
	float        __cfg_left_margin;
	float        __cfg_right_margin;
	float        __cfg_left_offset;
	float        __cfg_right_offset;
	float        __cfg_left_start;
	float        __cfg_right_start;
	float        __cfg_left_torque;
	float        __cfg_right_torque;
	float        __cfg_left_open_angle;
	float        __cfg_left_close_angle;
	float        __cfg_left_close_load_angle;
	float        __cfg_right_open_angle;
	float        __cfg_right_close_angle;
	float        __cfg_right_close_load_angle;
	float        __cfg_max_speed;
	float        __cfg_max_torque;
	float        __cfg_max_load;
	unsigned int __cfg_load_for_holds_puck;
	float        __cfg_angle_for_holds_puck_min;
	float        __cfg_angle_for_holds_puck_max;
	float        __cfg_center_angle_correction_amount;
	std::string  __cfg_ifid_joystick_;

	float __target_left;
	float __target_right;
	float __left_margin;
	float __right_margin;

	bool         load_left_pending;
	bool         load_right_pending;
	bool         center_pending;
	unsigned int cur_z_goal_speed;
	bool         z_alignment_pending;
	fawkes::Time time_to_stop_z_align;

	float        cur_torque_;
	fawkes::Time motion_start_timestamp_;
	bool         slap_left_pending_;
	bool         slap_right_pending_;

#ifdef HAVE_TF
	std::string __cfg_base_frame;
	std::string __cfg_left_link;
	std::string __cfg_right_link;

	fawkes::tf::Vector3 __translation_left;
	fawkes::tf::Vector3 __translation_right;

	fawkes::Mutex cfg_mutex_;

	bool __cfg_publish_transforms;
#endif

	float __last_left;
	float __last_right;
	void  goto_gripper(float left, float right);
	void  goto_gripper_load(float left, float right);
	void  goto_gripper_timed(float left, float right, float time_sec);
	void  rel_goto_z(int rel_z);
	void  get_gripper(float &left, float &right);
	void  get_gripper(float &left, float &right, fawkes::Time &time);
	void  set_velocities(float left_vel, float right_vel);
	void  set_velocities_normalized(float left_vel, float right_vel);
	void  get_velocities(float &left_vel, float &right_vel);
	void  get_loads(unsigned int &left, unsigned int &right);
	void  set_margins(float left_margin, float right_margin);
	void  set_torque(float torque);
	void  set_torque_left(unsigned int torque);
	void  set_torque_right(unsigned int torque);
	bool  is_final();
	bool  is_enabled();
	void  set_enabled(bool enabled);
	void  set_led_enabled(bool enabled);
	float get_opening_angle();
	bool  holds_puck();
	void  stop_motion();
	/* bool has_fresh_data(); */
	/* void wait_for_fresh_data(); */
	void stop_left();
	void stop_right();
	/* void set_servo_angle(unsigned int servo_id, float servo_angle); */
	void load_config();
	void config_value_erased(const char *path);
	void config_tag_changed(const char *new_tag);
	void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
	void config_value_changed(const fawkes::Configuration::ValueIterator *v);
};

#endif
