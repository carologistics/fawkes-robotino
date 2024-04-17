/***************************************************************************
 *  direct_com_thread.h - Arduino com thread for direct communication
 *
 *  Created: Mon Apr 04 11:48:36 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
 *                  2022  Matteo Tschesche
 *                  2023  Tim Wendt
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

#ifndef __PLUGINS_ARDUINO_COM_THREAD_H_
#define __PLUGINS_ARDUINO_COM_THREAD_H_

#include "com_message.h"
#include "interfaces/ArduinoInterface.h"
#include "serialport.h"
#include "tf_thread.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <config/change_handler.h>
#include <core/threading/thread.h>
#include <interfaces/JoystickInterface.h>
#include <tf/types.h>

#include <memory>

#define NEMA_STEPS_PER_REVOLUTION 200.0 * 4.0

class ArduinoComMessage;

namespace fawkes {
class Clock;
class TimeWait;

class BatteryInterface;
class ArduinoInterface;
} // namespace fawkes

class ArduinoComThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::ClockAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::BlackBoardInterfaceListener,
                         public fawkes::TransformAspect,
                         public fawkes::ConfigurationChangeHandler
{
public:
	ArduinoComThread();
	/**
   * @brief Constructor for the arduino communication thread
   *
   * @param cfg_name Name of the config file
   * @param cfg_prefix Prefix tags to arduino config
   * @param tf_thread Pointer to transform thread
   */
	ArduinoComThread(std::string &cfg_name, std::string &cfg_prefix, ArduinoTFThread *tf_thread);
	virtual ~ArduinoComThread();

	virtual void init();
	virtual void loop();
	void         initInterface();
	virtual void finalize();

	// For BlackBoardInterfaceListener
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) throw();

	virtual void config_value_erased(const char *path) override;
	virtual void config_tag_changed(const char *new_tag) override;
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v) override;
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v) override;

	/**
   * @brief All variables that define the position of the gripper
   * X,Y,Z position of the axis
   * A position of the motor, that controls the gripper
   */
	typedef enum { X, Y, Z, A } gripper_pose_t;

private:
	bool port_disconnected = false;
	void disconnect_callback();

	std::string  cfg_device_;
	unsigned int cfg_speed_;
	unsigned int cfg_accel_;
	std::string  cfg_prefix_;
	std::string  cfg_name_;
	std::string  cfg_ifid_joystick_;
	std::string  cfg_gripper_frame_id_;
	std::string  cfg_gripper_dyn_x_frame_id_;
	std::string  cfg_gripper_dyn_y_frame_id_;
	std::string  cfg_gripper_dyn_z_frame_id_;
	float        cfg_x_max_;
	float        cfg_y_max_;
	float        cfg_z_max_;

	unsigned int cfg_speeds_[4];
	unsigned int cfg_accs_[4];

	float cfg_steps_per_mm_[3];

	unsigned int cfg_a_toggle_steps_;
	unsigned int cfg_a_half_toggle_steps_;

	volatile bool movement_pending_;

	unsigned int msecs_to_wait_;
	unsigned int no_data_count = 0;

	char current_arduino_status_;
	int  gripper_pose_[3]     = {0, 0, 0};
	int  goal_gripper_pose[3] = {0, 7500, 13000};
	bool goal_gripper_is_open = false;
	bool is_homed             = false;
	int  home_gripper_pose[3] = {0, 7500, 13000};

	void timer_callback();
	void handle_nodata();
	void receive(const std::string &buff);

	std::deque<ArduinoComMessage *> messages_;
	std::mutex                      queue_mutex;
	fawkes::Time                    expected_finish_time_;

	std::unique_ptr<SerialPort> port_;
	boost::thread               timer_thread;

	fawkes::ArduinoInterface  *arduino_if_;
	fawkes::JoystickInterface *joystick_if_;

	ArduinoTFThread *tf_thread_;

	void load_config();

	void append_message_to_queue(char cmd, unsigned int value = 0);
	void append_message_to_queue(ArduinoComMessage *msg);
	bool add_command_to_message(ArduinoComMessage *msg, char command, unsigned int value);
	bool send_message(ArduinoComMessage &msg);
	bool send_message_from_queue();
	void append_config_messages();

	void handle_queue();

	float inline round_to_2nd_dec(float f);
	void pose_publish_tf();

	bool handle_xyz_message(fawkes::ArduinoInterface::MoveXYZAbsMessage *message);

	bool         handle_rel_xyz_messag(fawkes::ArduinoInterface::MoveXYZRelMessage *msg);
	inline float from_arduino_units(float in_steps, ArduinoComThread::gripper_pose_t axis);
};

#endif
