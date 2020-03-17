/***************************************************************************
 *  direct_com_thread.h - Arduino com thread for direct communication
 *
 *  Created: Mon Apr 04 11:48:36 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
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
#include <utils/time/time.h>

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <memory>

#define NEMA_STEPS_PER_REVOLUTION 200.0 * 4.0

class ArduinoComMessage;

namespace fawkes {
class Mutex;
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
	virtual void finalize();

	/**
   * @brief Checks if the serial connection is open
   *
   * @return True if the serial connection is open
   */
	virtual bool is_connected();

	// For BlackBoardInterfaceListener
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message *  message) throw();

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
	void open_device();
	void close_device();
	void flush_device();

	bool        sync_with_arduino();
	std::string read_packet(unsigned int timeout);
	void        send_message(ArduinoComMessage &msg);

	void handle_nodata(const boost::system::error_code &ec);
	bool send_one_message();

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
  unsigned int cfg_x_microstep_;
  unsigned int cfg_y_microstep_;
  unsigned int cfg_z_microstep_;

	bool movement_pending_;
	bool calibrated_;
	char current_arduino_status_;

	unsigned int msecs_to_wait_;

	// gripper pose to be stored in X, Y, Z
	// TODO: setup proper values!
	int gripper_pose_[3] = {100000, 100000, 100000};

	size_t bytes_read_;
	bool   read_pending_;
	bool   set_speed_pending_;
	bool   set_acceleration_pending_;
	bool   home_pending_;

	bool         opened_;
	unsigned int open_tries_;

	std::queue<ArduinoComMessage *> messages_;

	boost::asio::io_service     io_service_;
	boost::asio::serial_port    serial_;
	boost::asio::deadline_timer deadline_;
	boost::asio::streambuf      input_buffer_;

	boost::mutex               io_mutex_;
	fawkes::ArduinoInterface * arduino_if_;
	fawkes::JoystickInterface *joystick_if_;

	ArduinoTFThread *tf_thread_;

	void load_config();

	void append_message_to_queue(ArduinoComMessage::command_id_t cmd,
	                             unsigned int                    value   = 0,
	                             unsigned int                    timeout = 1000);
	void append_message_to_queue(ArduinoComMessage *msg);
	bool add_command_to_message(ArduinoComMessage *             msg,
	                            ArduinoComMessage::command_id_t command,
	                            unsigned int                    value);

	float inline round_to_2nd_dec(float f);
	void pose_publish_tf();

protected:
	/** Mutex to protect data_. Lock whenever accessing it. */
	fawkes::Mutex *data_mutex_;
};

#endif
