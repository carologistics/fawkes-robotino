//TODO PARSING OF RETURN TO GOAL AND INTERFACE IS NOT WOKRING
//TODO CHECK IF COREDUMPS ARE GONE
//TODO IF PORT CLOSE TRY TO OPEN INSTANTLY
//TODO FIX MAJOR CACHE ISSUES ON THE ROBOTINO AS WELL AS CMAKE CHRONO ISSUES *  com_thread.cpp - Arduino com thread
//TODO HANLDE HAF
//TODO HANDLE STOP
//TODO GRIPPER CLICKING
//TODO SOME TIMES CONNECTION DIES
//TODO When unpugging the deadline_timer stops.
/***************************************************************************
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
 *                  2022  Matteo Tschesche
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

#include "com_thread.h"

#include "ArduinoSketch/commands.h"
#include "com_message.h"
#include "interface/interface.h"
#include "interface/message.h"
#include "serialport.h"

#include <baseapp/run.h>
#include <bits/types/struct_iovec.h>
#include <core/threading/mutex.h>
// #include <core/threading/mutex_locker.h>
#include <interfaces/ArduinoInterface.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/pthread/thread_data.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <libudev.h>
#include <memory>
#include <thread>
#include <unistd.h>

using namespace fawkes;

/** @class ArduinoComThread "com_thread.h"
 * Thread to communicate with an Arduino Uno via boost::asio
 * @author Tim Niemueller, Nicolas Limpert, Matteo Tschesche
 */

/** Constructor. */
ArduinoComThread::ArduinoComThread(std::string     &cfg_name,
                                   std::string     &cfg_prefix,
                                   ArduinoTFThread *tf_thread)
: Thread("ArduinoComThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("ArduinoThread(%s)", cfg_prefix.c_str()),
  fawkes::TransformAspect(),
  ConfigurationChangeHandler(cfg_prefix.c_str()), //serial_(io_service_), io_service_(),
  deadline_timer(io_service_),
  // mutex_(new boost::mutex()),
  tf_thread_(tf_thread)
{
	cfg_prefix_ = cfg_prefix;
	cfg_name_   = cfg_name;
}
/** Destructor. */
ArduinoComThread::~ArduinoComThread()
{
}

inline float
ArduinoComThread::from_arduino_units(float in_meter, ArduinoComThread::gripper_pose_t axis)
{
	return in_meter * cfg_steps_per_mm_[axis] * 1000.0;
}

inline float
ArduinoComThread::to_arduino_units(float in_steps, ArduinoComThread::gripper_pose_t axis)
{
	return in_steps / cfg_steps_per_mm_[axis] / 1000.0;
}

void
ArduinoComThread::receive(const std::string &buf)
{
	no_data_count = 0;
	//TODO add cfg
	deadline_timer.expires_from_now(boost::posix_time::seconds(5));
	logger->log_warn(name(), "received %s", buf.c_str());

	logger->log_debug(name(), "read_packet: %s", buf.c_str());

	bool is_open;
	if (!ArduinoComMessage::parse_message_from_arduino(
	      gripper_pose_, is_open, current_arduino_status_, buf)) {
		append_config_messages();
		logger->log_error(name(), "Arduino relaunched needed to be reconfigured %s", buf.c_str());
		return;
	}
	if (current_arduino_status_ == 'E') {
		logger->log_error(name(), "Arduino error: %s", buf.c_str());
		port_.reset();
	}

	arduino_if_->set_final(false);
	if (current_arduino_status_ == 'I') {
		arduino_if_->set_final(true);
		arduino_if_->set_status(ArduinoInterface::IDLE);
		if (gripper_pose_[X] != goal_gripper_pose[X] || gripper_pose_[Y] != goal_gripper_pose[Y]
		    || gripper_pose_[Z] != goal_gripper_pose[Z] || is_open != goal_gripper_is_open) {
			arduino_if_->set_final(false);
			ArduinoComMessage *arduino_msg = new ArduinoComMessage();
			add_command_to_message(arduino_msg, CMD_X_NEW_POS, goal_gripper_pose[X]);
			add_command_to_message(arduino_msg, CMD_Y_NEW_POS, goal_gripper_pose[Y]);
			add_command_to_message(arduino_msg, CMD_Z_NEW_POS, goal_gripper_pose[Z]);

			if (goal_gripper_pose) {
				// add_command_to_message(arduino_msg, , goal_gripper_pose[Z]) //TODO handle half
			}

			append_message_to_queue(arduino_msg);

			arduino_if_->set_status(ArduinoInterface::MOVING);
		}
	}

	arduino_if_->set_x_position(from_arduino_units(gripper_pose_[X], X));
	arduino_if_->set_x_position(from_arduino_units(gripper_pose_[Y], Y));
	arduino_if_->set_x_position(from_arduino_units(gripper_pose_[Z], Z));
	arduino_if_->set_gripper_closed(is_open);
	arduino_if_->write();
}

void
ArduinoComThread::init()
{
	load_config();
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

	initInterface();
	deadline_timer.expires_from_now(boost::posix_time::seconds(5));
	// deadline_timer.async_wait(
	//   [this](const boost::system::error_code &error) { timer_callback(error); });
	io_service_thread_ = boost::thread([this]() {
		while (true) {
			timer_callback();
			// io_service_.run_for(std::chrono::seconds(2));
			wakeup();
			boost::this_thread::sleep_for(boost::chrono::seconds(1));
		}
		printf("NAH DAS LIEF JA NICHT SO GUT");
	});

	movement_pending_ = false;

	joystick_if_ =
	  blackboard->open_for_reading<JoystickInterface>("Joystick", cfg_ifid_joystick_.c_str());

	movement_pending_ = false;

	// initially calibrate the gripper on startup
	// move to home position on startup
	// open_device();

	bbil_add_message_interface(arduino_if_);

	blackboard->register_listener(this);
	arduino_if_->set_final(true);

	arduino_if_->set_status(ArduinoInterface::IDLE);
	arduino_if_->write();
	wakeup();
}

void
ArduinoComThread::disconnect_callback()
{
	port_disconnected = true;
}

//@brief Writes the config values to the Interface
void
ArduinoComThread::initInterface()
{
	arduino_if_->set_x_max(cfg_x_max_);
	arduino_if_->set_y_max(cfg_y_max_);
	arduino_if_->set_z_max(cfg_z_max_);
	arduino_if_->write();
}

void
ArduinoComThread::finalize()
{
	deadline_timer.cancel();
	io_service_thread_.interrupt();
	blackboard->unregister_listener(this);
	blackboard->close(arduino_if_);
	port_.reset();
}

void
ArduinoComThread::append_message_to_queue(char cmd, unsigned int value, unsigned int timeout)
{
	ArduinoComMessage *msg = new ArduinoComMessage(cmd, value);
	msg->set_msecs_if_lower(timeout);
	append_message_to_queue(msg);
}

void
ArduinoComThread::append_message_to_queue(ArduinoComMessage *msg)
{
	printf("STATRTED\n");
	if (*msg == messages_) {
		printf("EQ\n");
		return;
	}
	printf("ENDED\n");
	messages_.push(msg);
}

bool
ArduinoComThread::add_command_to_message(ArduinoComMessage *msg, char command, unsigned int value)
{
	// TODO: Check if consistency for sending values is kept - is it always
	// unsigned int?
	if (!msg->add_command(command, value)) {
		logger->log_error(name(),
		                  "Faulty command! id: %c value: %u size: %u , index: %u",
		                  command,
		                  value,
		                  ArduinoComMessage::num_digits(value) + 1,
		                  msg->get_cur_buffer_index());
		return false;
	}
	return true;
}

void
ArduinoComThread::append_config_messages()
{
	printf("HALLO WAS GEHT\n");
	ArduinoComMessage *msg = new ArduinoComMessage();

	add_command_to_message(msg, CMD_X_NEW_ACC, cfg_accs_[X]);
	add_command_to_message(msg, CMD_Y_NEW_ACC, cfg_accs_[Y]);
	add_command_to_message(msg, CMD_Z_NEW_ACC, cfg_accs_[Z]);
	add_command_to_message(msg, CMD_A_NEW_ACC, cfg_accs_[A]);
	add_command_to_message(msg, CMD_X_NEW_SPEED, cfg_speeds_[X]);
	add_command_to_message(msg, CMD_Y_NEW_SPEED, cfg_speeds_[Y]);
	add_command_to_message(msg, CMD_Z_NEW_SPEED, cfg_speeds_[Z]);
	add_command_to_message(msg, CMD_A_NEW_SPEED, cfg_speeds_[A]);
	// ue(CMD_CALIBRATE, 0, 50000); //now done on arduino
	add_command_to_message(msg, CMD_A_SET_TOGGLE_STEPS, cfg_a_toggle_steps_);
	add_command_to_message(msg, CMD_A_SET_HALF_TOGGLE_STEPS, cfg_a_half_toggle_steps_);

	append_message_to_queue(msg);
}

void
ArduinoComThread::handle_queue()
{
	int i = 0;
	while (messages_.size() > 0 && i < 10) {
		arduino_if_->set_final(false);
		arduino_if_->set_status(ArduinoInterface::MOVING);
		arduino_if_->write();

		printf("starting done%i\n", i);
		send_message_from_queue();

		arduino_if_->write();

		tf_thread_->set_position(arduino_if_->x_position(),
		                         arduino_if_->y_position(),
		                         arduino_if_->z_position());
		++i;
		printf("not done%i\n", i);
	}
}

void
ArduinoComThread::loop()
{
	if (!port_ || port_disconnected) {
		printf("GOING TO SHUTDOWN\n");
		port_.reset();
		port_disconnected = false;
		try {
			printf("Opening the device\n");
			port_ = std::make_unique<SerialPort>(
			  cfg_device_,
			  boost::bind(&ArduinoComThread::receive, this, boost::placeholders::_1),
			  boost::bind(&ArduinoComThread::disconnect_callback, this));
		} catch (boost::system::error_code ec) {
			logger->log_error(name(),
			                  "Could now open PORT: %s, %s",
			                  cfg_device_.c_str(),
			                  ec.what().c_str());
		}
	}
	printf("Loop\n");
	handle_queue();
}

bool
ArduinoComThread::send_message(ArduinoComMessage &msg)
{
	printf("SENDING %s\n", msg.buffer().c_str());
	//TODO: Check of is open
	// msg.get_position_data(goal_gripper_pose, goal_gripper_is_open);
	if (!port_) {
		printf("DAS DUMM \n");
		return false;
	}
	if (!port_->write(msg.buffer())) {
		logger->log_error(name(), "Error while trying to send data to Arduino!");
		// port_.reset();
		printf("TJDSFKLJ\n");
		return false;
	}
	usleep(100);
	return true;
}

bool
ArduinoComThread::send_message_from_queue()
{
	if (!port_) {
		printf("NONONO\n");
		return false; //Let the timer handle that
	}
	if (messages_.size() > 0) {
		printf("sending\n");
		ArduinoComMessage *cur_msg = messages_.front();
		msecs_to_wait_             = cur_msg->get_msecs();
		if (!send_message(*cur_msg)) {
			printf("not send\n");
			return false;
		}

		printf("send\n");
		messages_.pop();
		delete cur_msg;
		return true;
	}
	return false;
}

void
ArduinoComThread::timer_callback()
{
	printf("round: %i port %d\n", no_data_count, !!port_);

	if (!port_)
		return;

	++no_data_count;

	//TODO add cfg
	if (no_data_count > 60) {
		// port_.reset();
		no_data_count = 0;
		return;
	}
	append_message_to_queue(CMD_STATUS_REQ, 0, 1000);
	;

	wakeup();
}

void
ArduinoComThread::load_config()
{
	config->add_change_handler(this);
	try {
		logger->log_info(name(), "load_config");
		cfg_device_           = config->get_string(cfg_prefix_ + "/device");
		cfg_speed_            = config->get_int(cfg_prefix_ + "/speed");
		cfg_accel_            = config->get_int(cfg_prefix_ + "/accel");
		cfg_ifid_joystick_    = config->get_string(cfg_prefix_ + "/joystick_interface_id");
		cfg_gripper_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_frame_id");

		cfg_x_max_ = config->get_float(cfg_prefix_ + "/x_max");
		cfg_y_max_ = config->get_float(cfg_prefix_ + "/y_max");
		cfg_z_max_ = config->get_float(cfg_prefix_ + "/z_max");

		cfg_speeds_[X] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/speed_x").c_str(), 0.f);
		cfg_speeds_[Y] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/speed_y").c_str(), 0.f);
		cfg_speeds_[Z] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/speed_z").c_str(), 0.f);
		cfg_speeds_[A] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/speed_a").c_str(), 0.f);
		cfg_accs_[X] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/acc_x").c_str(), 0.f);
		cfg_accs_[Y] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/acc_y").c_str(), 0.f);
		cfg_accs_[Z] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/acc_z").c_str(), 0.f);
		cfg_accs_[A] =
		  config->get_float_or_default((cfg_prefix_ + "/firmware_settings/acc_a").c_str(), 0.f);

		int cfg_x_microstep = config->get_int(cfg_prefix_ + "/hardware_settings/x_micro_stepping");
		int cfg_y_microstep = config->get_int(cfg_prefix_ + "/hardware_settings/y_micro_stepping");
		int cfg_z_microstep = config->get_int(cfg_prefix_ + "/hardware_settings/z_micro_stepping");

		// the factor the microstepping mode needs to be multiplied with
		// depends on the individual thread diameter and slope.
		cfg_steps_per_mm_[X] = 200.0 * cfg_x_microstep / 3.0;
		cfg_steps_per_mm_[Y] = 200.0 * cfg_y_microstep / 2.0;
		cfg_steps_per_mm_[Z] = 200.0 * cfg_z_microstep / 1.5;

		cfg_a_toggle_steps_ = config->get_int(cfg_prefix_ + "/hardware_settings/a_toggle_steps");
		cfg_a_half_toggle_steps_ =
		  config->get_int(cfg_prefix_ + "/hardware_settings/a_half_toggle_steps");
		// 2mm / rotation
	} catch (Exception &e) {
	}
}

bool
ArduinoComThread::handle_xyz_message(ArduinoInterface::MoveXYZAbsMessage *msg)
{
	ArduinoComMessage *arduino_msg = new ArduinoComMessage();

	fawkes::tf::StampedTransform tf_pose_target;

	try {
		tf_listener->lookup_transform(cfg_gripper_frame_id_, msg->target_frame(), tf_pose_target);
	} catch (fawkes::tf::ExtrapolationException &e) {
		logger->log_error(name(), "Extrapolation error");
		return false;
	} catch (fawkes::tf::ConnectivityException &e) {
		logger->log_error(name(), "Connectivity exception: %s", e.what());
		return false;
	} catch (fawkes::IllegalArgumentException &e) {
		logger->log_error(name(),
		                  "IllegalArgumentException exception - did you set "
		                  "the frame_id?: %s",
		                  e.what());
		return false;
	} catch (fawkes::Exception &e) {
		logger->log_error(name(), "Other exception: %s", e.what());
		return false;
	}
	logger->log_info(
	  name(), "Target: %f,%f,%f in frame %s", msg->x(), msg->y(), msg->z(), msg->target_frame());

	float goal_x = tf_pose_target.getOrigin().getX() + msg->x();
	float goal_y = tf_pose_target.getOrigin().getY() + msg->y() + cfg_y_max_ / 2.;
	float goal_z = tf_pose_target.getOrigin().getZ() + msg->z();

	goal_gripper_pose[X] = goal_x;
	goal_gripper_pose[Y] = goal_y;
	goal_gripper_pose[Z] = goal_z;

	logger->log_debug(name(), "Transformed target axis values: %f,%f,%f", goal_x, goal_y, goal_z);

	bool msg_has_data = false;
	int  d            = 0;
	if (goal_x >= 0. && goal_x <= arduino_if_->x_max()) {
		int new_abs_x = round_to_2nd_dec(goal_x * cfg_steps_per_mm_[X] * 1000.0);
		logger->log_info(name(), "Set new X: %u", new_abs_x);
		add_command_to_message(arduino_msg, CMD_X_NEW_POS, new_abs_x);

		// calculate millseconds needed for this movement
		d = new_abs_x - gripper_pose_[X];
		arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
		msg_has_data = true;
	} else {
		logger->log_error(name(), "Motion exceeds X dimension: %f", msg->x());
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_X);
		arduino_if_->write();
	}

	if (goal_y >= 0. && goal_y <= arduino_if_->y_max()) {
		int new_abs_y = round_to_2nd_dec(goal_y * cfg_steps_per_mm_[Y] * 1000.0);
		logger->log_debug(name(), "Set new Y: %u", new_abs_y);
		add_command_to_message(arduino_msg, CMD_Y_NEW_POS, new_abs_y);

		// calculate millseconds needed for this movement
		d = new_abs_y - gripper_pose_[Y];
		arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
		msg_has_data = true;
	} else {
		logger->log_error(name(), "Motion exceeds Y dimension: %f", msg->y());
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_Y);
		arduino_if_->write();
	}
	if (goal_z >= 0. && goal_z <= arduino_if_->z_max()) {
		int new_abs_z = round_to_2nd_dec(goal_z * cfg_steps_per_mm_[Z] * 1000.0);
		logger->log_debug(name(), "Set new Z: %u", new_abs_z);
		add_command_to_message(arduino_msg, CMD_Z_NEW_POS, new_abs_z);

		// calculate millseconds needed for this movement
		d = new_abs_z - gripper_pose_[Z];
		arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
		msg_has_data = true;
	} else {
		logger->log_error(name(), "Motion exceeds Z dimension: %f", msg->z());
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_Z);
		arduino_if_->write();
	}

	append_message_to_queue(arduino_msg);
	return true;
}

bool
ArduinoComThread::handle_rel_xyz_messag(ArduinoInterface::MoveXYZRelMessage *msg)
{
	ArduinoComMessage *arduino_msg = new ArduinoComMessage();

	bool msg_has_data = false;

	float cur_x = gripper_pose_[X] / cfg_steps_per_mm_[X] / 1000.;
	float cur_y = gripper_pose_[Y] / cfg_steps_per_mm_[Y] / 1000.;
	float cur_z = gripper_pose_[Z] / cfg_steps_per_mm_[Z] / 1000.;
	logger->log_debug(name(),
	                  "Move rel: %f %f %f cur pose: %f %f %f",
	                  msg->x(),
	                  msg->y(),
	                  msg->z(),
	                  cur_x,
	                  cur_y,
	                  cur_z);
	if (msg->x() + cur_x >= 0. && msg->x() + cur_x <= arduino_if_->x_max()) {
		int new_abs_x        = round_to_2nd_dec((msg->x() + cur_x) * cfg_steps_per_mm_[X] * 1000.0);
		goal_gripper_pose[X] = new_abs_x;
		logger->log_info(name(), "Set new X: %u", new_abs_x);
		add_command_to_message(arduino_msg, CMD_X_NEW_POS, new_abs_x);

		// calculate millseconds needed for this movement
		int d = new_abs_x - gripper_pose_[X];
		arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
		msg_has_data = true;
	} else {
		logger->log_error(name(), "Motion exceeds X dimension: %f", msg->x() + cur_x);
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_X);
		arduino_if_->write();
	}

	if (msg->y() + cur_y >= 0. && msg->y() + cur_y <= arduino_if_->y_max()) {
		int new_abs_y        = round_to_2nd_dec((msg->y() + cur_y) * cfg_steps_per_mm_[Y] * 1000.0);
		goal_gripper_pose[Y] = new_abs_y;
		logger->log_debug(name(), "Set new Y: %u", new_abs_y);
		add_command_to_message(arduino_msg, CMD_Y_NEW_POS, new_abs_y);

		// calculate millseconds needed for this movement
		int d = new_abs_y - gripper_pose_[Y];
		arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
		msg_has_data = true;
	} else {
		logger->log_error(name(), "Motion exceeds Y dimension: %f", msg->y() + cur_y);
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_Y);
		arduino_if_->write();
	}
	if (msg->z() + cur_z >= 0. && msg->z() + cur_z <= arduino_if_->z_max()) {
		int new_abs_z        = round_to_2nd_dec((msg->z() + cur_z) * cfg_steps_per_mm_[Z] * 1000.0);
		goal_gripper_pose[Z] = new_abs_z;
		logger->log_debug(name(), "Set new Z: %u", new_abs_z);
		add_command_to_message(arduino_msg, CMD_Z_NEW_POS, new_abs_z);

		// calculate millseconds needed for this movement
		int d = new_abs_z - gripper_pose_[Z];
		arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
		msg_has_data = true;
	} else {
		logger->log_error(name(), "Motion exceeds Z dimension: %f", msg->z() + cur_z);
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_Z);
		arduino_if_->write();
	}

	if (msg_has_data == true) {
		append_message_to_queue(arduino_msg);
	}
	return true;
}

//TODO make good conversion class
bool
ArduinoComThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	bool status = false;
	if (message->is_of_type<ArduinoInterface::MoveXYZAbsMessage>()) {
		status = handle_xyz_message((ArduinoInterface::MoveXYZAbsMessage *)message);
	} else if (message->is_of_type<ArduinoInterface::MoveXYZRelMessage>()) {
		status = handle_rel_xyz_messag((ArduinoInterface::MoveXYZRelMessage *)message);
	} else if (message->is_of_type<ArduinoInterface::ToHomeMessage>()) {
		ArduinoComMessage *msg = new ArduinoComMessage();
		printf("joa\n");
		add_command_to_message(msg, CMD_X_NEW_POS, 0);
		add_command_to_message(msg, CMD_Y_NEW_POS, (cfg_y_max_ / 2) * cfg_steps_per_mm_[Y] * 1000);
		add_command_to_message(msg, CMD_Z_NEW_POS, 0);

		append_message_to_queue(msg);
	} else if (message->is_of_type<ArduinoInterface::CalibrateMessage>()) {
		append_message_to_queue(CMD_CALIBRATE);
		status = true;
	} else if (message->is_of_type<ArduinoInterface::CloseGripperMessage>()) {
		append_message_to_queue(CMD_CLOSE);
		status = true;
	} else if (message->is_of_type<ArduinoInterface::OpenGripperMessage>()) {
		append_message_to_queue(CMD_OPEN);
		status = true;
	} else if (message->is_of_type<ArduinoInterface::OpenHalfGripperMessage>()) {
		append_message_to_queue(CMD_HALF_OPEN);
		status = true;
	} else if (message->is_of_type<ArduinoInterface::StatusUpdateMessage>()) {
		append_message_to_queue(CMD_STATUS_REQ);
		status = true;
	}

	wakeup();
	return status;
}

float inline ArduinoComThread::round_to_2nd_dec(float f)
{
	return round(f * 100.) / 100.;
}

void
ArduinoComThread::config_value_changed(const Configuration::ValueIterator *v)
{
	if (v->valid()) {
		std::string path       = v->path();
		std::string sufx       = path.substr(strlen(cfg_prefix_.c_str()));
		std::string sub_prefix = sufx.substr(0, sufx.substr(1).find("/") + 1);
		std::string full_pfx   = cfg_prefix_.c_str() + sub_prefix;
		std::string opt        = path.substr(full_pfx.length());
		if (sub_prefix == "firmware_settings") {
			ArduinoComMessage *arduino_msg  = new ArduinoComMessage();
			bool               msg_has_data = false;
			if (opt == "/speed_x") {
				cfg_speeds_[X] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_X_NEW_SPEED, cfg_speeds_[X]);
				msg_has_data = true;
			} else if (opt == "/speed_y") {
				cfg_speeds_[Y] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_Y_NEW_SPEED, cfg_speeds_[Y]);
				msg_has_data = true;
			} else if (opt == "/speed_z") {
				cfg_speeds_[Z] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_Z_NEW_SPEED, cfg_speeds_[Z]);
				msg_has_data = true;
			} else if (opt == "/speed_a") {
				cfg_speeds_[A] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_A_NEW_SPEED, cfg_speeds_[A]);
				msg_has_data = true;
			} else if (opt == "/acc_x") {
				cfg_accs_[X] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_X_NEW_ACC, cfg_accs_[X]);
				msg_has_data = true;
			} else if (opt == "/acc_y") {
				cfg_accs_[Y] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_Y_NEW_ACC, cfg_accs_[Y]);
				msg_has_data = true;
			} else if (opt == "/acc_z") {
				cfg_accs_[Z] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_Z_NEW_ACC, cfg_accs_[Z]);
				msg_has_data = true;
			} else if (opt == "/acc_a") {
				cfg_accs_[A] = v->get_uint();
				add_command_to_message(arduino_msg, CMD_A_NEW_ACC, cfg_accs_[A]);
				msg_has_data = true;
			}
			if (msg_has_data) {
				append_message_to_queue(arduino_msg);
				wakeup();
			} else
				delete arduino_msg;
		}
	}
}

void
ArduinoComThread::config_value_erased(const char *path)
{
}
void
ArduinoComThread::config_tag_changed(const char *new_tag)
{
}
void
ArduinoComThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v)
{
}
