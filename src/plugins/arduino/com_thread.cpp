
/***************************************************************************
 *  com_thread.cpp - Arduino com thread
 *
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
#include "serialport.h"

#include <baseapp/run.h>
#include <bits/types/struct_iovec.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <interfaces/ArduinoInterface.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/thread.hpp>
#include <functional>
#include <iostream>
#include <libudev.h>
#include <memory>
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
  ConfigurationChangeHandler(cfg_prefix.c_str()),
  //serial_(io_service_),
  io_service_(),
  deadline_timer(io_service_),
  tf_thread_(tf_thread)
{
	cfg_prefix_ = cfg_prefix;
	cfg_name_   = cfg_name;
}
/** Destructor. */
ArduinoComThread::~ArduinoComThread()
{
}

void
ArduinoComThread::receive(const std::string &buf)
{
	// reset_timer(
	//   boost::bind(&ArduinoComThread::handle_nodata, this, boost::asio::placeholders::error));
	logger->log_info(name(), "received %s", buf.c_str());


	logger->log_debug(name(), "read_packet: %s", buf.c_str());

	bool is_open;
	if (!ArduinoComMessage::parse_message_from_arduino(
	      gripper_pose_, is_open, current_arduino_status_, buf)) {
		logger->log_error(name(), "Arduino relaunched needed to be reconfigured %s", buf.c_str());
		append_config_messages();
		return;
	}
	std::cout << current_arduino_status_ << "nasdf " << std::endl;
	if (current_arduino_status_ == 'E') {
		logger->log_error(name(), "Arduino error: %s", buf.c_str());
	}
	if (!is_open) {
		arduino_if_->set_gripper_closed(true);
		arduino_if_->write();
	}
	if (!is_open) {
		arduino_if_->set_gripper_closed(false);
		arduino_if_->write();
	}
}

void
ArduinoComThread::init()
{
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

	initInterface();
	deadline_timer.expires_from_now(boost::posix_time::seconds(5));
	deadline_timer.async_wait(
	  [this](const boost::system::error_code &error) { timer_callback(error); });
	io_service_thread_ = std::thread([this]() { io_service_.run(); }); // io_service_.run();

	movement_pending_ = false;

	joystick_if_ =
	  blackboard->open_for_reading<JoystickInterface>("Joystick", cfg_ifid_joystick_.c_str());

	load_config();

	io_mutex_ = std::make_shared<boost::mutex>();

	movement_pending_ = false;

	// initially calibrate the gripper on startup
	// move to home position on startup
	open_device();

	home_pending_ = true;

	new_msg_ = false;

	bbil_add_message_interface(arduino_if_);

	blackboard->register_listener(this);
	arduino_if_->set_final(true);

	arduino_if_->set_status(ArduinoInterface::IDLE);
	arduino_if_->write();
	wakeup();
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
	blackboard->unregister_listener(this);
	blackboard->close(arduino_if_);
	close_device();
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
	messages_.push(msg);
}

void
ArduinoComThread::set_message(char cmd, unsigned int value, unsigned int timeout)
{
	next_msg_ = new ArduinoComMessage(cmd, value);
	next_msg_->set_msecs_if_lower(timeout);
	new_msg_ = true;
}

void
ArduinoComThread::set_message(ArduinoComMessage *msg)
{
	next_msg_ = msg;
	new_msg_  = true;
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
	append_message_to_queue(CMD_X_NEW_ACC, cfg_accs_[X], 1000);
	append_message_to_queue(CMD_Y_NEW_ACC, cfg_accs_[Y], 1000);
	append_message_to_queue(CMD_Z_NEW_ACC, cfg_accs_[Z], 1000);
	append_message_to_queue(CMD_A_NEW_ACC, cfg_accs_[A], 1000);
	append_message_to_queue(CMD_X_NEW_SPEED, cfg_speeds_[X], 1000);
	append_message_to_queue(CMD_Y_NEW_SPEED, cfg_speeds_[Y], 1000);
	append_message_to_queue(CMD_Z_NEW_SPEED, cfg_speeds_[Z], 1000);
	append_message_to_queue(CMD_A_NEW_SPEED, cfg_speeds_[A], 1000);
	// append_message_to_queue(CMD_CALIBRATE, 0, 50000);
	append_message_to_queue(CMD_A_SET_TOGGLE_STEPS, cfg_a_toggle_steps_, 1000);
	append_message_to_queue(CMD_A_SET_HALF_TOGGLE_STEPS, cfg_a_half_toggle_steps_, 1000);
}

void
ArduinoComThread::handle_queue()
{
	while (messages_.size() > 0) {
		arduino_if_->set_final(false);
		arduino_if_->set_status(ArduinoInterface::MOVING);
		arduino_if_->write();

		send_message_from_queue();

		movement_pending_ = current_arduino_status_ != 'I';

		if (movement_pending_ == false) {
			// Update gripper pose in iface

			arduino_if_->set_status(ArduinoInterface::IDLE);
			arduino_if_->write();

			if (calibrated_ == false) {
				calibrated_ = true;
				if (home_pending_ == true) {
					wakeup();
				}
			}
		}
		arduino_if_->set_x_position(gripper_pose_[X] / cfg_steps_per_mm_[X] / 1000.);
		arduino_if_->set_y_position(gripper_pose_[Y] / cfg_steps_per_mm_[Y] / 1000.);
		arduino_if_->set_z_position(gripper_pose_[Z] / cfg_steps_per_mm_[Z] / 1000.);
		arduino_if_->set_final(!movement_pending_);
		arduino_if_->write();

		tf_thread_->set_position(arduino_if_->x_position(),
		                         arduino_if_->y_position(),
		                         arduino_if_->z_position());
	}

	if (new_msg_) {
		arduino_if_->set_final(false);
		arduino_if_->set_status(ArduinoInterface::MOVING);
		arduino_if_->write();

		logger->log_debug(name(), "Send Message");
		send_one_message();
	}
}

void
ArduinoComThread::loop()
{
	/*
	do {
		if (opened_) {
			if (calibrated_ && !arduino_if_->is_final()) {
				gripper_update();
			}
			arduino_if_->read();

			while (!arduino_if_->msgq_empty() && calibrated_) {
				if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
					ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);

					ArduinoComMessage *arduino_msg = new ArduinoComMessage();

					fawkes::tf::StampedTransform tf_pose_target;

					try {
						tf_listener->lookup_transform(cfg_gripper_frame_id_,
						                              msg->target_frame(),
						                              tf_pose_target);
					} catch (fawkes::tf::ExtrapolationException &e) {
						logger->log_error(name(), "Extrapolation error");
						break;
					} catch (fawkes::tf::ConnectivityException &e) {
						logger->log_error(name(), "Connectivity exception: %s", e.what());
						break;
					} catch (fawkes::IllegalArgumentException &e) {
						logger->log_error(name(),
						                  "IllegalArgumentException exception - did you set "
						                  "the frame_id?: %s",
						                  e.what());
						break;
					} catch (fawkes::Exception &e) {
						logger->log_error(name(), "Other exception: %s", e.what());
						break;
					}
					logger->log_info(name(),
					                 "Target: %f,%f,%f in frame %s",
					                 msg->x(),
					                 msg->y(),
					                 msg->z(),
					                 msg->target_frame());

					float goal_x = tf_pose_target.getOrigin().getX() + msg->x();
					float goal_y = tf_pose_target.getOrigin().getY() + msg->y() + cfg_y_max_ / 2.;
					float goal_z = tf_pose_target.getOrigin().getZ() + msg->z();
					logger->log_debug(
					  name(), "Transformed target axis values: %f,%f,%f", goal_x, goal_y, goal_z);

					bool msg_has_data = false;
					int  d            = 0;
					if (goal_x >= 0. && goal_x <= arduino_if_->x_max()) {
						int new_abs_x = round_to_2nd_dec(goal_x * cfg_steps_per_mm_[X] * 1000.0);
						logger->log_debug(name(), "Set new X: %u", new_abs_x);
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

					if (msg_has_data == true) {
						set_message(arduino_msg);
					} else {
						delete arduino_msg;
					}

				} else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZRelMessage>()) {
					ArduinoInterface::MoveXYZRelMessage *msg         = arduino_if_->msgq_first(msg);
					ArduinoComMessage                   *arduino_msg = new ArduinoComMessage();

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
						int new_abs_x = round_to_2nd_dec((msg->x() + cur_x) * cfg_steps_per_mm_[X] * 1000.0);
						logger->log_debug(name(), "Set new X: %u", new_abs_x);
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
						int new_abs_y = round_to_2nd_dec((msg->y() + cur_y) * cfg_steps_per_mm_[Y] * 1000.0);
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
						int new_abs_z = round_to_2nd_dec((msg->z() + cur_z) * cfg_steps_per_mm_[Z] * 1000.0);
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
						set_message(arduino_msg);
					} else {
						delete arduino_msg;
					}
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperRelMessage>()) {
					// TODO
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::ToHomeMessage>()) {
					home_pending_ = true;
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::CalibrateMessage>()) {
					calibrated_ = false;
					// TODO
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::CloseGripperMessage>()) {
					ArduinoInterface::CloseGripperMessage *msg = arduino_if_->msgq_first(msg);
					logger->log_debug(name(), "Close Gripper");
					append_message_to_queue(CMD_CLOSE, 0, 10000);
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::OpenGripperMessage>()) {
					ArduinoInterface::OpenGripperMessage *msg = arduino_if_->msgq_first(msg);
					logger->log_debug(name(), "Open Gripper");
					append_message_to_queue(CMD_OPEN, 0, 10000);
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::StopMessage>()) {
					ArduinoInterface::StopMessage *msg = arduino_if_->msgq_first(msg);
					logger->log_debug(name(), "Stop Movement");
					append_message_to_queue(CMD_STOP, 0, 10000);
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::OpenHalfGripperMessage>()) {
					ArduinoInterface::OpenHalfGripperMessage *msg = arduino_if_->msgq_first(msg);
					logger->log_debug(name(), "Open Half Gripper");
					append_message_to_queue(CMD_HALF_OPEN, 0, 10000);
				} else if (arduino_if_->msgq_first_is<ArduinoInterface::StatusUpdateMessage>()) {
					ArduinoInterface::StatusUpdateMessage *msg = arduino_if_->msgq_first(msg);
					logger->log_debug(name(), "Request Status");
					append_message_to_queue(CMD_STATUS_REQ, 0, 10000);
				}

				arduino_if_->msgq_pop();
			}

			//        joystick_if_->read();

			if (calibrated_ == true) {
				if (home_pending_ == true) {
					logger->log_info(name(), "home pending");
					ArduinoComMessage *arduino_msg = new ArduinoComMessage();

					int new_abs_x = 0;
					int new_abs_y =
					  round_to_2nd_dec(arduino_if_->y_max() * cfg_steps_per_mm_[Y] * 1000. / 2.);
					int new_abs_z = 0;
					add_command_to_message(arduino_msg, CMD_X_NEW_POS, new_abs_x);
					add_command_to_message(arduino_msg, CMD_Y_NEW_POS, new_abs_y);
					add_command_to_message(arduino_msg, CMD_Z_NEW_POS, new_abs_z);

					// simply wait for 10 seconds for a timeout.
					arduino_msg->set_msecs_if_lower(50000);
					set_message(arduino_msg);
					home_pending_ = false;
				}

				tf_thread_->set_position(gripper_pose_[X] / cfg_steps_per_mm_[X] / 1000.,
				                         gripper_pose_[Y] / cfg_steps_per_mm_[Y] / 1000.,
				                         gripper_pose_[Z] / cfg_steps_per_mm_[Z] / 1000.);

			} else {
				logger->log_warn(name(), "Calibrate pending");
				// before calibration set all speeds and accs
				append_config_messages();
			}
		} else {
			try {
				open_device();
				opened_ = true;
				logger->log_info(name(), "Connection re-established after %u tries", open_tries_ + 1);
			} catch (Exception &e) {
				open_tries_ += 1;
				if (open_tries_ >= 1000) {
					logger->log_error(name(),
					                  "Connection problem to arduino. Tried 1000 "
					                  "reconnects - retrying...");
					open_tries_ = 0;
				}
			}
		}

		handle_queue();
	} while (!arduino_if_->is_final());
	*/
}

void
ArduinoComThread::timer_callback(const boost::system::error_code &ec)
{
	port_->write("AT X 1000 +");
	std::cout << "nashorn" << std::endl;
	if (!port_) {
		open_device();
	}
	deadline_timer.expires_from_now(boost::posix_time::seconds(5));
	deadline_timer.async_wait(
	  [this](const boost::system::error_code &error) { timer_callback(error); });
}

void
ArduinoComThread::open_device()
{
	if (port_) {
		port_.reset();
	}
	try {
		port_ = std::make_unique<SerialPort>(cfg_device_,
		                                     boost::bind(&ArduinoComThread::receive,
		                                                 this,
		                                                 boost::placeholders::_1),
		                                     io_mutex_);
	} catch (boost::system::error_code ec) {
		logger->log_error(name(),
		                  "Could now open PORT: %s, %s",
		                  cfg_device_.c_str(),
		                  ec.what().c_str());
	}
	append_config_messages();
	calibrated_ = false;
}

void
ArduinoComThread::close_device()
{
	port_.reset();
}

void
ArduinoComThread::send_message(ArduinoComMessage &msg)
{
	/*
	if (!port_)
		sync_with_arduino()

		  msg.get_position_data(cur_demanded_gripper_pose, cur_demanded_is_gripper_open);
	if (!port_->write(msg.buffer())) {
		logger->log_error(name(), "Error while trying to send data to Arduino!");
	}*/
}

bool
ArduinoComThread::sync_with_arduino()
{
	/*
	std::string  s;
	std::size_t  found;
	fawkes::Time start_time;
	fawkes::Time now;

	logger->log_debug(name(), "sync with arduino");
	do {
		s = read_packet(6000);
		logger->log_debug(name(), "Read '%s'", s.c_str());
		found = s.find("AT HELLO");
		now   = fawkes::Time();
	} while (found == std::string::npos && (now - start_time < 3.));

	if (now - start_time >= 3.) {
		logger->log_error(name(), "Timeout reached trying to sync with arduino");
		return false;
	}
	if (found == std::string::npos) {
		logger->log_error(name(), "Synchronization with Arduino failed, HELLO-Package not located");
		return false;
	} else {
		logger->log_info(name(), "Synchronization with Arduino successful");
		return true;
	}
	*/
	return true;
}

bool
ArduinoComThread::send_message_from_queue()
{
	/*	if (messages_.size() > 0) {
		ArduinoComMessage *cur_msg = messages_.front();
		messages_.pop();
		msecs_to_wait_ = cur_msg->get_msecs();
		send_message(*cur_msg);

		delete cur_msg;

		std::string s = read_packet(1000); // read receipt
		logger->log_debug(name(), "Read receipt: %s", s.c_str());
		s = read_packet(msecs_to_wait_); // read
		logger->log_debug(name(), "Read status: %s", s.c_str());

		return true;
	} else {
		return false;
	}
	*/
	return true;
}

bool
ArduinoComThread::send_one_message()
{
	if (new_msg_) {
		send_message(*next_msg_);
		new_msg_ = false;
		return true;
	} else {
		return false;
	}
}

void
ArduinoComThread::handle_nodata(const boost::system::error_code &ec)
{
	if (ec)
		return;

	if (!port_)
		open_device();

	++no_data_count;

	// reset_timer(
	//   boost::bind(&ArduinoComThread::handle_nodata, this, boost::asio::placeholders::error));
	//TODO add cfg
	if (no_data_count > 100) {
		close_device();
		open_device();
		no_data_count = 0;
		return;
	}
	append_message_to_queue(CMD_STATUS_REQ, 0, 1000);
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
ArduinoComThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	wakeup();
	return true;
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
				set_message(arduino_msg);
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
