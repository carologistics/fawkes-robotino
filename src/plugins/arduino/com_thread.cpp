
/***************************************************************************
 *  com_thread.cpp - Arduino com thread
 *
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#include "com_thread.h"

#include <baseapp/run.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <interfaces/ArduinoInterface.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/thread.hpp>
#include <libudev.h>
#include <unistd.h>

using namespace fawkes;

/** @class ArduinoComThread "com_thread.h"
 * Thread to communicate with an Arduino Uno via boost::asio
 * @author Tim Niemueller, Nicolas Limpert
 */

/** Constructor. */
ArduinoComThread::ArduinoComThread(std::string &    cfg_name,
                                   std::string &    cfg_prefix,
                                   ArduinoTFThread *tf_thread)
: Thread("ArduinoComThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("ArduinoThread(%s)", cfg_prefix.c_str()),
  fawkes::TransformAspect(),
  ConfigurationChangeHandler(cfg_prefix.c_str()),
  serial_(io_service_),
  deadline_(io_service_),
  tf_thread_(tf_thread)
{
	data_mutex_ = new Mutex();
	cfg_prefix_ = cfg_prefix;
	cfg_name_   = cfg_name;
	set_coalesce_wakeups(false);
}

/** Destructor. */
ArduinoComThread::~ArduinoComThread()
{
}

void
ArduinoComThread::init()
{
	// --------------------------------------------------------------------------
	// //
	load_config();

	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

	joystick_if_ =
	  blackboard->open_for_reading<JoystickInterface>("Joystick", cfg_ifid_joystick_.c_str());

	deadline_.expires_at(boost::posix_time::pos_infin);
	opened_ = false;

	open_device();

	open_tries_       = 0;
	movement_pending_ = false;

	// initially calibrate the gripper on startup
	calibrated_ = false;

	// move to home position on startup

	home_pending_ = true;

	set_acceleration_pending_ = false;
	new_msg_                  = false;

	bbil_add_message_interface(arduino_if_);

	blackboard->register_listener(this);
	arduino_if_->set_final(true);

	arduino_if_->set_status(ArduinoInterface::IDLE);
	arduino_if_->write();
	wakeup();
}

void
ArduinoComThread::finalize()
{
	// TODO: 18:05:21.526199 PluginNetworkHandler: [EXCEPTION]
	// Thread[ArduinoComThread]::finalize() threw unsupported exception

	blackboard->unregister_listener(this);
	blackboard->close(arduino_if_);
	close_device();
}

void
ArduinoComThread::set_message(ArduinoComMessage::command_id_t cmd,
                              unsigned int                    value,
                              unsigned int                    timeout)
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
ArduinoComThread::add_command_to_message(ArduinoComMessage *             msg,
                                         ArduinoComMessage::command_id_t command,
                                         unsigned int                    value)
{
	// TODO: Check if consistency for sending values is kept - is it always
	// unsigned int?
	if (!msg->add_command(command, value)) {
		logger->log_error(name(),
		                  "Faulty command! id: %c value: %u size: %u msg_len: %u, index: %u",
		                  static_cast<char>(command),
		                  value,
		                  msg->get_data_size(),
		                  ArduinoComMessage::num_digits(value) + 1,
		                  msg->get_cur_buffer_index());
		return false;
	}
	return true;
}

void
ArduinoComThread::loop()
{
	if (opened_) {
		arduino_if_->read();

		while (!arduino_if_->msgq_empty() && arduino_if_->is_final() && calibrated_) {
			if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
				ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);

				ArduinoComMessage *arduino_msg = new ArduinoComMessage();

				fawkes::tf::StampedTransform tf_pose_target;

				try {
					tf_listener->lookup_transform(cfg_gripper_frame_id_, msg->target_frame(), tf_pose_target);
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
					add_command_to_message(arduino_msg,
					                       ArduinoComMessage::command_id_t::CMD_X_NEW_POS,
					                       new_abs_x);

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
					add_command_to_message(arduino_msg,
					                       ArduinoComMessage::command_id_t::CMD_Y_NEW_POS,
					                       new_abs_y);

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
					add_command_to_message(arduino_msg,
					                       ArduinoComMessage::command_id_t::CMD_Z_NEW_POS,
					                       new_abs_z);

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
				ArduinoComMessage *                  arduino_msg = new ArduinoComMessage();

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
					add_command_to_message(arduino_msg,
					                       ArduinoComMessage::command_id_t::CMD_X_NEW_POS,
					                       new_abs_x);

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
					add_command_to_message(arduino_msg,
					                       ArduinoComMessage::command_id_t::CMD_Y_NEW_POS,
					                       new_abs_y);

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
					add_command_to_message(arduino_msg,
					                       ArduinoComMessage::command_id_t::CMD_Z_NEW_POS,
					                       new_abs_z);

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
				set_message(ArduinoComMessage::command_id_t::CMD_CLOSE, 0, 10000);
			} else if (arduino_if_->msgq_first_is<ArduinoInterface::OpenGripperMessage>()) {
				ArduinoInterface::OpenGripperMessage *msg = arduino_if_->msgq_first(msg);
				logger->log_debug(name(), "Open Gripper");
				set_message(ArduinoComMessage::command_id_t::CMD_OPEN, 0, 10000);
			} else if (arduino_if_->msgq_first_is<ArduinoInterface::StatusUpdateMessage>()) {
				ArduinoInterface::StatusUpdateMessage *msg = arduino_if_->msgq_first(msg);
				logger->log_debug(name(), "Request Status");
				set_message(ArduinoComMessage::command_id_t::CMD_STATUS_REQ, 0, 10000);
			}

			arduino_if_->msgq_pop();
		}

		//        joystick_if_->read();

		if (calibrated_ == true) {
			if (home_pending_ == true) {
				logger->log_info(name(), "home pending");
				ArduinoComMessage *arduino_msg = new ArduinoComMessage();

				int new_abs_x = 0;
				int new_abs_y = round_to_2nd_dec(arduino_if_->y_max() * cfg_steps_per_mm_[Y] * 1000. / 2.);
				int new_abs_z = 0;
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_X_NEW_POS,
				                       new_abs_x);
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_Y_NEW_POS,
				                       new_abs_y);
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_Z_NEW_POS,
				                       new_abs_z);

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
			set_message(ArduinoComMessage::command_id_t::CMD_X_NEW_ACC, cfg_accs_[X], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_Y_NEW_ACC, cfg_accs_[Y], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_Z_NEW_ACC, cfg_accs_[Z], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_A_NEW_ACC, cfg_accs_[A], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_X_NEW_SPEED, cfg_speeds_[X], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_Y_NEW_SPEED, cfg_speeds_[Y], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_Z_NEW_SPEED, cfg_speeds_[Z], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_A_NEW_SPEED, cfg_speeds_[A], 1000);
			set_message(ArduinoComMessage::command_id_t::CMD_CALIBRATE, 0, 50000);
			set_message(ArduinoComMessage::command_id_t::CMD_SET_A_TOGGLE_STEPS,
			            cfg_a_toggle_steps_,
			            1000);
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

	if (new_msg_) {
		arduino_if_->set_final(false);
		arduino_if_->set_status(ArduinoInterface::MOVING);
		arduino_if_->write();

		send_one_message();
	}

	if (tf_thread_->get_moving()
	    && (expected_finish_time_ - fawkes::Time() < fawkes::Time((const long int)50))) {
		//TODO: check if it works with 0, else use like 10 and 0 for read_packet
		std::string s = read_packet(50);
		logger->log_debug(name(), "Read status: %s", s.c_str());
		tf_thread_->set_moving(false);

		movement_pending_ = current_arduino_status_ != 'I';

		if (movement_pending_ == false) {
			// Update gripper pose in iface

			arduino_if_->set_status(ArduinoInterface::IDLE);
			arduino_if_->write();

			if (calibrated_ == false) {
				arduino_if_->set_x_max(cfg_x_max_);
				arduino_if_->set_y_max(cfg_y_max_);
				arduino_if_->set_z_max(cfg_z_max_);
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
}

bool
ArduinoComThread::is_connected()
{
	return serial_.is_open();
}

void
ArduinoComThread::open_device()
{
	if (!opened_) {
		logger->log_debug(name(), "Open device");
		try {
			input_buffer_.consume(input_buffer_.size());

			boost::mutex::scoped_lock lock(io_mutex_);

			serial_.open(cfg_device_);

			boost::asio::serial_port::parity         PARITY(boost::asio::serial_port::parity::none);
			boost::asio::serial_port::baud_rate      BAUD(115200);
			boost::asio::serial_port::character_size thecsize(
			  boost::asio::serial_port::character_size(8U));
			boost::asio::serial_port::stop_bits STOP(boost::asio::serial_port::stop_bits::one);

			serial_.set_option(PARITY);
			serial_.set_option(BAUD);
			serial_.set_option(thecsize);
			serial_.set_option(STOP);

			{
				struct termios param;
				if (tcgetattr(serial_.native_handle(), &param) == 0) {
					// set blocking mode, seemingly needed to make Asio work properly
					param.c_cc[VMIN]  = 1;
					param.c_cc[VTIME] = 0;
					if (tcsetattr(serial_.native_handle(), TCSANOW, &param) != 0) {
						// another reason to fail...
					}
				} // else: BANG, cannot set VMIN/VTIME, fail
			}

			opened_ = sync_with_arduino();

		} catch (boost::system::system_error &e) {
			throw Exception("Arduino failed I/O: %s", e.what());
		}
	}
}

void
ArduinoComThread::close_device()
{
	boost::mutex::scoped_lock lock(io_mutex_);
	serial_.cancel();
	serial_.close();
	opened_ = false;
}

void
ArduinoComThread::flush_device()
{
	if (serial_.is_open()) {
		try {
			boost::system::error_code ec = boost::asio::error::would_block;
			bytes_read_                  = 0;
			do {
				ec          = boost::asio::error::would_block;
				bytes_read_ = 0;

				deadline_.expires_from_now(boost::posix_time::milliseconds(200));
				boost::asio::async_read(serial_,
				                        input_buffer_,
				                        boost::asio::transfer_at_least(1),
				                        (boost::lambda::var(ec)          = boost::lambda::_1,
				                         boost::lambda::var(bytes_read_) = boost::lambda::_2));

				do
					io_service_.run_one();
				while (ec == boost::asio::error::would_block);

				if (bytes_read_ > 0) {
					logger->log_warn(name(), "Flushing %zu bytes\n", bytes_read_);
				}

			} while (bytes_read_ > 0);
			deadline_.expires_from_now(boost::posix_time::pos_infin);
		} catch (boost::system::system_error &e) {
			// ignore, just assume done, if there really is an error we'll
			// catch it later on
		}
	}
}

void
ArduinoComThread::send_message(ArduinoComMessage &msg)
{
	try {
		boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()));
	} catch (boost::system::system_error &e) {
		logger->log_error(name(), "ERROR on send message! %s", e.what());
	}
}

bool
ArduinoComThread::sync_with_arduino()
{
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
}

bool
ArduinoComThread::send_one_message()
{
	boost::mutex::scoped_lock lock(io_mutex_);
	if (new_msg_) {
		expected_finish_time_ =
		  fawkes::Time() + (const long int)next_msg_->get_msecs(); //TODO: make it work
		send_message(*next_msg_);

		std::string s = read_packet(1000); // read receipt
		logger->log_debug(name(), "Read receipt: %s", s.c_str());
		tf_thread_->set_moving(true);

		new_msg_ = false;
		return true;
	} else {
		return false;
	}
}

void
ArduinoComThread::handle_nodata(const boost::system::error_code &ec)
{
	// ec may be set if the timer is cancelled, i.e., updated
	if (!ec) {
		serial_.cancel();
		logger->log_error(name(), "No data received for too long, re-establishing connection");
		//        printf("No data received for too long, re-establishing
		//        connection\n");
		logger->log_debug(name(), "BufSize: %zu\n", input_buffer_.size());
		std::string s(boost::asio::buffer_cast<const char *>(input_buffer_.data()),
		              input_buffer_.size());
		logger->log_debug(name(), "Received: %zu  %s\n", s.size(), s.c_str());

		io_mutex_.unlock();

		close_device();
		sleep(1);
		open_device();
	}
}

std::string
ArduinoComThread::read_packet(unsigned int timeout)
{
	boost::system::error_code ec = boost::asio::error::would_block;
	bytes_read_                  = 0;

	logger->log_debug(name(), "read_packet with timeout: %u", timeout);

	deadline_.expires_from_now(boost::posix_time::milliseconds(timeout));
	deadline_.async_wait(
	  boost::bind(&ArduinoComThread::handle_nodata, this, boost::asio::placeholders::error));

	boost::asio::async_read_until(serial_,
	                              input_buffer_,
	                              "\r\n",
	                              (boost::lambda::var(ec)          = boost::lambda::_1,
	                               boost::lambda::var(bytes_read_) = boost::lambda::_2));

	//    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

	do {
		io_service_.run_one();
		if (ec == boost::asio::error::would_block) {
			usleep(10000);
		}
	} while (ec == boost::asio::error::would_block);

	if (ec) {
		if (ec.value() == boost::system::errc::operation_canceled) {
			logger->log_error(name(), "Arduino read operation cancelled: %s", ec.message().c_str());
		}
	}

	// Package received - analyze package
	std::string s(boost::asio::buffer_cast<const char *>(input_buffer_.data()), bytes_read_);
	input_buffer_.consume(bytes_read_);
	deadline_.cancel();

	if (s.find("AT ") == std::string::npos) {
		logger->log_error(name(), "Package error - bytes read: %zu, %s", bytes_read_, s.c_str());
		// TODO: after re-opening the device it fails to be used again - fix this!
		return "";
	}
	if (bytes_read_ > 4) {
		logger->log_debug(name(), "Package received: %s:", s.c_str());
		//        if (s.find("AT OK") == std::string::npos) {
		// Package is no receipt for the previous sent package
		current_arduino_status_ = s.at(3);
		//        }
	}
	if (current_arduino_status_ == 'E') {
		logger->log_error(name(), "Arduino error: %s", s.substr(4).c_str());
	} else if (current_arduino_status_ == 'I') {
		// TODO: setup absolute pose reporting!

		std::stringstream ss(s.substr(4));
		std::string       gripper_status;
		ss >> gripper_pose_[X] >> gripper_pose_[Y] >> gripper_pose_[Z] >> gripper_pose_[A]
		  >> gripper_status;
		if (gripper_status == "CLOSED") {
			arduino_if_->set_gripper_closed(true);
			arduino_if_->write();
		} else if (gripper_status == "OPEN") {
			arduino_if_->set_gripper_closed(false);
			arduino_if_->write();
		}
	} else {
		// Probably something went wrong with communication
		current_arduino_status_ = 'E';
	}
	//    read_pending_ = false;
	return s;
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

		set_speed_pending_        = false;
		set_acceleration_pending_ = false;

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
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_X_NEW_SPEED,
				                       cfg_speeds_[X]);
				msg_has_data = true;
			} else if (opt == "/speed_y") {
				cfg_speeds_[Y] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_Y_NEW_SPEED,
				                       cfg_speeds_[Y]);
				msg_has_data = true;
			} else if (opt == "/speed_z") {
				cfg_speeds_[Z] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_Z_NEW_SPEED,
				                       cfg_speeds_[Z]);
				msg_has_data = true;
			} else if (opt == "/speed_a") {
				cfg_speeds_[A] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_A_NEW_SPEED,
				                       cfg_speeds_[A]);
				msg_has_data = true;
			} else if (opt == "/acc_x") {
				cfg_accs_[X] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_X_NEW_ACC,
				                       cfg_accs_[X]);
				msg_has_data = true;
			} else if (opt == "/acc_y") {
				cfg_accs_[Y] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_Y_NEW_ACC,
				                       cfg_accs_[Y]);
				msg_has_data = true;
			} else if (opt == "/acc_z") {
				cfg_accs_[Z] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_Z_NEW_ACC,
				                       cfg_accs_[Z]);
				msg_has_data = true;
			} else if (opt == "/acc_a") {
				cfg_accs_[A] = v->get_uint();
				add_command_to_message(arduino_msg,
				                       ArduinoComMessage::command_id_t::CMD_A_NEW_ACC,
				                       cfg_accs_[A]);
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
