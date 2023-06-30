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

#include <interfaces/ArduinoInterface.h>

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
	return in_meter / cfg_steps_per_mm_[axis] / 1000.0;
}

void
ArduinoComThread::receive(const std::string &buf)
{
	no_data_count = 0;
	logger->log_debug(name(), "read_packet: %s", buf.c_str());

	bool is_open;
	if (!ArduinoComMessage::parse_message_from_arduino(
	      gripper_pose_, is_open, current_arduino_status_, buf)) {
		append_config_messages();
		logger->log_error(name(), "Arduino relaunched needed to be reconfigured %s", buf.c_str());
		is_homed = false;
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
		if (!is_homed) {
			if (gripper_pose_[X] != home_gripper_pose[X] || gripper_pose_[Y] != home_gripper_pose[Y]
			    || gripper_pose_[Z] != home_gripper_pose[Z]) {
				ArduinoComMessage *arduino_msg = new ArduinoComMessage();
				add_command_to_message(arduino_msg, CMD_X_NEW_POS, home_gripper_pose[X]);
				add_command_to_message(arduino_msg, CMD_Y_NEW_POS, home_gripper_pose[Y]);
				add_command_to_message(arduino_msg, CMD_Z_NEW_POS, home_gripper_pose[Z]);
				append_message_to_queue(arduino_msg);

				arduino_if_->set_status(ArduinoInterface::MOVING);
				arduino_if_->set_final(false);
			} else {
				is_homed = true;
			}
		}
		if (is_homed
		    && (gripper_pose_[X] != goal_gripper_pose[X] || gripper_pose_[Y] != goal_gripper_pose[Y]
		        || gripper_pose_[Z] != goal_gripper_pose[Z] || is_open != goal_gripper_is_open)) {
			arduino_if_->set_final(false);
			ArduinoComMessage *arduino_msg = new ArduinoComMessage();
			add_command_to_message(arduino_msg, CMD_X_NEW_POS, goal_gripper_pose[X]);
			add_command_to_message(arduino_msg, CMD_Y_NEW_POS, goal_gripper_pose[Y]);
			add_command_to_message(arduino_msg, CMD_Z_NEW_POS, goal_gripper_pose[Z]);

			if (is_open != goal_gripper_is_open) {
				if (goal_gripper_is_open) {
					add_command_to_message(arduino_msg, CMD_OPEN, 0);
				} else {
					add_command_to_message(arduino_msg, CMD_CLOSE, 0);
				}
			}

			append_message_to_queue(arduino_msg);

			arduino_if_->set_status(ArduinoInterface::MOVING);
		}
	}

	arduino_if_->set_x_position(from_arduino_units(gripper_pose_[X], X));
	arduino_if_->set_y_position(from_arduino_units(gripper_pose_[Y], Y)); // - (cfg_y_max_ / 2));
	arduino_if_->set_z_position(from_arduino_units(gripper_pose_[Z], Z));
	arduino_if_->set_gripper_closed(is_open);
	arduino_if_->write();
	tf_thread_->set_position(arduino_if_->x_position(),
	                         arduino_if_->y_position(),
	                         arduino_if_->z_position());
}

void
ArduinoComThread::init()
{
	load_config();
	arduino_if_ = blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

	initInterface();
	timer_thread = boost::thread([this]() {
		while (true) {
			timer_callback();
			wakeup();
			boost::this_thread::sleep_for(boost::chrono::seconds(1));
		}
	});

	movement_pending_ = false;

	joystick_if_ =
	  blackboard->open_for_reading<JoystickInterface>("Joystick", cfg_ifid_joystick_.c_str());

	movement_pending_ = false;

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
	timer_thread.interrupt();
	blackboard->unregister_listener(this);
	blackboard->close(arduino_if_);
	port_.reset();
}

void
ArduinoComThread::append_message_to_queue(char cmd, unsigned int value)
{
	ArduinoComMessage *msg = new ArduinoComMessage(cmd, value);
	append_message_to_queue(msg);
}

void
ArduinoComThread::append_message_to_queue(ArduinoComMessage *msg)
{
	if (std::find(*messages_.begin(), *messages_.end(), msg) != *messages_.end()) {
		//Checking if this one is already in the queue
		return;
	}
	std::scoped_lock lock(queue_mutex);
	messages_.push_back(msg);
}

bool
ArduinoComThread::add_command_to_message(ArduinoComMessage *msg, char command, unsigned int value)
{
	if (!msg->add_command(command, value)) {
		logger->log_error(name(), "Faulty command! id: %c value: %u", command, value);
		return false;
	}
	return true;
}

void
ArduinoComThread::append_config_messages()
{
	append_message_to_queue(CMD_X_NEW_ACC, cfg_accs_[X]);
	append_message_to_queue(CMD_Y_NEW_ACC, cfg_accs_[Y]);
	append_message_to_queue(CMD_Z_NEW_ACC, cfg_accs_[Z]);
	append_message_to_queue(CMD_A_NEW_ACC, cfg_accs_[A]);

	append_message_to_queue(CMD_X_NEW_SPEED, cfg_speeds_[X]);
	append_message_to_queue(CMD_Y_NEW_SPEED, cfg_speeds_[Y]);
	append_message_to_queue(CMD_Z_NEW_SPEED, cfg_speeds_[Z]);
	append_message_to_queue(CMD_A_NEW_SPEED, cfg_speeds_[A]);
	// ue(CMD_CALIBRATE, 0, 50000); //now done on arduino
	append_message_to_queue(CMD_A_SET_TOGGLE_STEPS, cfg_a_toggle_steps_);
}

void
ArduinoComThread::handle_queue()
{
	int i = 0;
	while (messages_.size() > 0 && i < 10) {
		arduino_if_->set_final(false);
		arduino_if_->set_status(ArduinoInterface::MOVING);
		arduino_if_->write();

		send_message_from_queue();

		arduino_if_->write();

		++i;
	}
}

void
ArduinoComThread::loop()
{
	if (!port_ || port_disconnected) {
		logger->log_warn(name(), "Restarting arduino\n");
		port_.reset();
		port_disconnected = false;
		try {
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
	handle_queue();
}

bool
ArduinoComThread::send_message(ArduinoComMessage &msg)
{
	if (!port_) {
		return false;
	}
	if (!port_->write(msg.buffer())) {
		logger->log_error(name(), "Error while trying to send data to Arduino!");
		return false;
	}
	usleep(100);
	return true;
}

bool
ArduinoComThread::send_message_from_queue()
{
	if (!port_) {
		return false; //Let the timer handle that
	}
	if (messages_.size() > 0) {
		std::scoped_lock   lock(queue_mutex);
		ArduinoComMessage *cur_msg = messages_.front();
		if (!send_message(*cur_msg)) {
			return false;
		}

		messages_.pop_front();
		delete cur_msg;
		return true;
	}
	return false;
}

void
ArduinoComThread::timer_callback()
{
	if (!port_)
		return;

	++no_data_count;

	if (no_data_count > 5) {
		no_data_count = 0;
		return;
	}
	append_message_to_queue(CMD_STATUS_REQ, 0);

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

	logger->log_debug(name(), "Transformed target axis values: %f,%f,%f", goal_x, goal_y, goal_z);

	bool msg_has_data = false;
	int  d            = 0;
	if (goal_x >= 0. && goal_x <= arduino_if_->x_max()) {
		int new_abs_x = round_to_2nd_dec(goal_x * cfg_steps_per_mm_[X] * 1000.0);
		logger->log_info(name(), "Set new X: %u", new_abs_x);
		add_command_to_message(arduino_msg, CMD_X_NEW_POS, new_abs_x);

		goal_gripper_pose[X] = new_abs_x;
		msg_has_data         = true;
	} else {
		logger->log_error(name(), "Motion exceeds X dimension: %f", msg->x());
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_X);
		arduino_if_->write();
	}

	if (goal_y >= 0. && goal_y <= arduino_if_->y_max()) {
		int new_abs_y = round_to_2nd_dec(goal_y * cfg_steps_per_mm_[Y] * 1000.0);
		logger->log_debug(name(), "Set new Y: %u", new_abs_y);
		add_command_to_message(arduino_msg, CMD_Y_NEW_POS, new_abs_y);

		goal_gripper_pose[Y] = new_abs_y;
		msg_has_data         = true;
	} else {
		logger->log_error(name(), "Motion exceeds Y dimension: %f", msg->y());
		arduino_if_->set_status(ArduinoInterface::ERROR_OUT_OF_RANGE_Y);
		arduino_if_->write();
	}
	if (goal_z >= 0. && goal_z <= arduino_if_->z_max()) {
		int new_abs_z = round_to_2nd_dec(goal_z * cfg_steps_per_mm_[Z] * 1000.0);
		logger->log_debug(name(), "Set new Z: %u", new_abs_z);
		add_command_to_message(arduino_msg, CMD_Z_NEW_POS, new_abs_z);

		goal_gripper_pose[Z] = new_abs_z;
		msg_has_data         = true;
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
		add_command_to_message(msg, CMD_X_NEW_POS, 0);
		add_command_to_message(msg, CMD_Y_NEW_POS, (cfg_y_max_ / 2) * cfg_steps_per_mm_[Y] * 1000);
		add_command_to_message(msg, CMD_Z_NEW_POS, 0);

		append_message_to_queue(msg);
	} else if (message->is_of_type<ArduinoInterface::CalibrateMessage>()) {
		append_message_to_queue(CMD_CALIBRATE);
		status = true;
	} else if (message->is_of_type<ArduinoInterface::CloseGripperMessage>()) {
		append_message_to_queue(CMD_CLOSE);
		goal_gripper_is_open = false;
		status               = true;
	} else if (message->is_of_type<ArduinoInterface::OpenGripperMessage>()) {
		append_message_to_queue(CMD_OPEN);
		goal_gripper_is_open = true;
		status               = true;
	} else if (message->is_of_type<ArduinoInterface::StatusUpdateMessage>()) {
		append_message_to_queue(CMD_STATUS_REQ);
		status = true;
	} else if (message->is_of_type<ArduinoInterface::StopMessage>()) {
		goal_gripper_pose[X] = gripper_pose_[X];
		goal_gripper_pose[Y] = gripper_pose_[Y];
		goal_gripper_pose[Z] = gripper_pose_[Z];
		append_message_to_queue(CMD_STOP);
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
