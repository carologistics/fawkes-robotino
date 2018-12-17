
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
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <interfaces/ArduinoInterface.h>

#include <unistd.h>

#include <libudev.h>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/thread.hpp>

using namespace fawkes;

/** @class ArduinoComThread "com_thread.h"
 * Thread to communicate with an Arduino Uno via boost::asio
 * @author Tim Niemueller, Nicolas Limpert
 */

/** Constructor. */
ArduinoComThread::ArduinoComThread(std::string &cfg_name,
        std::string &cfg_prefix, ArduinoTFThread* tf_thread) :
        Thread("ArduinoComThread", Thread::OPMODE_WAITFORWAKEUP),
        BlackBoardInterfaceListener("ArduinoThread(%s)", cfg_name.c_str()),
        fawkes::TransformAspect(),
        serial_(io_service_), deadline_(io_service_), tf_thread_(tf_thread)
{
    data_mutex_ = new Mutex();
    cfg_prefix_ = cfg_prefix;
    cfg_name_ = cfg_name;
    set_coalesce_wakeups(false);
}

/** Destructor. */
ArduinoComThread::~ArduinoComThread()
{
}

void
ArduinoComThread::init()
{
    // -------------------------------------------------------------------------- //
    load_config();

    arduino_if_ =
            blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

    joystick_if_ =
            blackboard->open_for_reading<JoystickInterface>("Joystick", cfg_ifid_joystick_.c_str());

    deadline_.expires_at(boost::posix_time::pos_infin);
    opened_ = false;

    open_device();

    open_tries_ = 0;
    movement_pending_ = false;

    // initially calibrate the gripper on startup
    calibrated_ = false;

    // move to home position on startup

    home_pending_ = true;

    set_acceleration_pending_ = false;
    msecs_to_wait_ = 0;

    bbil_add_message_interface(arduino_if_);

    blackboard->register_listener(this);
    arduino_if_->set_final(true);

    arduino_if_->set_status(ArduinoInterface::IDLE);

    arduino_if_->set_x_max(cfg_x_max_);
    arduino_if_->set_y_max(cfg_y_max_);
    arduino_if_->set_z_max(cfg_z_max_);

    arduino_if_->write();
    wakeup();

    std::vector<ArduinoComMessage::setting_id_t> incorrect_settings;
    if(!check_config(incorrect_settings))
      write_config(incorrect_settings); //if some settings diverged, correct them

    go_home();
}

void ArduinoComThread::go_home()
{
  boost::mutex::scoped_lock lock(io_mutex_);
  auto msg = new ArduinoComMessage(ArduinoComMessage::command_id_t::CMD_HOME,{});
  send_message(*msg);
  delete msg;
}

void
ArduinoComThread::finalize()
{
  // TODO: 18:05:21.526199 PluginNetworkHandler: [EXCEPTION] Thread[ArduinoComThread]::finalize() threw unsupported exception

    blackboard->unregister_listener(this);
    blackboard->close(arduino_if_);
    close_device();
}

void
ArduinoComThread::append_message_to_queue(ArduinoComMessage* msg)
{
  messages_.push(msg);
}

void
ArduinoComThread::loop()
{
  if (opened_) {
      arduino_if_->read();

        while (!arduino_if_->msgq_empty() && arduino_if_->is_final() && calibrated_) {
            if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
                ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);


                fawkes::tf::StampedTransform tf_pose_target;

                try {
                  tf_listener->lookup_transform(cfg_gripper_frame_id_, msg->target_frame(), tf_pose_target);
                } catch (fawkes::tf::ExtrapolationException &e) {
                  logger->log_debug(name(), "Extrapolation error");
                  break;
                } catch (fawkes::tf::ConnectivityException &e) {
                  logger->log_debug(name(), "Connectivity exception: %s", e.what());
                  break;
                } catch (fawkes::IllegalArgumentException &e) {
                  logger->log_debug(name(), "IllegalArgumentException exception - did you set the frame_id?: %s", e.what());
                  break;
                } catch (fawkes::Exception &e) {
                  logger->log_debug(name(), "Other exception: %s", e.what());
                  break;
                }

                float goal_x = tf_pose_target.getOrigin().x() + msg->x();
                float goal_y = tf_pose_target.getOrigin().y() + msg->y();
                float goal_z = tf_pose_target.getOrigin().z() + msg->z();


                if (goal_x >= 0. && goal_x < cfg_x_max_) {
                  logger->log_debug(name(), "Set new X: %f", goal_x);
                } else {
                  logger->log_error(name(), "Motion exceeds X dimension: %f, range is 0.0 to %f", goal_x,cfg_x_max_);
                  goal_x = std::max(0.0f,std::min(goal_x,cfg_x_max_)); //Always do the best you can!
                }

                if (goal_y >= 0. && goal_y < cfg_y_max_) {
                  logger->log_debug(name(), "Set new Y: %f", goal_y);
                } else {
                  logger->log_error(name(), "Motion exceeds Y dimension: %f, range is 0.0 to %f", goal_y,cfg_y_max_);
                  goal_y = std::max(0.0f,std::min(goal_y,cfg_y_max_)); //Always do the best you can!
                }
                if (goal_z >= 0. && goal_z < cfg_z_max_) {
                  logger->log_debug(name(), "Set new Z: %f", goal_z);
                } else {
                  logger->log_error(name(), "Motion exceeds Z dimension: %f, range is 0.0 to %f", goal_z,cfg_z_max_);
                  goal_z = std::max(0.0f,std::min(goal_z,cfg_z_max_)); //Always do the best you can!
                }

                auto arduino_msg = new ArduinoComMessage();
                arduino_msg->add_command(ArduinoComMessage::command_id_t::CMD_GOTO_LINEAR,{{'X',goal_x},{'Y',goal_y},{'Z',goal_z}});
                append_message_to_queue(arduino_msg);

            } else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZRelMessage>()) {
              /* 
                //TODO
                ArduinoInterface::MoveXYZRelMessage *msg = arduino_if_->msgq_first(msg);
                ArduinoComMessage* arduino_msg = new ArduinoComMessage();

                bool msg_has_data = false;

                float cur_x = gripper_pose_[X] / X_AXIS_STEPS_PER_MM / 1000.;
                float cur_y = gripper_pose_[Y] / Y_AXIS_STEPS_PER_MM / 1000.;
                float cur_z = gripper_pose_[Z] / Z_AXIS_STEPS_PER_MM / 1000.;
                logger->log_debug(name(), "Move rel: %f %f %f cur pose: %f %f %f", msg->x(), msg->y(), msg->z(), cur_x, cur_y, cur_z);
                if (msg->x() + cur_x >= 0. && msg->x() + cur_x < arduino_if_->x_max()) {
                  int new_abs_x = round_to_2nd_dec((msg->x() + cur_x) * X_AXIS_STEPS_PER_MM * 1000.0);
                  logger->log_debug(name(), "Set new X: %u", new_abs_x);
                  add_command_to_message(arduino_msg, ArduinoComMessage::command_id_t::CMD_X_NEW_POS, new_abs_x);

                  // calculate millseconds needed for this movement
                  int d = new_abs_x - gripper_pose_[X];
                  arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
                  msg_has_data = true;
                }
                if (msg->y() + cur_y >= 0. && msg->y() + cur_y < arduino_if_->y_max()) {
                  int new_abs_y = round_to_2nd_dec((msg->y() + cur_y) * Y_AXIS_STEPS_PER_MM * 1000.0);
                  logger->log_debug(name(), "Set new Y: %u", new_abs_y);
                  add_command_to_message(arduino_msg, ArduinoComMessage::command_id_t::CMD_Y_NEW_POS, new_abs_y);

                  // calculate millseconds needed for this movement
                  int d = new_abs_y - gripper_pose_[Y];
                  arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
                  msg_has_data = true;
                }
                if (msg->z() + cur_z >= 0. && msg->z() + cur_z < arduino_if_->z_max()) {
                  int new_abs_z = round_to_2nd_dec((msg->z() + cur_z) * Z_AXIS_STEPS_PER_MM * 1000.0);
                  logger->log_debug(name(), "Set new Z: %u", new_abs_z);
                  add_command_to_message(arduino_msg, ArduinoComMessage::command_id_t::CMD_Z_NEW_POS, new_abs_z);

                  // calculate millseconds needed for this movement
                  int d = new_abs_z - gripper_pose_[Z];
                  arduino_msg->set_msecs_if_lower(abs(d) * cfg_speed_);
                  msg_has_data = true;
                }

                if (msg_has_data == true) {
                  append_message_to_queue(arduino_msg);
                } else {
                  delete arduino_msg;
                }
                */
            } else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperAbsMessage>()) {
              /*
              //TODO
              ArduinoInterface::MoveGripperAbsMessage *msg = arduino_if_->msgq_first(msg);

              int new_abs_a = round_to_2nd_dec(msg->a() * A_AXIS_STEPS_PER_MM);

              // calculate millseconds needed for this movement
//              int d = new_abs_a - gripper_pose_[A];
              logger->log_debug(name(), "Set new gripper a: %u", new_abs_a);
              //append_message_to_queue(ArduinoComMessage::command_id_t::CMD_A_NEW_POS, new_abs_a, 10000);

              */
            } else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperRelMessage>()) {
              // TODO
            } else if (arduino_if_->msgq_first_is<ArduinoInterface::ToHomeMessage>()) {
              home_pending_ = true;
            } else if (arduino_if_->msgq_first_is<ArduinoInterface::CalibrateMessage>()) {
              calibrated_ = false;
              // TODO
            }

            arduino_if_->msgq_pop();
        }

//        joystick_if_->read();

    } else {
        try {
            open_device();
            opened_ = true;
            logger->log_info(name(), "Connection re-established after %u tries", open_tries_ + 1);
        } catch (Exception &e) {
            open_tries_ += 1;
            if (open_tries_ >= 1000) {
                logger->log_error(name(), "Connection problem to arduino. Tried 1000 reconnects - retrying...");
                open_tries_ = 0;
            }
        }
    }

    while (messages_.size() > 0) {
        arduino_if_->set_final(false);
        arduino_if_->set_status(ArduinoInterface::MOVING);
        arduino_if_->write();

        send_one_message();

        movement_pending_ = current_arduino_status_ != 'I';

        if (movement_pending_ == false) {
          // Update gripper pose in iface

          arduino_if_->set_status(ArduinoInterface::IDLE);
          arduino_if_->write();

        }
        arduino_if_->set_x_position(gripper_pose_[X] / X_AXIS_STEPS_PER_MM / 1000.);
        arduino_if_->set_y_position(gripper_pose_[Y] / Y_AXIS_STEPS_PER_MM / 1000.);
        arduino_if_->set_z_position(gripper_pose_[Z] / Z_AXIS_STEPS_PER_MM / 1000.);
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

            boost::asio::serial_port::parity PARITY(boost::asio::serial_port::parity::none);
            boost::asio::serial_port::baud_rate BAUD(115200);
            boost::asio::serial_port::character_size thecsize(boost::asio::serial_port::character_size(8U));
            boost::asio::serial_port::stop_bits STOP( boost::asio::serial_port::stop_bits::one );

            serial_.set_option(PARITY);
            serial_.set_option(BAUD);
            serial_.set_option(thecsize);
            serial_.set_option(STOP);

            {
                struct termios param;
                if (tcgetattr(serial_.native_handle(), &param) == 0) {
                    // set blocking mode, seemingly needed to make Asio work properly
                    param.c_cc[VMIN] = 1;
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
            bytes_read_ = 0;
            do {
                ec = boost::asio::error::would_block;
                bytes_read_ = 0;

                deadline_.expires_from_now(boost::posix_time::milliseconds(200));
                boost::asio::async_read(serial_, input_buffer_,
                        boost::asio::transfer_at_least(1),
                        (boost::lambda::var(ec) = boost::lambda::_1,
                        boost::lambda::var(bytes_read_) = boost::lambda::_2));

                do io_service_.run_one(); while (ec == boost::asio::error::would_block);

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
    std::string s;
    std::size_t found;
    fawkes::Time start_time;
    fawkes::Time now;

    logger->log_debug(name(), "sync with arduino");
    do {
        s = read_packet(6000);
        logger->log_debug(name(), "Read '%s'", s.c_str());
        found = s.find("Grbl");
        now = fawkes::Time();
    } while (found == std::string::npos && (now - start_time < 3.));

    if (now - start_time >= 3.) {
        logger->log_error(name(), "Timeout reached trying to sync with arduino");
        return false;
    }
    if (found == std::string::npos) {
        logger->log_error(name(), "Synchronization with Arduino failed, Bootup-message not located");
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
    if (messages_.size() > 0) {
        ArduinoComMessage* cur_msg = messages_.front();
        messages_.pop();
        msecs_to_wait_ = cur_msg->get_msecs();
        send_message(*cur_msg);

        delete cur_msg;

        std::string s = read_packet(1000); // read receipt
        logger->log_debug(name(), "Read receipt: %s", s.c_str());
        s = read_packet(msecs_to_wait_); // read
        logger->log_debug(name(), "Read status: %s", s.c_str());

        return true;
    }
    else {
        return false;
    }
}

void
ArduinoComThread::handle_nodata(const boost::system::error_code &ec)
{
    // ec may be set if the timer is cancelled, i.e., updated
    if (! ec) {
        serial_.cancel();
        logger->log_error(name(), "No data received for too long, re-establishing connection");
        //        printf("No data received for too long, re-establishing connection\n");
        logger->log_debug(name(), "BufSize: %zu\n", input_buffer_.size());
        std::string s(boost::asio::buffer_cast<const char*>(input_buffer_.data()), input_buffer_.size());
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
    bytes_read_ = 0;

    logger->log_debug(name(), "read_packet with timeout: %u", timeout);

    deadline_.expires_from_now(boost::posix_time::milliseconds(timeout));
    deadline_.async_wait(boost::bind(&ArduinoComThread::handle_nodata, this, boost::asio::placeholders::error));

    boost::asio::async_read_until(serial_, input_buffer_, "\r\n",
            (boost::lambda::var(ec) = boost::lambda::_1,
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

    //Package received - analyze package
    std::string s(boost::asio::buffer_cast<const char*>(input_buffer_.data()), bytes_read_);
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
      //TODO: setup absolute pose reporting!

      std::stringstream ss(s.substr(4));
      ss >> gripper_pose_[X] >> gripper_pose_[Y] >> gripper_pose_[Z] >> gripper_pose_[A];
    } else {
      // Probably something went wrong with communication
      current_arduino_status_ = 'E';
    }
    //    read_pending_ = false;
    return s;
}

void ArduinoComThread::load_hardcoded_config()
{
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_STEP_PORT_INVERT] = 0u;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_DIR_PORT_INVERT] = 0u;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_STEP_ENA_INV] = false;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_LIMIT_PINS_INV] = false;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_PROBE_PINS_INV] = false;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_STATUS_REPORT] = 0u;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_JUNCTION_DEV] = 0.0f;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_ARC_TOL] = 0.0f;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_REPORT_INCHES] = false;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_SOFT_LIMS] = false;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HARD_LIMS] = true;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HOME_CYCLE] = true;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HOME_DEBOUNCE] = 0u;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_MAX_SPINDLE_SPEED] = 0.0f;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_MIN_SPINDLE_SPEED] = 0.0f;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_LASER_MODE] = false;
}

void
ArduinoComThread::load_config()
{
  // TODO: allow setting of stepper velocity from config!
    try {
        logger->log_info(name(), "load_config"); 
        cfg_device_ = config->get_string(cfg_prefix_ + "/device");
        cfg_speed_ = config->get_int(cfg_prefix_ + "/speed");
        cfg_accel_ = config->get_int(cfg_prefix_ + "/accel");
        cfg_ifid_joystick_ = config->get_string(cfg_prefix_ + "/joystick_interface_id");
        cfg_gripper_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_frame_id");
        cfg_gripper_dyn_frame_id_ = config->get_string(cfg_prefix_ + "/gripper_dyn_frame_id");

        set_speed_pending_ = false;
        set_acceleration_pending_ = false;

        if(cfg_grbl_settings_.empty()) load_hardcoded_config(); //only do this once
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_PULSE_LENGTH]
          = config->get_uint(cfg_prefix_ + "/grbl_config/step_pulse");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_IDLE_DELAY]
          = config->get_uint(cfg_prefix_ + "/grbl_config/step_idle_delay");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HOME_DIR_INVERT]
          = config->get_uint(cfg_prefix_ + "/grbl_config/homing_dir_invert");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HOME_FEED]
          = config->get_float(cfg_prefix_ + "/grbl_config/homing_feed");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HOME_SEEK]
          = config->get_float(cfg_prefix_ + "/grbl_config/homing_seek");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_HOME_PULL_OFF]
          = config->get_float(cfg_prefix_ + "/grbl_config/homing_pulloff");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_X_STEPS]
          = config->get_float(cfg_prefix_ + "/grbl_config/x_steps");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Y_STEPS]
          = config->get_float(cfg_prefix_ + "/grbl_config/y_steps");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Z_STEPS]
          = config->get_float(cfg_prefix_ + "/grbl_config/z_steps");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_X_MAX_RATE]
          = config->get_float(cfg_prefix_ + "/grbl_config/x_max_rate");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Y_MAX_RATE]
          = config->get_float(cfg_prefix_ + "/grbl_config/y_max_rate");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Z_MAX_RATE]
          = config->get_float(cfg_prefix_ + "/grbl_config/z_max_rate");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_X_ACC]
          = config->get_float(cfg_prefix_ + "/grbl_config/x_acc");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Y_ACC]
          = config->get_float(cfg_prefix_ + "/grbl_config/y_acc");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Z_ACC]
          = config->get_float(cfg_prefix_ + "/grbl_config/z_acc");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_X_MAX_TRAVEL]
          = config->get_float(cfg_prefix_ + "/grbl_config/x_max");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Y_MAX_TRAVEL]
          = config->get_float(cfg_prefix_ + "/grbl_config/y_max");
        cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Z_MAX_TRAVEL]
          = config->get_float(cfg_prefix_ + "/grbl_config/z_max");
        //now fill the easy access copies
        cfg_x_max_ = boost::get<float>(cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_X_MAX_TRAVEL]);
        cfg_y_max_ = boost::get<float>(cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Y_MAX_TRAVEL]);
        cfg_z_max_ = boost::get<float>(cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_Z_MAX_TRAVEL]);
    } catch (Exception &e) {
    }
}

/**
 * @brief boost Visitor to compare two setting_values
 */
class is_equal_comparator : public boost::static_visitor<bool>
{
  public:
    /**
     * Visitor function for uneven types
     * @param operand1 First value
     * @param operand2 Second value
     * @return Always false, as types are different
     */
    template<class T, class U>
    bool operator()(const T & operand1, const U & operand2) const
    {
      return false;
    }
    /**
     * Visitor function for even types (only bool and int)
     * @param operand1 First value
     * @param operand2 Second value
     * @return True if same, False if not
     */
    template<class T>
    bool operator()(const T & operand1, const T & operand2) const
    {
      return operand1 == operand2;
    }
    /**
     * Visitor function for float types. Since float values are only given with 3 digits from grbl,
     * values are equal if their difference is smaller than 0.0005.
     * @param operand1 First value
     * @param operand2 Second value
     * @return True if same, False if not
     */
    bool operator()(const float & operand1, const float & operand2) const
    {
      float diff = operand1-operand2;
      if(diff<0.0) diff *= -1;
      return diff < 0.0005; //settings are displayed with 3 digits
    }
};

bool 
ArduinoComThread::check_config(std::vector<ArduinoComMessage::setting_id_t>& incorrect_settings)
{
  boost::mutex::scoped_lock lock(io_mutex_);
  incorrect_settings.clear(); 
  bool all_correct = true;
  //ask for all settings, send $$
  auto msg = new ArduinoComMessage(ArduinoComMessage::command_id_t::CMD_GETSETTINGS,{});
  send_message(*msg);
  delete msg;

  for(const auto& setting: ArduinoComMessage::setting_map)
  {
    std::string setting_string = read_packet(1000);
    unsigned int read_id;
    unsigned int tries=10;
    do {
      sscanf(setting_string.c_str(),"%u",&read_id);
    } while(read_id != static_cast<unsigned int>(setting.first) && tries>0); // ignore unused settings and other bullshit on the line
    if(tries == 0) // could not get all the settings
    {
      logger->log_error(name(), "Error in reading the settings");
      return false;
    }

    ArduinoComMessage::set_val_type read_value;
    switch(setting.second) {
      case ArduinoComMessage::setting_type::SET_BOOL:
        unsigned int temp_read_bool;
        sscanf(setting_string.c_str(),"%*u=%u",&temp_read_bool);
        read_value = static_cast<bool>(temp_read_bool);
        break;
      case ArduinoComMessage::setting_type::SET_INT:
        read_value=0u;
        sscanf(setting_string.c_str(),"%*u=%u",&boost::get<unsigned int>(read_value));
        break;
      case ArduinoComMessage::setting_type::SET_FLOAT:
        read_value=0.0f;
        sscanf(setting_string.c_str(),"%*u=%f",&boost::get<float>(read_value));
        break;
    }
    if(boost::apply_visitor(is_equal_comparator(),read_value,cfg_grbl_settings_[setting.first])){
      logger->log_debug(name(), "Setting %u diverged: expected %s, got %s",static_cast<unsigned int>(setting.first),ArduinoComMessage::value_to_string(read_value).c_str(),ArduinoComMessage::value_to_string(cfg_grbl_settings_[setting.first]).c_str());
      all_correct = false;
      incorrect_settings.push_back(setting.first);
    }
  }

  //get value for every setting
  return all_correct;
}


bool 
ArduinoComThread::write_config(const std::vector<ArduinoComMessage::setting_id_t>& incorrect_settings)
{ 
  boost::mutex::scoped_lock lock(io_mutex_);
  for(const auto& incorrect_setting:incorrect_settings)
  {
    auto msg = new ArduinoComMessage(incorrect_setting, cfg_grbl_settings_[incorrect_setting]);
    send_message(*msg);
    delete msg;
    //check for ok?
  }
  return true;
}

bool
ArduinoComThread::bb_interface_message_received(Interface *interface,
        Message *message) throw()
{
    wakeup();
    return true;
}
