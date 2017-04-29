
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
//#include <tf/types.h>

#include <interfaces/BatteryInterface.h>
#include <interfaces/ArduinoInterface.h>
//#include <interfaces/IMUInterface.h>

#include <unistd.h>

#include <libudev.h>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/thread.hpp>

using namespace fawkes;

/** @class ArduinoComThread "openarduino_com_thread.h"
 * Thread to communicate with an Arduino Uno via boost::asio
 * @author Tim Niemueller, Nicolas Limpert
 */

/** Constructor. */
ArduinoComThread::ArduinoComThread(std::string &cfg_name,
        std::string &cfg_prefix)
: Thread("ArduinoComThread", Thread::OPMODE_CONTINUOUS),
        BlackBoardInterfaceListener("ArduinoThread(%s)", cfg_name.c_str()),
        serial_(io_service_), deadline_(io_service_)
{
    data_mutex_ = new Mutex();
    new_data_ = false;
    cfg_prefix_ = cfg_prefix;
    cfg_name_ = cfg_name;
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
    
    arduino_if =
            blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

    joystick_if_ =
        blackboard->open_for_reading<JoystickInterface>("Joystick", cfg_ifid_joystick_.c_str());

    deadline_.expires_at(boost::posix_time::pos_infin);

    open_device();

    opened_ = true;
    open_tries_ = 0;
    z_movement_pending = false;
    // read initial "IDLE"-message
    read_pending_ = true;

    // Initially nullify the z-position
    move_to_z_0_pending_ = true;
    msecs_to_wait = 0;
    current_z_position_ = 0;
    init_pos_pending_ = true;

    blackboard->register_listener(this);
}

void
ArduinoComThread::finalize()
{
}

void
ArduinoComThread::loop()
{
    if (opened_) {
        while (!arduino_if->msgq_empty() && arduino_if->is_final()) {

            arduino_if->read();
            if (arduino_if->msgq_first_is<ArduinoInterface::MoveUpwardsMessage>()) {
                ArduinoInterface::MoveUpwardsMessage *msg = arduino_if->msgq_first(msg);

                if (msg->num_mm() > 0) {
                    if (current_z_position_ - (int) msg->num_mm() < 0) {
                        logger->log_error(name(), "Limit exceeded, min: %i, desired: %i", 0, current_z_position_ + msg->num_mm());
                    } else {
                           ArduinoComMessage req;
                           req.add_command(ArduinoComMessage::CMD_STEP_UP);
                           req.set_number(msg->num_mm() * ArduinoComMessage::NUM_STEPS_PER_MM);
                           msecs_to_wait = ((double) (msg->num_mm() * ArduinoComMessage::NUM_STEPS_PER_MM) / (double)cfg_speed_) * 1000. * 10.;
                           logger->log_debug(name(), "sending: %u", msg->num_mm() * ArduinoComMessage::NUM_STEPS_PER_MM);
                           send_and_recv(req);
                           read_pending_ = true;
                    }
                }

            } else if (arduino_if->msgq_first_is<ArduinoInterface::MoveDownwardsMessage>()) {
                ArduinoInterface::MoveDownwardsMessage *msg = arduino_if->msgq_first(msg);

                if (msg->num_mm() > 0) {
                    if (current_z_position_ + (int) msg->num_mm() > cfg_max_mm_) {
                        logger->log_error(name(), "Limit exceeded, max: %i, desired: %i", cfg_max_mm_, current_z_position_ + msg->num_mm());
                    } else {
                           ArduinoComMessage req;
                           req.add_command(ArduinoComMessage::CMD_STEP_DOWN);
                           req.set_number(msg->num_mm() * ArduinoComMessage::NUM_STEPS_PER_MM);
                           msecs_to_wait = ((double) (msg->num_mm() * ArduinoComMessage::NUM_STEPS_PER_MM) / (double)cfg_speed_) * 1000. * 10.;
                           logger->log_debug(name(), "sending: %u", msg->num_mm() * ArduinoComMessage::NUM_STEPS_PER_MM);
                           send_and_recv(req);
                           read_pending_ = true;
                    }
                }

            } else if (arduino_if->msgq_first_is<ArduinoInterface::MoveToZ0Message>()) {
                move_to_z_0_pending_ = true;
            }
            arduino_if->msgq_pop();
        }

        joystick_if_->read();

        if (set_acceleration_pending_) {
            ArduinoComMessage req;
            req.add_command(ArduinoComMessage::CMD_SET_ACCEL);
            req.set_number(cfg_accel_);
            send_and_recv(req);
            set_acceleration_pending_ = false;

        } else if (set_speed_pending_) {
            ArduinoComMessage req;
            req.add_command(ArduinoComMessage::CMD_SET_SPEED);
            req.set_number(cfg_speed_);
            send_and_recv(req);
            set_speed_pending_ = false;

        } else if (move_to_z_0_pending_) {
            ArduinoComMessage req;
            req.add_command(ArduinoComMessage::CMD_TO_Z_0);
            msecs_to_wait = 10000;
            send_and_recv(req);
            move_to_z_0_pending_ = false;
            read_pending_ = true;
            arduino_if->set_final(false);

        } else if (init_pos_pending_ && arduino_if->is_final()) {
            ArduinoComMessage req;
            req.add_command(ArduinoComMessage::CMD_STEP_DOWN);
            req.set_number(cfg_init_mm_ * ArduinoComMessage::NUM_STEPS_PER_MM);
            msecs_to_wait = ((double) (cfg_init_mm_ * ArduinoComMessage::NUM_STEPS_PER_MM) / (double)cfg_speed_) * 1000. * 10.;
            logger->log_debug(name(), "sending: %u", cfg_init_mm_ * ArduinoComMessage::NUM_STEPS_PER_MM);
            send_and_recv(req);
            init_pos_pending_ = false;
            read_pending_ = true;
        } else if (joystick_if_->pressed_buttons() & JoystickInterface::BUTTON_14 &&
                   arduino_if->is_final() && !init_pos_pending_) {
            ArduinoComMessage req;
            req.add_command(ArduinoComMessage::CMD_STEP_UP);
            req.set_number(2 * ArduinoComMessage::NUM_STEPS_PER_MM);
            msecs_to_wait = ((double) (2 * ArduinoComMessage::NUM_STEPS_PER_MM) / (double)cfg_speed_) * 1000. * 10.;
            logger->log_debug(name(), "sending: %u", 2 * ArduinoComMessage::NUM_STEPS_PER_MM);
            send_and_recv(req);
            arduino_if->set_final(false);
            read_pending_ = true;
        } else if (joystick_if_->pressed_buttons() & JoystickInterface::BUTTON_15 &&
                   arduino_if->is_final() && !init_pos_pending_) {
            ArduinoComMessage req;
            req.add_command(ArduinoComMessage::CMD_STEP_DOWN);
            req.set_number(2 * ArduinoComMessage::NUM_STEPS_PER_MM);
            msecs_to_wait = ((double) (2 * ArduinoComMessage::NUM_STEPS_PER_MM) / (double)cfg_speed_) * 1000. * 10.;
            logger->log_debug(name(), "sending: %u", 2 * ArduinoComMessage::NUM_STEPS_PER_MM);
            send_and_recv(req);
            arduino_if->set_final(false);
            read_pending_ = true;
        }


        if (read_pending_ || z_movement_pending) {
            read_packet();
        }

        z_movement_pending = current_arduino_status != 'I';
        arduino_if->set_final(!z_movement_pending);
        arduino_if->set_z_position(current_z_position_);
        arduino_if->write();

    } else {
        try {
            open_device();
            opened_ = true;
            logger->log_info(name(), "Connection re-established after %u tries", open_tries_ + 1);
        } catch (Exception &e) {
            open_tries_ += 1;
            if (open_tries_ >= (1000 / cfg_sensor_update_cycle_time_)) {
                logger->log_error(name(), "Connection problem to base persists");
                open_tries_ = 0;
            }
        }
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

        sync_with_arduino();

    } catch (boost::system::system_error &e) {
        throw Exception("Arduino failed I/O: %s", e.what());
    }
}

void
ArduinoComThread::close_device()
{
    boost::mutex::scoped_lock lock(io_mutex_);
    serial_.cancel();
    serial_.close();
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
    boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()));
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
        s = read_packet(3000);
        found = s.find("AT HELLO");
        now = fawkes::Time();
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

{
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
        close_device();
        open_device();
//        for (size_t i = 0; i < s.size(); ++i) {
//	    printf("%02x ", s[i]);
//        }
//        printf("\n");
    }
}

std::string
ArduinoComThread::read_packet(unsigned int timeout)
{
    boost::system::error_code ec = boost::asio::error::would_block;
    bytes_read_ = 0;

    deadline_.expires_from_now(boost::posix_time::milliseconds(timeout));
    deadline_.async_wait(boost::bind(handle_nodata, boost::asio::placeholders::error));

    boost::asio::async_read_until(serial_, input_buffer_, "\r\n",
            (boost::lambda::var(ec) = boost::lambda::_1,
            boost::lambda::var(bytes_read_) = boost::lambda::_2));

    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    if (ec) {
        if (ec.value() == boost::system::errc::operation_canceled) {
            logger->log_error(name(), "Arduino read operation cancelled: %s", ec.message().c_str());
        }
    }

    std::string s(boost::asio::buffer_cast<const char*>(input_buffer_.data()), bytes_read_);
    input_buffer_.consume(bytes_read_);
    deadline_.cancel();
    return s;
}

void
ArduinoComThread::load_config()
{
    try {
        cfg_device_ = config->get_string("/arduino/device");
        cfg_rpm_ = config->get_int("/arduino/rpm");
        cfg_speed_ = config->get_int("/arduino/speed");
        cfg_accel_ = config->get_int("/arduino/accel");
        cfg_max_mm_ = config->get_int("/arduino/max_mm");
        cfg_init_mm_ = config->get_uint("/arduino/init_mm");
        cfg_ifid_joystick_ = config->get_string("/arduino/joystick_interface_id");

        set_speed_pending_ = true;
        set_acceleration_pending_ = true;

        // 2mm / rotation
        seconds_per_mm = (2. / cfg_rpm_) / 60.;
    } catch (Exception &e) {
    }
}
