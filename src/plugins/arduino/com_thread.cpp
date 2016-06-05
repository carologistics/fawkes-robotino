
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
 * Thread to communicate with Arduino via OpenArduino API (v1 or v2).
 * @author Tim Niemueller
 */

/** Constructor. */
ArduinoComThread::ArduinoComThread(std::string &cfg_name,
        std::string &cfg_prefix)
: Thread("ArduinoComThread", Thread::OPMODE_WAITFORWAKEUP),
BlackBoardInterfaceListener("ArduinoThread(%s)", cfg_name.c_str()),
BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE),
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
    last_seqnum_ = 0;
    time_wait_ = new TimeWait(clock, cfg_sensor_update_cycle_time_ * 1000);

    // -------------------------------------------------------------------------- //

    load_config();
    
    arduino_if =
            blackboard->open_for_writing<ArduinoInterface>("Arduino", cfg_name_.c_str());

    deadline_.expires_at(boost::posix_time::pos_infin);

    open_device();

    opened_ = true;
    open_tries_ = 0;
    z_movement_pending = false;
    current_stepper_pos = 0;
    // read initial "IDLE"-message
    read_pending_ = true;

    blackboard->register_listener(this);
}

void
ArduinoComThread::finalize()
{
    delete time_wait_;
}

void
ArduinoComThread::loop()
{
    time_wait_->mark_start();

    if (z_movement_pending) {
        fawkes::Time now(clock);
        if (now >= time_to_stop_z_align) {
            arduino_if->set_final(false);
            z_movement_pending = false;
        }
    }

    if (opened_) {
        while (!arduino_if->msgq_empty()) {

            arduino_if->read();
            if (arduino_if->msgq_first_is<ArduinoInterface::MoveUpwardsMessage>()) {
                ArduinoInterface::MoveUpwardsMessage *msg = arduino_if->msgq_first(msg);

                ArduinoComMessage req;
                req.add_command(ArduinoComMessage::CMD_STEP_UP);
                req.set_num_steps(msg->num_mm());
                send_and_recv(req);
                arduino_if->set_z_position(arduino_if->z_position() - msg->num_mm());
                arduino_if->write();

                read_pending_ = true;

                reset_timer_for_z_alignment(msg->num_mm());

            } else if (arduino_if->msgq_first_is<ArduinoInterface::MoveDownwardsMessage>()) {
                ArduinoInterface::MoveDownwardsMessage *msg = arduino_if->msgq_first(msg);

                ArduinoComMessage req;
                req.add_command(ArduinoComMessage::CMD_STEP_DOWN);
                req.set_num_steps(msg->num_mm());
                send_and_recv(req);
                arduino_if->set_z_position(arduino_if->z_position() + msg->num_mm());

                read_pending_ = true;
                arduino_if->write();

                reset_timer_for_z_alignment(msg->num_mm());
            } else if (arduino_if->msgq_first_is<ArduinoInterface::MoveToZ0Message>()) {
                goto_zero_position();
            }
            arduino_if->msgq_pop();
        }

        if (read_pending_ || z_movement_pending) {
            read_packet();
        }

        z_movement_pending = current_arduino_status != 'I';
        arduino_if->set_final(!z_movement_pending);
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

    time_wait_->wait();
}

void
ArduinoComThread::reset_timer_for_z_alignment(unsigned int num_mm)
{
    fawkes::Time now(clock);
    float seconds_to_drive = cfg_rpm_ + seconds_per_mm * num_mm;

    time_to_stop_z_align = now + seconds_to_drive;

    arduino_if->set_final(false);
    z_movement_pending = true;
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

        serial_.set_option(PARITY);
        serial_.set_option(BAUD);
        serial_.set_option(thecsize);

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
    boost::mutex::scoped_lock lock(io_mutex_);
    boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()));
}

std::shared_ptr<ArduinoComMessage>
ArduinoComThread::send_and_recv(ArduinoComMessage &msg)
{
    boost::mutex::scoped_lock lock(io_mutex_);
    boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()));
    return NULL;
}

void
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
    }
    if (found == std::string::npos) {
        logger->log_error(name(), "Synchronization with Arduino failed, HELLO-Package not located");
    } else {
        logger->log_info(name(), "Synchronization with Arduino successful");
    }
    return;
}

std::string
ArduinoComThread::read_packet()
{
    std::string s = read_packet(10000);
    if (s.find("AT ") == std::string::npos) {
        logger->log_error(name(), "Package error - bytes read: %zu", bytes_read_);
    }
    if (bytes_read_ > 4) {
        logger->log_debug(name(), "Package received: %s:", s.c_str());
      current_arduino_status = s.at(3);
    }
    if (current_arduino_status == 'E') {
        // TODO
        logger->log_error(name(), "Arduino error: %s", s.substr(4));
    }
    read_pending_ = false;
    return s;
}

void
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
            logger->log_error(name(), "Arduino read operation cancelled: %s", ec.message());
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

        // 2mm / rotation
        seconds_per_mm = (2. / cfg_rpm_) / 60.;
    } catch (Exception &e) {
    }
}
void
ArduinoComThread::goto_zero_position()
{
    ArduinoComMessage req;
    req.add_command(ArduinoComMessage::CMD_TO_Z_0);
    send_and_recv(req);
    arduino_if->set_z_position(0);
    arduino_if->write();
}
