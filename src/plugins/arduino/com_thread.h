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
#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <aspect/blocked_timing.h>
#include <interfaces/JoystickInterface.h>

#include <utils/time/time.h>

#include <memory>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>

class ArduinoComMessage;

namespace fawkes {
    class Mutex;
    class Clock;
    class TimeWait;

    class BatteryInterface;
    class ArduinoInterface;
}

class ArduinoComThread
: public fawkes::Thread,
public fawkes::LoggingAspect,
public fawkes::ConfigurableAspect,
public fawkes::ClockAspect,
public fawkes::BlackBoardAspect,
public fawkes::BlackBoardInterfaceListener
{
public:

    ArduinoComThread();
    ArduinoComThread(std::string &cfg_name, std::string &cfg_prefix);
    virtual ~ArduinoComThread();

    virtual void init();
    virtual void loop();
    virtual void finalize();

    virtual bool is_connected();

    // For BlackBoardInterfaceListener
    virtual bool bb_interface_message_received(fawkes::Interface *interface,
                                             fawkes::Message *message) throw();

private:
    void open_device();
    void close_device();
    void flush_device();

    bool sync_with_arduino();
    std::string read_packet(unsigned int timeout);
    void send_message(ArduinoComMessage &msg);

    void handle_nodata(const boost::system::error_code &ec);
    bool send_one_message();

    std::string cfg_device_;
    unsigned int cfg_rpm_;
    unsigned int cfg_speed_;
    unsigned int cfg_accel_;
    std::string cfg_hostname_;
    std::string cfg_prefix_;
    std::string cfg_name_;
    std::string cfg_ifid_joystick_;
    bool cfg_enable_gyro_;
    unsigned int cfg_sensor_update_cycle_time_;
    bool cfg_gripper_enabled_;
    int cfg_max_mm_;
    unsigned int cfg_init_mm_;
    bool z_movement_pending_;
    fawkes::Time time_to_stop_z_align;
    char current_arduino_status_;

    unsigned int msecs_to_wait_;

    int current_z_position_;

    size_t bytes_read_;
    bool read_pending_;
    bool set_speed_pending_;
    bool set_acceleration_pending_;
    bool move_to_z_0_pending_;
    bool init_pos_pending_;

    bool opened_;
    unsigned int open_tries_;

    std::queue<ArduinoComMessage> messages_;

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;
    boost::asio::deadline_timer deadline_;
    boost::asio::streambuf input_buffer_;
    boost::mutex io_mutex_;
    fawkes::ArduinoInterface *arduino_if_;
    fawkes::JoystickInterface *joystick_if_;

    void load_config();

protected:
    /** Mutex to protect data_. Lock whenever accessing it. */
    fawkes::Mutex *data_mutex_;

};


#endif

