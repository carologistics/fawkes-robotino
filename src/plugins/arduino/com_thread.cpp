
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
        serial_(io_service_), deadline_(io_service_),
        work_(io_service_), // this work object is necessary to prevent the io_service from stopping itself
        tf_thread_(tf_thread)
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

  open_pending_ = true;
  open_tries_ = 0;
  home_pending_ = true;
  config_check_pending_ = true;
  config_check_failed_final_ = false;
  device_status_ = DeviceStatus::SETTING_UP;





  bbil_add_message_interface(arduino_if_);

  blackboard->register_listener(this);
  arduino_if_->set_final(true);

  arduino_if_->set_status(ArduinoInterface::IDLE);

  arduino_if_->set_x_max(cfg_x_max_);
  arduino_if_->set_y_max(cfg_y_max_);
  arduino_if_->set_z_max(cfg_z_max_);

  arduino_if_->write();
  wakeup();


}

void ArduinoComThread::go_home()
{
  ArduinoComMessage msg(ArduinoComMessage::command_id_t::CMD_HOME,{});
  send_message(msg);
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
  messages_.push_back(msg);
}

void
ArduinoComThread::priorized_append_message_to_queue(ArduinoComMessage* msg)
{
  messages_.push_front(msg);
}

/* 
 * @brief Try to connect to the device until the max number of connection retries is reached.
 * @return True if successful, false if not
 */
bool
ArduinoComThread::reset_device()
{
  do{
    logger->log_info(name(), "Trying the %u of %u times to open the device.", open_tries_, cfg_max_open_tries_);
    close_device();
    open_device();
  } while(open_pending_ && open_tries_ < cfg_max_open_tries_);

  if(open_tries_ >= cfg_max_open_tries_ || open_pending_) {
    logger->log_error(name(), "Tried %u times to open device. Giving up.", cfg_max_open_tries_);
    return false; 
  }
  //Device is open now, need to check config values now

  if(config_check_failed_final_) return false;
  logger->log_info(name(),"Should check those config values now");
  std::vector<ArduinoComMessage::setting_id_t> incorrect_settings;
  if(!check_config(incorrect_settings)){
    write_config(incorrect_settings); //if some settings diverged, correct them
    if(!check_config(incorrect_settings)){ // check whether they are really correct now
      logger->log_error(name(), "After writing settings, read settings were different.\n To not break EEPROM memory, no further tries will happen. FIX IT!");
      config_check_failed_final_ = true;
      return false;
    }
  }
  flush_device();

  // device open and cfg values are checked, time to go home

  go_home();
  std::string dummy;
  if(ResponseType::OK != read_packet(dummy, 20*1000, true)) return false;

  return true;
}

void
ArduinoComThread::convert_commands()
{
    while (!arduino_if_->msgq_empty() && arduino_if_->is_final()) { //while there are open commands, create grbl messages from them
      if (arduino_if_->msgq_first_is<ArduinoInterface::MoveXYZAbsMessage>()) {
        ArduinoInterface::MoveXYZAbsMessage *msg = arduino_if_->msgq_first(msg);

/*
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
        */

        float goal_x = msg->x(); //tf_pose_target.getOrigin().x() + msg->x();
        float goal_y = msg->y(); //tf_pose_target.getOrigin().x() + msg->x();
        float goal_z = msg->z(); //tf_pose_target.getOrigin().x() + msg->x();
        //float goal_y = tf_pose_target.getOrigin().y() + msg->y();
        //float goal_z = tf_pose_target.getOrigin().z() + msg->z();


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
        arduino_msg->add_command(ArduinoComMessage::command_id_t::CMD_GOTO_LINEAR,{{'X',-goal_x},{'Y',-goal_y},{'Z',-goal_z},{'F',500}});
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
      /* } else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperAbsMessage>()) { */
        
      /*   //TODO */
      /*   ArduinoInterface::MoveGripperAbsMessage *msg = arduino_if_->msgq_first(msg); */

      /*   int new_abs_a = round_to_2nd_dec(msg->a() * A_AXIS_STEPS_PER_MM); */

      /*   // calculate millseconds needed for this movement */
      /*   //              int d = new_abs_a - gripper_pose_[A]; */
      /*   logger->log_debug(name(), "Set new gripper a: %u", new_abs_a); */
      /*   //append_message_to_queue(ArduinoComMessage::command_id_t::CMD_A_NEW_POS, new_abs_a, 10000); */


      /* } else if (arduino_if_->msgq_first_is<ArduinoInterface::MoveGripperRelMessage>()) { */
      /*   // TODO */
      /* } else if (arduino_if_->msgq_first_is<ArduinoInterface::ToHomeMessage>()) { */
      /*   home_pending_ = true; */
      /* } else if (arduino_if_->msgq_first_is<ArduinoInterface::CalibrateMessage>()) { */
      /*   calibrated_ = false; */
      /*   // TODO */
      }

      arduino_if_->msgq_pop();
    }
}

void
ArduinoComThread::loop()
{ 
  logger->log_info(name(), "Loop it");
  if(given_up_) {
    logger->log_error(name(), "Arduino plugin has given up communication");
    return;
  }
  if(open_pending_)
  {
    if(!reset_device()){
      given_up_ = true;
      return; // if resetting was not successful, give up
    }
  }
  device_status_ = DeviceStatus::IDLE;

  // device is ready to use now

  // now the fun starts
  //
  
  const unsigned int status_timer_reset = 10;
  unsigned int status_timer = status_timer_reset; //do not check status every cycle
  arduino_if_->read();
  do{
    std::string dummy;
    ResponseType reply;
    convert_commands(); // generate messages if there are any //takes no time
    if(!messages_.empty()) { // seems like there is work to do 
      arduino_if_->set_status(ArduinoInterface::MOVING);
      arduino_if_->set_final(false);
      arduino_if_->write();
      device_status_ = DeviceStatus::RUN;
    }

    bool should_continue; // false if either message queue is empty or buffer would overflow with next message
    do {
      should_continue = send_one_message();
    } while(should_continue); // send messages until either buffer would overflow or no messages left

    if(status_timer-- == 0) {
      ArduinoComMessage check_status_msg(ArduinoComMessage::fast_command_id_t::CMD_STAT_QUERY);
      send_message(check_status_msg); 
      do {
        reply = read_packet(dummy, 300, true);
        if(given_up_) return;
      } while(reply!=ResponseType::STATUS);
      status_timer = status_timer_reset;
    }

    do {
      reply = read_packet(dummy, 10, false);
      if(given_up_) return;
    } while(reply != ResponseType::NO); // read until there are no replies anymore

    usleep(10000); // wait for 10 ms
    arduino_if_->read();
  } while(!arduino_if_->msgq_empty() || device_status_!=DeviceStatus::IDLE); 
}

bool
ArduinoComThread::is_connected()
{
  return serial_.is_open();
}

void
ArduinoComThread::open_device()
{
  if (open_pending_){
    logger->log_debug(name(), "Open device");
    home_pending_ = true;
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

      ArduinoComMessage reset_msg(ArduinoComMessage::fast_command_id_t::CMD_SOFT_RESET);
      send_message(reset_msg);

      open_pending_ = !sync_with_arduino();

      if(!open_pending_) flush_device(); // clean start

    } catch (boost::system::system_error &e) {
      throw Exception("Arduino failed I/O: %s", e.what());
    }
  }
}

void
ArduinoComThread::close_device()
{
  boost::mutex::scoped_lock lock(io_mutex_);
  if(is_connected()){
    serial_.cancel();
    serial_.close();
  }
}

void
ArduinoComThread::flush_buffer(std::deque<ArduinoComMessage*> & buffer)
{
  while(!buffer.empty())
  {
    auto* cur_msg = buffer.front();
    buffer.pop_front();
    delete cur_msg;
  }
}

void
ArduinoComThread::handle_nodata_while_flushing(const boost::system::error_code &ec)
{
  // ec may be set if the timer is cancelled, i.e., updated
  if (! ec) {
    serial_.cancel();
    logger->log_info(name(), "No data received while flushing. Flushing is done");
    logger->log_debug(name(), "BufSize: %zu\n", input_buffer_.size());
    std::string s(boost::asio::buffer_cast<const char*>(input_buffer_.data()), input_buffer_.size());
    logger->log_debug(name(), "Received: %zu  %s\n", s.size(), s.c_str());
    input_buffer_.consume(input_buffer_.size());
  }
}

void
ArduinoComThread::flush_device()
{
  flush_buffer(sent_messages_);
  if (serial_.is_open()) {
    try {
      boost::system::error_code ec = boost::asio::error::would_block;
      bytes_read_ = 0;
      do {
        ec = boost::asio::error::would_block;
        bytes_read_ = 0;

        deadline_.expires_from_now(boost::posix_time::milliseconds(200));
        deadline_.async_wait(boost::bind(&ArduinoComThread::handle_nodata_while_flushing, this, boost::asio::placeholders::error));
        boost::asio::async_read(serial_, input_buffer_,
            boost::asio::transfer_at_least(1),
            (boost::lambda::var(ec) = boost::lambda::_1,
             boost::lambda::var(bytes_read_) = boost::lambda::_2));

        do {
          io_service_.poll();
          if (ec == boost::asio::error::would_block) {
            usleep(10000);
          }
        } while (ec == boost::asio::error::would_block);

        if (bytes_read_ > 0) {
          logger->log_warn(name(), "Flushing %zu bytes\n", bytes_read_);
        }

      } while (bytes_read_ > 0);
      deadline_.cancel();
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
  bool found = false;
  fawkes::Time start_time;
  fawkes::Time now;

  logger->log_debug(name(), "sync with arduino");
  do {
    std::string dummy;
    found = (ResponseType::BOOTUP == read_packet(dummy,1000));
    now = fawkes::Time();
  } while (!found && (now - start_time < 3.));

  if (!found) {
    logger->log_error(name(), "Timeout reached trying to sync with arduino");
    return false;
  } else {
    logger->log_info(name(), "Synchronization with Arduino successful");
    return true;
  }
}

bool
ArduinoComThread::send_one_message()
{
  if(messages_.empty()) return false; //if there are no elements to send

  boost::mutex::scoped_lock lock(io_mutex_);
  ArduinoComMessage *cur_msg = messages_.front();
  if(!arduino_enough_buffer(cur_msg)) return false; //if there are elements, but prevent buffer overflow

  messages_.pop_front();
  send_message(*cur_msg);
  sent_messages_.push_back(cur_msg);

  return true;
}

void
ArduinoComThread::handle_nodata(const boost::system::error_code &ec, bool expected)
{
  // ec may be set if the timer is cancelled, i.e., updated
  if (! ec) {
    serial_.cancel();
    logger->log_error(name(), "No data received for too long, re-establishing connection");
    //        printf("No data received for too long, re-establishing connection\n");
    logger->log_debug(name(), "BufSize: %zu\n", input_buffer_.size());
    std::string s(boost::asio::buffer_cast<const char*>(input_buffer_.data()), input_buffer_.size());
    logger->log_debug(name(), "Received: %zu  %s\n", s.size(), s.c_str());

    if(expected){
      open_pending_ = true; // if something was expected but did not arrive force restart
    }
  }
}

ArduinoComThread::ResponseType
ArduinoComThread::read_packet(std::string &s, unsigned int timeout /*= 100*/, bool expected /*= true*/)
{

  boost::system::error_code ec = boost::asio::error::would_block;
  bytes_read_ = 0;

  logger->log_debug(name(), "read_packet with timeout: %u", timeout);

  deadline_.expires_from_now(boost::posix_time::milliseconds(timeout));
  deadline_.async_wait(boost::bind(&ArduinoComThread::handle_nodata, this, boost::asio::placeholders::error, expected));

  boost::asio::async_read_until(serial_, input_buffer_, "\r\n",
      (boost::lambda::var(ec) = boost::lambda::_1,
       boost::lambda::var(bytes_read_) = boost::lambda::_2));

  //    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

  do {
    io_service_.poll();
    if (ec == boost::asio::error::would_block) {
      usleep(10000);
    }
  } while (ec == boost::asio::error::would_block);

  //received something, cancel the deadline
  deadline_.cancel();
  io_service_.poll();

  if (ec) {
    if (ec.value() == boost::system::errc::operation_canceled) {
      logger->log_error(name(), "Arduino read operation cancelled: %s", ec.message().c_str());
      return ResponseType::NO;
    }
  }
  if(bytes_read_ == 0) return ResponseType::NO;


  //Package received - analyze package
  s = std::string(boost::asio::buffer_cast<const char*>(input_buffer_.data()), bytes_read_);
  input_buffer_.consume(bytes_read_);

  //analyze
  
  if(s.find("ok") != std::string::npos) {
    drop_sent_message();
    return ResponseType::OK;
  }

  if(s.find("error") != std::string::npos) {
    std::string responsible_command = drop_sent_message();
    logger->log_error(name(), "Error at command: %s", responsible_command.c_str());
    unsigned int error_id;
    if(sscanf(s.c_str(),"error:%u",&error_id)){
      if(error_states.count(error_id)){
        logger->log_error(name(), "Error description: %s", error_states.at(error_id).c_str()); 
      }
    }
    return ResponseType::ERROR;
  }

  if(s.find("<") != std::string::npos) {
    logger->log_debug(name(), "Status message grbl received: %s", s.c_str());
    analyze_status(s.c_str());
    return ResponseType::STATUS;
  }

  if (s.find("Grbl ") != std::string::npos) {
    logger->log_info(name(), "Found the GRBL startup message");
    return ResponseType::BOOTUP;
  }

  if(s.find("ALARM") != std::string::npos) {
    logger->log_error(name(), "Alarm state in grbl");
    unsigned int alarm_id;
    if(sscanf(s.c_str(),"ALARM:%u",&alarm_id)){
      if(alarm_states.count(alarm_id)){
        logger->log_error(name(), "Alarm description: %s", alarm_states.at(alarm_id).c_str()); 
      }
    }
    return ResponseType::ALARM;
    //TODO: React on alarm state properly
  }

  if(s.find("$") == 0) {
    unsigned int setting_id;
    char value[10];
    sscanf(s.c_str(),"$%u=%s",&setting_id, value);
    logger->log_debug(name(), "Setting %u has value %s", setting_id, value);
    return ResponseType::SETTING;
  }

  if(s.find("[") == 0) {
    if(s.find("MSG") != std::string::npos) {
      logger->log_debug(name(), "Non queried feedback message: %s", s.c_str());
      return ResponseType::FEEDBACK_MSG;
    }
    if(s.find("GC") != std::string::npos) {
      logger->log_debug(name(), "g-code state: %s", s.c_str());
      return ResponseType::GCODE_STATE;
    }
    if(s.find("HLP") != std::string::npos) {
      logger->log_debug(name(), "help message: %s", s.c_str());
      return ResponseType::HELP;
    }
    if(s.find("G") == 1 or s.find("TLO") == 1 or s.find("PRB") == 1) {
      logger->log_debug(name(), "parameter printout: %s", s.c_str());
      return ResponseType::PARAMETER;
    }
    if(s.find("VER") != std::string::npos) {
      logger->log_debug(name(), "Version: %s", s.c_str());
      return ResponseType::VERSION;
    }
    if(s.find("echo") != std::string::npos) {
      logger->log_debug(name(), "automated line echo: %s", s.c_str());
      return ResponseType::RECHO;
    }
  }

  logger->log_debug(name(), "cannot categorize the response");
  return ResponseType::NONSENSE; // is not categorizable
}

void ArduinoComThread::load_hardcoded_config()
{
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_STEP_PORT_INVERT] = 0u;
  cfg_grbl_settings_[ArduinoComMessage::setting_id_t::SET_DIR_PORT_INVERT] = 7u;
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

    cfg_max_open_tries_
      = config->get_int(cfg_prefix_ + "max_open_tries");
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

std::string
ArduinoComThread::drop_sent_message()
{
  if(sent_messages_.empty())
  {
    logger->log_error(name(),"No sent message to drop");
    return "";
  }
  ArduinoComMessage *message_to_drop = sent_messages_.front();
  sent_messages_.pop_front();
  std::string s = message_to_drop->get_data();
  delete message_to_drop;
  return s;
}

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
    std::string setting_string;

    unsigned int read_id;
    unsigned int tries=100;
    do {
      if(read_packet(setting_string, 1000) == ResponseType::SETTING){
        sscanf(setting_string.c_str(),"$%u",&read_id);
        logger->log_info(name(), "Read the setting %u", read_id);
      }
    } while(read_id != static_cast<unsigned int>(setting.first) && --tries>0); // ignore unused settings and other bullshit on the line
    if(tries == 0) // could not get all the settings
    {
      logger->log_error(name(), "Error in reading the settings");
      return false;
    }

    ArduinoComMessage::set_val_type read_value;
    switch(setting.second) {
      case ArduinoComMessage::setting_type::SET_BOOL:
        unsigned int temp_read_bool;
        sscanf(setting_string.c_str(),"$%*u=%u",&temp_read_bool);
        read_value = static_cast<bool>(temp_read_bool);
        break;
      case ArduinoComMessage::setting_type::SET_INT:
        read_value=0u;
        sscanf(setting_string.c_str(),"$%*u=%u",&boost::get<unsigned int>(read_value));
        break;
      case ArduinoComMessage::setting_type::SET_FLOAT:
        read_value=0.0f;
        sscanf(setting_string.c_str(),"$%*u=%f",&boost::get<float>(read_value));
        break;
    }
    if(!boost::apply_visitor(is_equal_comparator(),read_value,cfg_grbl_settings_[setting.first])){
      logger->log_debug(name()
          , "Setting %u diverged: expected %s, got %s"
          ,static_cast<unsigned int>(setting.first)
          ,ArduinoComMessage::value_to_string(read_value).c_str()
          ,ArduinoComMessage::value_to_string(cfg_grbl_settings_[setting.first]).c_str());
      all_correct = false;
      incorrect_settings.push_back(setting.first);
    }
  }

  //get value for every setting
  logger->log_info(name(),"all_correct: %s", all_correct?"true":"false");
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
    usleep(10000); // do not write settings to fast, otherwise grbl will hick up
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

bool
ArduinoComThread::arduino_enough_buffer(ArduinoComMessage *msg)
{
  unsigned int complete_size = msg->get_cur_buffer_index();
  for(const auto& sent_message: sent_messages_)
  {
    complete_size += sent_message->get_cur_buffer_index();
  }
  return complete_size < 127; // this is the maximum size of the arduino serial buffer-1 (leave one for status request
}

void
ArduinoComThread::analyze_status(const char* status_string)
{
  //get status
  if(strstr(status_string,"Idle")){
    device_status_ = DeviceStatus::IDLE;
  } else if(strstr(status_string,"Run")) {
    device_status_ = DeviceStatus::RUN;
  } else if(strstr(status_string,"Hold")) {
    device_status_ = DeviceStatus::HOLD;
  } else if(strstr(status_string,"Jog")) {
    device_status_ = DeviceStatus::JOG;
  } else if(strstr(status_string,"Alarm")) {
    device_status_ = DeviceStatus::ALARM;
  } else if(strstr(status_string,"Door")) {
    device_status_ = DeviceStatus::DOOR;
  } else if(strstr(status_string,"Check")) {
    device_status_ = DeviceStatus::CHECK;
  } else if(strstr(status_string,"Home")) {
    device_status_ = DeviceStatus::HOME;
  } else if(strstr(status_string,"Sleep")) {
    device_status_ = DeviceStatus::SLEEP;
  } else {
    logger->log_error(name(),"Could not identify status from status message: %s", status_string);
  }
  switch(device_status_){
    case DeviceStatus::IDLE:
      arduino_if_->set_status(ArduinoInterface::IDLE);
      arduino_if_->set_final(true);
      break;
    default:
      arduino_if_->set_status(ArduinoInterface::MOVING);
      arduino_if_->set_final(false);
      break;
  }
  arduino_if_->write();
  //TODO: update blackboard
  
  //now get the position
  const char * position;
  if((position=strstr(status_string,"WPos:"))) {
    float pos_x,pos_y,pos_z;
    sscanf(position,"WPos:%f,%f,%f",&pos_x,&pos_y,&pos_z);
    logger->log_info(name(),"The device is at position %f, %f, %f",pos_x,pos_y,pos_z);
  } else {
    logger->log_error(name(), "Could not extract the position from status message: %s", status_string);
  }
  //TODO: update blackboard and publish new transform
}

const std::map<unsigned int, std::string> ArduinoComThread::alarm_states {
  {1,"Hard limit triggered. Machine position is likely lost due to sudden and immediate halt. Re-homing is highly recommended."},
  {2,"G-code motion target exceeds machine travel. Machine position safely retained. Alarm may be unlocked."},
  {3,"Reset while in motion. Grbl cannot guarantee position. Lost steps are likely. Re-homing is highly recommended."},
  {4,"Probe fail. The probe is not in the expected initial state before starting probe cycle, where G38.2 and G38.3 is not triggered and G38.4 and G38.5 is triggered."},
  {5,"Probe fail. Probe did not contact the workpiece within the programmed travel for G38.2 and G38.4."},
  {6,"Homing fail. Reset during active homing cycle."},
  {7,"Homing fail. Safety door was opened during active homing cycle."},
  {8,"Homing fail. Cycle failed to clear limit switch when pulling off. Try increasing pull-off setting or check wiring."},
  {9,"Homing fail. Could not find limit switch within search distance. Defined as 1.5 * max_travel on search and 5 * pulloff on locate phases."},
};

const std::map<unsigned int, std::string> ArduinoComThread::error_states {
  {1,"G-code words consist of a letter and a value. Letter was not found."},
  {2,"Numeric value format is not valid or missing an expected value."},
  {3,"Grbl '$' system command was not recognized or supported."},
  {4,"Negative value received for an expected positive value."},
  {5,"Homing cycle is not enabled via settings."},
  {6,"Minimum step pulse time must be greater than 3usec"},
  {7,"EEPROM read failed. Reset and restored to default values."},
  {8,"Grbl '$' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job."},
  {9,"G-code locked out during alarm or jog state"},
  {10,"Soft limits cannot be enabled without homing also enabled."},
  {11,"Max characters per line exceeded. Line was not processed and executed."},
  {12,"(Compile Option) Grbl '$' setting value exceeds the maximum step rate supported."},
  {13,"Safety door detected as opened and door state initiated."},
  {14,"(Grbl-Mega Only) Build info or startup line exceeded EEPROM line length limit."},
  {15,"Jog target exceeds machine travel. Command ignored."},
  {16,"Jog command with no '=' or contains prohibited g-code."},
  {17,"Laser mode requires PWM output."},
  {20,"Unsupported or invalid g-code command found in block."},
  {21,"More than one g-code command from same modal group found in block."},
  {22,"Feed rate has not yet been set or is undefined."},
  {23,"G-code command in block requires an integer value."},
  {24,"Two G-code commands that both require the use of the XYZ axis words were detected in the block."},
  {25,"A G-code word was repeated in the block."},
  {26,"A G-code command implicitly or explicitly requires XYZ axis words in the block, but none were detected."},
  {27,"N line number value is not within the valid range of 1 - 9,999,999."},
  {28,"A G-code command was sent, but is missing some required P or L value words in the line."},
  {29,"Grbl supports six work coordinate systems G54-G59. G59.1, G59.2, and G59.3 are not supported."},
  {30,"The G53 G-code command requires either a G0 seek or G1 feed motion mode to be active. A different motion was active."},
  {31,"There are unused axis words in the block and G80 motion mode cancel is active."},
  {32,"A G2 or G3 arc was commanded but there are no XYZ axis words in the selected plane to trace the arc."},
  {33,"The motion command has an invalid target. G2, G3, and G38.2 generates this error, if the arc is impossible to generate or if the probe target is the current position."},
  {34,"A G2 or G3 arc, traced with the radius definition, had a mathematical error when computing the arc geometry. Try either breaking up the arc into semi-circles or quadrants, or redefine them with the arc offset definition."},
  {35,"A G2 or G3 arc, traced with the offset definition, is missing the IJK offset word in the selected plane to trace the arc."},
  {36,"There are unused, leftover G-code words that aren't used by any command in the block."},
  {37,"The G43.1 dynamic tool length offset command cannot apply an offset to an axis other than its configured axis. The Grbl default axis is the Z-axis."},
  {38,"Tool number greater than max supported value."},
};
