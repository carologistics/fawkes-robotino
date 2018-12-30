/***************************************************************************
 *  com_message.cpp - Message for ArduinoThread
 *
 *  Created: Mon Apr 04 15:48:52 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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

#include "com_message.h"

#include <core/exception.h>
#include <iomanip>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>

using namespace fawkes;

/** @class ArduinoComMessage "com_message.h"
 * Arduino communication message.
 *
 * This object is used to create messages to be read by the Arduino
 * flashed with fawkes_plugin_comm.ino.
 *
 * This object is used to create messages to send and parse messages
 * to read. It is designed to be generic, i.e., it provides methods to
 * add messages and its fields and to iterate over commands and read
 * fields. These methods must be called in proper sequence, no command
 * type specific processing is performed within the class. This
 * approach was chosen since we do not strive for a user-facing
 * generic transport, but rather for a minimal method to interact with
 * Arduino's microcontroller.
 *
 * A message strictly differentiates between a reading and a writing
 * mode, depending on the constructor with which it was created. The
 * reading mode is used to parse received messages and provide access
 * to its commands, the writing mode to construct messages to send.
 * Furthermore, the message assumes quick run-throughs, i.e., after
 * requesting the buffer of a writing message once, it is fixed and
 * will not be re-generated after further additions.
 *
 * Terminology is a mix in part due to the naming in OpenArduino:
 * - Message: our representation of a message to send
 * - Command: a command field within a message, this is called tag
 *            and also command in OpenArduino. We chose the latter.
 *
 * @author Tim Niemueller, Nicolas Limpert
 */

class get_type : public boost::static_visitor<ArduinoComMessage::setting_type>
{
  public:
    ArduinoComMessage::setting_type operator()(const bool & to_test) const
    {
      return ArduinoComMessage::setting_type::SET_BOOL;
    }
    ArduinoComMessage::setting_type operator()(const unsigned int & to_test) const
    {
      return ArduinoComMessage::setting_type::SET_INT;
    }
    ArduinoComMessage::setting_type operator()(const float & to_test) const
    {
      return ArduinoComMessage::setting_type::SET_FLOAT;
    }
};

/** Constructor.
 * Create empty message for writing.
 */
ArduinoComMessage::ArduinoComMessage()
{
    ctor();
}

/** Constructor for initial command.
 * Create message for writing and add command for given message ID.
 * @param cmdid message ID of command to add
 * @param coordinates the coordinates to the added command
 */
ArduinoComMessage::ArduinoComMessage(command_id_t cmdid, const std::map<char, float>& coordinates)
{
  ctor();
  add_command(cmdid,  coordinates);
}

/** Constructor for a fast command message.
 * Create message for writing and add command for given message ID.
 * @param fastcmdid message ID of command to add
 */
ArduinoComMessage::ArduinoComMessage(fast_command_id_t fastcmdid)
{
  ctor();
  add_fast_command(fastcmdid);
}

/** Constructor for setting message.
 * Create message for writing and add setting
 * @param setting setting to set
 * @param value the value to set for this setting
 */
ArduinoComMessage::ArduinoComMessage(setting_id_t setting, set_val_type value)
{
  ctor();
  add_setting(setting,  value);
}

/** Destructor.
 */
ArduinoComMessage::~ArduinoComMessage()
{
  dtor();
}

void
ArduinoComMessage::dtor()
{
  free(data_);
}

void
ArduinoComMessage::ctor()
{
    // always allocate 128 bytes, increase if necessary
    data_size_ = 128;
    data_ = (char *) malloc(data_size_);

    // init buffer with zeros
    memset(data_, 0, data_size_);

    // start first command directly at start
    cur_buffer_index_ = 0;

    // setup a minimum of 1 second to wait
    msecs_to_wait_ = 1000;
}

/** Add a command.
 * Given the command and its appropriate value, it will add it as a sequence as long as it is
 * a valid command (in terms of a key in command_map). This method will return false
 * when the command is invalid or another command with the same id was previously added.
 * @param cmd command to add - see command_id_t and command_map for reference
 * @param coordinates the coordinates to the added command.
 * @return true if command was successfully added
 */
bool
ArduinoComMessage::add_command(command_id_t cmd, const std::map<char, float>& coordinates)
{
  bool valid_command = false;

  if (command_map.count(cmd)>0) // The command key exists
  {
    valid_command = true;
  }

  if (valid_command == true)
  {
    if(cur_buffer_index_>0) // allow only one command in a message
    {
      valid_command = false;
    }
  }

  if (valid_command == true)
  {
    // check whether we're exceeding the data_size_
    int length_cmd = command_map.at(cmd).length();
    ++length_cmd;  //space before coordinates
    for(const auto& coordinate: coordinates)
    {
      length_cmd += 1 + num_digits(coordinate.second) + 1; //coordinate specifier, coordinate value, space
    }
    --length_cmd; // remove the last space
    length_cmd +=2; // for \r\n
    valid_command &= cur_buffer_index_ + length_cmd < data_size_ - 1;
  }

  if (valid_command == false)
  {
    return false;
  }

  std::strncpy(data_+cur_buffer_index_,command_map.at(cmd).c_str(),command_map.at(cmd).length());
  cur_buffer_index_ += command_map.at(cmd).length();

  data_[cur_buffer_index_++] = ' ';

  for(const auto& coordinate: coordinates)
  {
    data_[cur_buffer_index_++] = coordinate.first;
    std::string coordinate_value = value_to_string(coordinate.second);
    std::strncpy(data_+cur_buffer_index_,coordinate_value.c_str(),coordinate_value.length()); //can also use sprintf
    cur_buffer_index_ += coordinate_value.length();
    data_[cur_buffer_index_++] = ' ';
  }

  data_[cur_buffer_index_-1] = '\r';  //replace the last space
  data_[cur_buffer_index_++] = '\n';

  return true;
}

/** Add a fast command.
 * Given the fast command, it will add it as a sequence.
 * @param fastcmd command to add - see fast_command_id_t for reference
 * @return true if command was successfully added
 */
bool
ArduinoComMessage::add_fast_command(fast_command_id_t fastcmd)
{
  // command is valid if it is of right datatype
  // command is always one char only, thus data_size_ is never exceeded
  if(cur_buffer_index_ != 0) return false; // real time command always alone

  data_[cur_buffer_index_++] = static_cast<char>(fastcmd);
  return true;
}

unsigned short ArduinoComMessage::num_digits(set_val_type value)
{
  class num_digits_visitor : public boost::static_visitor<unsigned short>
  {
    public:
      unsigned short operator()(unsigned int i) const
      {
        return i > 0 ? (int) log10 ((double) i) + 1 : 1;
      }
      unsigned short operator()(float f)
      {
        return ArduinoComMessage::value_to_string(f).length();
      }
      unsigned short operator()(bool b)
      {
        return 1;
      }
  };
  return boost::apply_visitor(num_digits_visitor(),value);
}

/**
 * @brief boost visitor to transform setting value into string
 */
class value_to_string_visitor : public boost::static_visitor<std::string>
{
  public:
    /**
     * @param value The value to be transformed
     * @return The value as std::string
     */
    template<class T> std::string operator()(T& value) const
    {
      return ArduinoComMessage::value_to_string(value);
    }
};
std::string  ArduinoComMessage::value_to_string(set_val_type value)
{
  return boost::apply_visitor(value_to_string_visitor(),value);
}

/** Add a configuration setting.
 * Given the setting identifier and its appropriate value, the setting command will be added. 
 * This method will return false when the setting is invalid.
 * @param setting the id of the setting to set
 * @param value the value of the chosen setting
 * @return true if setting was successfully added
 */
bool
ArduinoComMessage::add_setting(setting_id_t setting, set_val_type value)
{
  bool valid_setting = false;

  if (setting_map.count(setting)>0) // The setting key exists
  {
    valid_setting = true;
  }

  //test whether type of value is correct
  valid_setting &= setting_map.at(setting) == boost::apply_visitor(get_type(), value);

  if (valid_setting == true)
  {
    if(cur_buffer_index_>0) // allow only one setting in a message
    {
      valid_setting = false;
    }
  }

  if (valid_setting == true)
  {
    // check whether we're exceeding the data_size_
    int length_setting = 1+num_digits(static_cast<unsigned int>(setting))+1+num_digits(value)+2; // $, setting_id, =, value, \r\n
    valid_setting &= cur_buffer_index_ + length_setting < data_size_ - 1;
  }

  if (valid_setting == false)
  {
    return false;
  }

  data_[cur_buffer_index_++] = '$';
  cur_buffer_index_+=sprintf(data_+cur_buffer_index_,"%u",static_cast<unsigned int>(setting));
  data_[cur_buffer_index_++] = '=';

  std::string string_value = value_to_string(value);
  std::strncpy(data_+cur_buffer_index_,string_value.c_str(),string_value.length());
  cur_buffer_index_ += string_value.length();

  data_[cur_buffer_index_++] = '\r';
  data_[cur_buffer_index_++] = '\n';

  return true;
}
/** Get access to buffer for sending.
 * This implies packing. Note that after calling this methods later
 * modifications to the message will be ignored. The buffer is invalidated
 * on the destruction of the message.
 * @return buffer of escaped data.
 */
boost::asio::const_buffer
ArduinoComMessage::buffer()
{
#ifdef DEBUG
  std::cout << "Buffer: ";
//  printf("Buffer: ");
  for (size_t i = 0; i < data_size_; ++i) {
//      printf("%c", data_[i]);
    std::cout << data_[i];
  }
  std::cout << std::endl;
//  printf("\n");
#endif

  return boost::asio::buffer(data_, cur_buffer_index_);
}

/** Set the number of msecs the associated action of this
 * message is probably going to need to be executed (as
 * long as the last value is smaller than the new value).
 * @param msecs milliseconds
 */
void ArduinoComMessage::set_msecs_if_lower(unsigned int msecs)
{
  // TODO: Do we have to wait for each axis alone or just the longest running one?
  if (msecs_to_wait_ < msecs)
  {
    msecs_to_wait_ = msecs;
  }
}

/** Get the number of msecs the associated action of this
 * message is probably going to need to be executed
 * @return msecs milliseconds
 */
unsigned int ArduinoComMessage::get_msecs()
{
  return msecs_to_wait_;
}

/** Get the data size of the message
 * this is only used for error reporting
 * @return data size of the message
 */
unsigned short
ArduinoComMessage::get_data_size()
{
  return data_size_;
}

/** Get the current buffer index
 * this is only used for error reporting
 * @return current buffer index
 */
unsigned short
ArduinoComMessage::get_cur_buffer_index()
{
  return cur_buffer_index_;
}


const std::map<ArduinoComMessage::command_id_t, std::string> ArduinoComMessage::command_map {
  {command_id_t::CMD_HOME, "$H"},
  {command_id_t::CMD_GETSETTINGS, "$$"},
  {command_id_t::CMD_GOTO_LINEAR, "G01"},
};

const std::map<ArduinoComMessage::setting_id_t,ArduinoComMessage::setting_type> ArduinoComMessage::setting_map {
  {setting_id_t::SET_PULSE_LENGTH,setting_type::SET_INT}, // microseconds
  {setting_id_t::SET_IDLE_DELAY,setting_type::SET_INT},   // milliseconds
  {setting_id_t::SET_STEP_PORT_INVERT,setting_type::SET_INT}, //mask
  {setting_id_t::SET_DIR_PORT_INVERT,setting_type::SET_INT}, //mask
  {setting_id_t::SET_STEP_ENA_INV,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_LIMIT_PINS_INV,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_PROBE_PINS_INV,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_STATUS_REPORT,setting_type::SET_INT}, //mask
  {setting_id_t::SET_JUNCTION_DEV,setting_type::SET_FLOAT}, // millimeter
  {setting_id_t::SET_ARC_TOL,setting_type::SET_FLOAT}, //millimeter
  {setting_id_t::SET_REPORT_INCHES,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_SOFT_LIMS,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_HARD_LIMS,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_HOME_CYCLE,setting_type::SET_BOOL}, //bool
  {setting_id_t::SET_HOME_DIR_INVERT,setting_type::SET_INT}, // mask
  {setting_id_t::SET_HOME_FEED,setting_type::SET_FLOAT}, // mm/min
  {setting_id_t::SET_HOME_SEEK,setting_type::SET_FLOAT}, // mm/min
  {setting_id_t::SET_HOME_DEBOUNCE,setting_type::SET_INT}, // milliseconds
  {setting_id_t::SET_HOME_PULL_OFF,setting_type::SET_FLOAT}, // millimeters
  {setting_id_t::SET_MAX_SPINDLE_SPEED,setting_type::SET_FLOAT}, // RPM
  {setting_id_t::SET_MIN_SPINDLE_SPEED,setting_type::SET_FLOAT}, // RPM
  {setting_id_t::SET_LASER_MODE,setting_type::SET_BOOL}, // bool
  {setting_id_t::SET_X_STEPS,setting_type::SET_FLOAT}, // steps/mm
  {setting_id_t::SET_Y_STEPS,setting_type::SET_FLOAT}, // steps/mm
  {setting_id_t::SET_Z_STEPS,setting_type::SET_FLOAT}, // steps/mm
  {setting_id_t::SET_X_MAX_RATE,setting_type::SET_FLOAT}, // mm/min
  {setting_id_t::SET_Y_MAX_RATE,setting_type::SET_FLOAT}, // mm/min
  {setting_id_t::SET_Z_MAX_RATE,setting_type::SET_FLOAT}, // mm/min
  {setting_id_t::SET_X_ACC,setting_type::SET_FLOAT}, //mm/s^2
  {setting_id_t::SET_Y_ACC,setting_type::SET_FLOAT}, //mm/s^2
  {setting_id_t::SET_Z_ACC,setting_type::SET_FLOAT}, //mm/s^2
  {setting_id_t::SET_X_MAX_TRAVEL,setting_type::SET_FLOAT}, //millimeter
  {setting_id_t::SET_Y_MAX_TRAVEL,setting_type::SET_FLOAT}, //millimeter
  {setting_id_t::SET_Z_MAX_TRAVEL,setting_type::SET_FLOAT}, //millimeter
};
