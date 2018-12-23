/***************************************************************************
 *  com_message.h - Message for ArduinoThread
 *
 *  Created: Mon Apr 04 15:40:32 2016
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

#ifndef __PLUGINS_ARDUINO_COM_MESSAGE_H_
#define __PLUGINS_ARDUINO_COM_MESSAGE_H_

#include <cstdint>
#include <boost/asio.hpp>
#include <map>
#include <sstream>
#include <iomanip>

#include <boost/variant.hpp>

class ArduinoComMessage
{
 public:

 /**
  * @brief All possible commands, that can be send to the arduino
  *
  * Here only commands with enter are noted.
  */
  enum class command_id_t {
    CMD_HOME,
    CMD_GETSETTINGS,
    CMD_GOTO_LINEAR,
    //TODO: Add more commands
  };

  /**
   * @brief Real time commands
   *
   * As these are single character commands, no map is necessary
   */
  enum class fast_command_id_t : char {
    CMD_SOFT_RESET = 0x18,
    CMD_STAT_QUERY = '?',
    CMD_CYCLE_START = '~',
    CMD_FEED_HOLD = '!',
  };

  /**
   * @brief Mapping for all possible commands to its char representation
   */
  static const std::map<command_id_t, std::string> command_map;

  /**
   * @brief Data Types of the different settings of Gribl
   */
  enum class setting_type {
    SET_BOOL,
    SET_INT,
    SET_FLOAT
  };

  /**
   * @brief Typedef of the general used datatype for setting values
   */
  typedef boost::variant<bool, unsigned int, float> set_val_type;

  /**
   * @brief All possible Gribl settings
   */
  enum class setting_id_t : unsigned short {
    SET_PULSE_LENGTH=0, // microseconds
    SET_IDLE_DELAY=1,   // milliseconds
    SET_STEP_PORT_INVERT=2, //mask
    SET_DIR_PORT_INVERT=3, //mask
    SET_STEP_ENA_INV=4, //bool
    SET_LIMIT_PINS_INV=5, //bool
    SET_PROBE_PINS_INV=6, //bool
    SET_STATUS_REPORT=10, //mask
    SET_JUNCTION_DEV=11, // millimeter
    SET_ARC_TOL=12, //millimeter
    SET_REPORT_INCHES=13, //bool
    SET_SOFT_LIMS=20, //bool
    SET_HARD_LIMS=21, //bool
    SET_HOME_CYCLE=22, //bool
    SET_HOME_DIR_INVERT=23, // mask
    SET_HOME_FEED=24, // mm/min
    SET_HOME_SEEK=25, // mm/min
    SET_HOME_DEBOUNCE=26, // milliseconds
    SET_HOME_PULL_OFF=27, // millimeters
    SET_MAX_SPINDLE_SPEED=30, // RPM
    SET_MIN_SPINDLE_SPEED=31, // RPM
    SET_LASER_MODE=32, // bool
    SET_X_STEPS=100, // steps/mm
    SET_Y_STEPS=101, // steps/mm
    SET_Z_STEPS=102, // steps/mm
    SET_X_MAX_RATE=110, // mm/min
    SET_Y_MAX_RATE=111, // mm/min
    SET_Z_MAX_RATE=112, // mm/min
    SET_X_ACC=120, //mm/s^2
    SET_Y_ACC=121, //mm/s^2
    SET_Z_ACC=122, //mm/s^2
    SET_X_MAX_TRAVEL=130, //millimeter
    SET_Y_MAX_TRAVEL=131, //millimeter
    SET_Z_MAX_TRAVEL=132, //millimeter
  };

  /**
   * @brief Mapping for all possible Gribl settings to their respective data type
   */
  static const std::map<setting_id_t,setting_type> setting_map;


  ArduinoComMessage();
  ~ArduinoComMessage();
  ArduinoComMessage(command_id_t cmdid, const std::map<char,float>& coordinates);
  ArduinoComMessage(fast_command_id_t fastcmdid);
  ArduinoComMessage(setting_id_t cmdid, set_val_type value);


  bool add_command(command_id_t cmd, const std::map<char, float>& coordinates);
  bool add_fast_command(fast_command_id_t fastcmd);
  bool add_setting(setting_id_t setting, set_val_type value);

  unsigned short get_data_size();
  unsigned short get_cur_buffer_index();

	boost::asio::const_buffer buffer();

  void set_msecs_if_lower(unsigned int msecs);
  unsigned int get_msecs();

  /**
   * @brief Calculates the number of digits an integer consists of
   *
   * @param i The number of which the number of digits should be calculated
   *
   * @return The amount of digits i consists of
   */
  static inline unsigned short num_digits(unsigned int i)
  {
      return i > 0 ? (int) log10 ((double) i) + 1 : 1;
  }

  /** 
   * @brief Converts the bool into its string representation for Grbl
   *
   * @param b The boolean value to convert
   *
   * @return The string representation of the boolean
   */
  static std::string value_to_string(bool b)
  {
    return b?"1":"0";
  }

  /** 
   * @brief Converts float into its string representation for Grbl
   *
   * @param f The float value to convert
   *
   * @return The string representation of the float
   */
  static std::string value_to_string(float f)
  {
    std::stringstream result_stream;
    result_stream << std::fixed << std::setprecision(1) << f;
    return result_stream.str();
  }

  /** 
   * @brief Converts an unsigned integer into its string representation for Grbl
   *
   * @param i The integer value to convert
   *
   * @return The string representation of the unsigned integer
   */
  static std::string value_to_string(unsigned int i)
  {
    return std::to_string(i);
  }

  /**
   * @brief Converts a general settingvalue into its string representation for Grbl
   *
   * @param value The value to convert
   *
   * @return The string representation of the setting value
   */
  static std::string value_to_string(set_val_type value);



 private:
  void ctor();
  void dtor();

  unsigned short num_digits(set_val_type value);

 private:
	char *data_;
	unsigned short data_size_;
  unsigned short cur_buffer_index_; // index of next data field

  unsigned int msecs_to_wait_;
};


#endif

