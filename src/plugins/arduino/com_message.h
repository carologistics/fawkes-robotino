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

class ArduinoComMessage
{
 public:
 /**
  * @brief Mapping for all possible commands, that can be send to the arduino
  */
  enum class command_id_t {
    CMD_HOME,
    CMD_GETSETTINGS,
    CMD_GOTO_LINEAR,
  };
  const std::map<command_id_t, std::string> command_map {
    {command_id_t::CMD_HOME, "$H"},
    {command_id_t::CMD_GETSETTINGS, "$$"},
    {command_id_t::CMD_GOTO_LINEAR, "G01"},
  };

  enum class setting_type {
    SET_BOOL,
    SET_INT,
    SET_FLOAT
  };

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

  const std::map<setting_id_t,setting_type> setting_map {
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


  ArduinoComMessage();
  ~ArduinoComMessage();
  ArduinoComMessage(command_id_t cmdid, const std::map<char,float>& coordinates);
  template<class T> ArduinoComMessage(setting_id_t cmdid, T value);

  bool add_command(command_id_t cmd, const std::map<char, float>& coordinates);
  template<class T> bool add_setting(setting_id_t setting, T value);

  static inline bool check_type(setting_type type, bool value)
  {
    return type==setting_type::SET_BOOL;
  }
  static inline bool check_type(setting_type type, float value)
  {
    return type==setting_type::SET_FLOAT;
  }
  static inline bool check_type(setting_type type, unsigned int value)
  {
    return type==setting_type::SET_INT;
  }

  unsigned short get_data_size();
  unsigned short get_cur_buffer_index();

	boost::asio::const_buffer buffer();

  void set_msecs_if_lower(unsigned int msecs);
  unsigned int get_msecs();

  /**
   * @brief Calculates the number of letters an boolean is represented for Grbl
   *
   * @param b The boolean of which the number of letters should be calculated
   *
   * @return The amount of letters b is represented by
   */
  static inline unsigned short num_digits(bool b)
  {
    return 1;
  }
  /**
   * @brief Calculates the number of digits a float consists of
   *
   * @param f The number of which the number of digits should be calculated
   *
   * @return The amount of digits f consists of
   */
  static unsigned short num_digits(float f)
  {
    return value_to_string(f).length();
  }
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

  static std::string value_to_string(bool b)
  {
    return b?"1":"0";
  }
  static std::string value_to_string(float f)
  {
    std::stringstream result_stream;
    result_stream << std::fixed << std::setprecision(1) << f;
    return result_stream.str();
  }
  static std::string value_to_string(unsigned int i)
  {
    return std::to_string(i);
  }


 private:
  void ctor();
  void dtor();

 private:
	char *data_;
	unsigned short data_size_;
  unsigned short cur_buffer_index_; // index of next data field

  unsigned int msecs_to_wait_;


};


#endif

