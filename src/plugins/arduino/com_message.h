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
    //TODO yet to do
  };

  const std::map<setting_id_t,setting_type> setting_map {
    //TODO fill this
  };


  ArduinoComMessage();
  ~ArduinoComMessage();
  ArduinoComMessage(command_id_t cmdid, const std::map<char,float>& coordinates);

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

