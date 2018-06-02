/***************************************************************************
 *  direct_com_message.h - Message for ArduinoThread
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

class ArduinoComMessage
{
 public:
  enum class command_id_t : char{
   CMD_CALIBRATE   = 'C',
   CMD_X_NEW_POS   = 'X',
   CMD_Y_NEW_POS   = 'Y',
   CMD_Z_NEW_POS   = 'Z',
   CMD_A_NEW_POS   = 'A'
  };

  static const char MSG_HEAD[3];

  ArduinoComMessage();
  ~ArduinoComMessage();
  ArduinoComMessage(command_id_t cmdid, unsigned int value);

  bool add_command(command_id_t cmd, unsigned int number);
  unsigned short get_data_size();
  unsigned short get_cur_buffer_index();

	boost::asio::const_buffer buffer();

  void set_msecs_if_lower(unsigned int msecs);
  unsigned int get_msecs();

  static inline unsigned short num_digits(unsigned int i)
  {
      return i > 0 ? (int) log10 ((double) i) + 1 : 1;
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

