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

#ifndef __PLUGINS_ARDUINO_DIRECT_COM_MESSAGE_H_
#define __PLUGINS_ARDUINO_DIRECT_COM_MESSAGE_H_

#include <cstdint>
#include <boost/asio.hpp>

class ArduinoComMessage
{
 public:
	/// shared pointer to direct com message
	typedef std::shared_ptr<ArduinoComMessage> pointer;

	/// @cond INTERNAL
	typedef enum {
		CMD_NONE                =       0,
		CMD_STEP_UP		=	1,
		CMD_STEP_DOWN		=	2,
		CMD_TO_Z_0		=	3,
		CMD_SET_ACCEL		=	4,
		CMD_SET_SPEED		=	5,
		CMD_QUERY_POS		=	6,
	} command_id_t;

	typedef enum {
		READ,
		WRITE
	} mode_t;

	static const char MSG_HEAD[3];
        static const int  NUM_STEPS_PER_MM = 100;
	/// @endcond INTERNAL

	ArduinoComMessage();
	ArduinoComMessage(command_id_t cmdid);
	ArduinoComMessage(const unsigned char *msg, size_t msg_size);

	void add_command(command_id_t cmdid);
        
        void set_number(unsigned int number);

	boost::asio::const_buffer buffer();

 private:
	void ctor();
	void inc_payload_by(uint16_t count);
  
 private:
	mode_t mode_;

	char *data_;
	unsigned short data_size_;

	char *cur_cmd_;
	char *cur_data_;
        
        long steps;

};


#endif

