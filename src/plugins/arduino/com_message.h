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

#include <aspect/logging.h>

#include <boost/asio.hpp>
#include <cstdint>
#include <queue>
#include <sstream>
#include <string>

class ArduinoComMessage
{
public:
	/**
   * @brief The prefix of each message to the arduino
   */
	static const char MSG_HEAD[3];

	ArduinoComMessage();
	~ArduinoComMessage();
	ArduinoComMessage(char cmdid, unsigned int value);

	bool add_command(char cmd, unsigned int number);

	static bool parse_message_from_arduino(int (&gripperr_position)[3],
	                                       bool       &is_gripper_open,
	                                       char       &arduino_status,
	                                       std::string buffer);

	std::string buffer() const;

	bool
	operator==(const ArduinoComMessage *q)
	{
		return q->buffer() == buffer();
	}

	/**
	 * @brief All variables that define the position of the gripper
	 * X,Y,Z position of the axis
	 * A position of the motor, that controls the gripper
	 */
	typedef enum { X, Y, Z, A } gripper_pose_t;

private:
	inline void ctor();
	void        dtor();

private:
	std::stringstream data_;
};

#endif
