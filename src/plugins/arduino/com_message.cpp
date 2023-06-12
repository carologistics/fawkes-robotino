/***************************************************************************
 *  direct_com_message.cpp - Message for ArduinoThread
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

#include "ArduinoSketch/commands.h"

#include <core/exception.h>

#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <math.h>

using namespace fawkes;

const char ArduinoComMessage::MSG_HEAD[] = {'A', 'T', ' '};

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
 * @param value message value to be passed
 */
ArduinoComMessage::ArduinoComMessage(char cmdid, unsigned int value)
{
	if (!ArduinoHelper::isValidSerialCommand(cmdid)) {
		return;
	}
	ctor();
	add_command(cmdid, value);
}

/** Destructor.
 */
ArduinoComMessage::~ArduinoComMessage()
{
	ctor();
}

inline void
ArduinoComMessage::ctor()
{
	data_ << "AT ";
}

/** Add a command.
 * Given the command and its appropriate value, it will add it as a sequence as
 * long as it is a valid command (in terms of defined in command_id_t). This
 * method will return false when the command is invalid or another command with
 * the same id was previously added.
 * @param cmd command to add - see command_id_t for reference
 * @param value the value to the added command.
 * @return true if command was successfully added
 */
bool
ArduinoComMessage::add_command(char cmd, unsigned int value)
{
	bool valid_command = ArduinoHelper::isValidSerialCommand(cmd);
	if (!valid_command) {
		return false;
	}
	data_ << cmd;
	data_ << value;
	data_ << " ";
	return true;
}

/** Get access to buffer for sending.
 * This implies packing. Note that after calling this methods later
 * modifications to the message will be ignored. The buffer is invalidated
 * on the destruction of the message.
 * @return buffer of escaped data.
 */
std::string
ArduinoComMessage::buffer()
{
#ifdef DEBUG
	std::cout << "Buffer: " << data_ << std::endl;
#endif
	// Add terminator character to the end
	return data_.str().append("+");
}


// @brief if the return is false then the arduino reconed and we need to restart everything
bool
ArduinoComMessage::parse_message_from_arduino(int (&gripperr_position)[3],
                                              bool       &is_gripper_open,
                                              char       &arduino_status,
                                              std::string buffer)
{
	std::stringstream ss;
	std::string s   = buffer;
	size_t      pos = s.find("AT ");
	if (pos == std::string::npos) {
		//Not a valid command. This command will be ignored
		return false;
	}
	ss << buffer.substr(pos + 3);

	try {
		for (int i = 0; ss >> s; ++i) {
			if (s == "HELLO") {
				return false;
				break;
			}
			if (i == 0) {
				if (s == "I" || s == "M") {
					arduino_status = s[0];
					continue;
				}
				arduino_status = 'E';
			}
			if (i == 1) {
				int x                = std::stoi(s);
				gripperr_position[X] = x;
			}
			if (i == 2) {
				int y                = std::stoi(s);
				gripperr_position[Y] = y;
			}
			if (i == 3) {
				int z                = std::stoi(s);
				gripperr_position[Z] = z;
			}
			if (i == 5) {
				if (s == "CLOSED+") {
					is_gripper_open = false;
				}
				if (s == "OPEN+") {
					is_gripper_open = true;
				}
			}
		}
		return true;
	} catch (const std::exception &e) {
	}

	//catched exeption
	return false;
}
