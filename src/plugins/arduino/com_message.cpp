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

#include <core/exception.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <string>

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
ArduinoComMessage::ArduinoComMessage(command_id_t cmdid, unsigned int value)
{
	ctor();
	add_command(cmdid, value);
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
	data_      = (char *)malloc(data_size_);

	// init buffer with zeros
	memset(data_, 0, data_size_);

	// append header
	memcpy(data_, MSG_HEAD, 3);

	// start first command after header
	cur_buffer_index_ = 3;

	// setup a minimum of 1 second to wait
	msecs_to_wait_ = 1000;
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
ArduinoComMessage::add_command(command_id_t cmd, unsigned int value)
{
	// TODO: Test if "home" command also works if a "value" was accidentially
	// appended!
	bool valid_command = false;
	char char_cmd      = static_cast<char>(cmd);

	if (cmd == command_id_t::CMD_CALIBRATE || cmd == command_id_t::CMD_X_NEW_POS
	    || cmd == command_id_t::CMD_Y_NEW_POS || cmd == command_id_t::CMD_Z_NEW_POS
	    || cmd == command_id_t::CMD_X_SET_MICRO_STEPPING || cmd == command_id_t::CMD_Y_SET_MICRO_STEPPING
			|| cmd == command_id_t::CMD_Z_SET_MICRO_STEPPING || cmd == command_id_t::CMD_OPEN
			|| cmd == command_id_t::CMD_A_SET_TOGGLE_STEPS || cmd == command_id_t::CMD_CLOSE
			|| cmd == command_id_t::CMD_STATUS_REQ || cmd == command_id_t::CMD_X_NEW_SPEED
			|| cmd == command_id_t::CMD_Y_NEW_SPEED || cmd == command_id_t::CMD_Z_NEW_SPEED
			|| cmd == command_id_t::CMD_A_NEW_SPEED || cmd == command_id_t::CMD_X_NEW_ACC
			|| cmd == command_id_t::CMD_Y_NEW_ACC || cmd == command_id_t::CMD_Z_NEW_ACC
			|| cmd == command_id_t::CMD_A_NEW_ACC) {
		valid_command = true;
	}

	if (valid_command == true) {
		//    std::cout << "Buffer valid?: ";
		// skip the AT header, therefore start at index 3
		for (int i = 3; i < data_size_; i++) {
			//      std::cout << (int) data_[i] << ' ';
			// cancel when the command was already set
			if (data_[i] == char_cmd) {
				valid_command = false;
				break;
			}
		}
	}
	//  std::cout << std::endl;

	// check whether we're exceeding the data_size_
	valid_command &= cur_buffer_index_ + 1 + num_digits(value) < data_size_ - 1;

	if (valid_command == false) {
		return false;
	}

	data_[cur_buffer_index_] = char_cmd;
	cur_buffer_index_++;

	cur_buffer_index_ += sprintf(data_ + cur_buffer_index_, "%u", value);
	data_[cur_buffer_index_] = ' ';
	cur_buffer_index_++;

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
	// Add terminator character to the end
	data_[data_size_ - 1] = '+';

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

	return boost::asio::buffer(data_, data_size_);
}

/** Set the number of msecs the associated action of this
 * message is probably going to need to be executed (as
 * long as the last value is smaller than the new value).
 * @param msecs milliseconds
 */
void
ArduinoComMessage::set_msecs_if_lower(unsigned int msecs)
{
	// TODO: Do we have to wait for each axis alone or just the longest running
	// one?
	if (msecs_to_wait_ < msecs) {
		msecs_to_wait_ = msecs;
	}
}

/** Get the number of msecs the associated action of this
 * message is probably going to need to be executed
 * @return msecs milliseconds
 */
unsigned int
ArduinoComMessage::get_msecs()
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
