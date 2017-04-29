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
#include <iomanip>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>

using namespace fawkes;

/// @cond INTERNAL
const char ArduinoComMessage::MSG_HEAD[] = {'A', 'T', ' '};

// 5: 0xAA + payload_size/2 ... + checksum/2
//const unsigned int ArduinoComMessage::MSG_METADATA_SIZE = 5;
/// @endcond INTERNAL

/** @class ArduinoComMessage "direct_com_message.h"
 * Arduino communication message.
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
 * @author Tim Niemueller
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
 */
ArduinoComMessage::ArduinoComMessage(command_id_t cmdid)
{
    ctor();

    set_command(cmdid);
}

/** Constructor for incoming message.
 * Create message for reading from incoming buffer.
 * @param msg the message of \p msg_size is expected to be escaped and to range from
 * the including 0xAA head byte to the checksum.
 * @param msg_size size of \p msg buffer
 */
ArduinoComMessage::ArduinoComMessage(const unsigned char *msg, size_t msg_size)
{
    ctor();
}

void
ArduinoComMessage::ctor()
{
    // always allocate 5 bytes, increase if necessary
    data_size_ = 5;
    data_ = (char *) malloc(data_size_);
    memset(data_, 0, data_size_);
    memcpy(data_, MSG_HEAD, 3);
}

/** Add a command header.
 * This only allocates the header. You must call the appropriate methods to
 * add the required data fields afterwards or the message will be
 * rejected/ignored by the Arduino.
 * @param cmdid command ID to add.
 */
void
ArduinoComMessage::set_command(command_id_t cmdid)
{
    data_[3] = 0xff & (cmdid + '0');
    current_cmd_ = cmdid;
}

void ArduinoComMessage::set_number(unsigned int number)
{
    data_size_ += sprintf(data_+4,"%u", number);
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
    data_[data_size_ - 1] = 'X';
//    }
    return boost::asio::buffer(data_, data_size_);
}

void ArduinoComMessage::set_msecs(unsigned int msecs)
{
    msecs_to_wait_ = msecs;
}

unsigned int ArduinoComMessage::get_msecs()
{
    return msecs_to_wait_;
}
