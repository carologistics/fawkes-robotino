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

    add_command(cmdid);
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

/** Increase payload by a number of bytes.
 * This may reallocate the memory to hold the data if it exceeds the current size.
 * @param count number of bytes to extend payload by
 */
void
ArduinoComMessage::inc_payload_by(uint16_t count)
{
    cur_cmd_[1] += count;
}

/** Add a command header.
 * This only allocates the header. You must call the appropriate methods to
 * add the required data fields afterwards or the message will be
 * rejected/ignored by the Arduino.
 * @param cmdid command ID to add.
 */
void
ArduinoComMessage::add_command(command_id_t cmdid)
{
    data_[3] = 0xff & (cmdid + '0');
//    cur_cmd_ = cur_data_;
//    cur_data_ += 2;
//    inc_payload_by(2);

//    cur_cmd_[0] = 0xff & cmdid;
//    cur_cmd_[1] = 0;
    //        payload_size_ = 1;
    //        std::cout << "size: " << payload_size_ << " ";
    //        for (int i = 0; i < payload_size_; i++) {
    //            std::cout << (int) cur_cmd_[i] << " ";
    //        }
    //        std::cout << std::endl;
}

void ArduinoComMessage::set_num_steps(unsigned int num_mm)
{
    data_size_ += sprintf(data_+4,"%u", (num_mm * NUM_STEPS_PER_MM));
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
    //	pack();
    //            std::cout << std::endl;
    //            escaped_data_[0] = '5';
    //            escaped_data_[1] = '2';
    //            escaped_data_size_ = 2;
    //        std::cout << "size: " << escaped_data_size_ << " " << std::endl;
    //        for (int i = 0; i < escaped_data_size_; i++) {
    //            std::cout << (int) escaped_data_[i] << " ";
    //        }
    //	return boost::asio::buffer(escaped_data_, escaped_data_size_);
    data_[data_size_ - 1] = 'X';
    
//    std::cout << "buffer" << std::endl;
//    std::cout << "echo " << data_size_ << " : " << std::endl;
//    for (int i = 0; i < data_size_; i++) {
//       std::cout << data_[i] << ' ';
//    }
//    std::cout << std::endl;
    return boost::asio::buffer(data_, data_size_);
}