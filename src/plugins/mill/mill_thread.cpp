/***************************************************************************
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
 *                  2022  Matteo Tschesche
 *                  2023  Tim Wendt
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

#include "mill_thread.h"

#include "interface/interface.h"
#include "interface/message.h"
#include <interfaces/ArduinoInterface.h>
#include <stdlib.h>

using namespace fawkes;

/** @class ArduinoComThread "com_thread.h"
 * To communicate with an Arduino Uno via boost::asio, it creates a SerialPort object that spins up a boost::asio::io_service thread and handles the communication. This object will be destroyed once the communication breaks down with the Arduino. This will then trigger a reconnect in the main loop. The main loop will be triggered every second by a wakeup timer.
 * @author Tim Niemueller, Nicolas Limpert, Matteo Tschesche, Tim Wendt
 */

/** Constructor. */
MillThread::MillThread(std::string     &cfg_name,
                                   std::string     &cfg_prefix)
: Thread("MillThread", Thread::OPMODE_CONTINUOUS),
  BlackBoardInterfaceListener("MillThread(%s)", cfg_prefix.c_str()),
  fawkes::TransformAspect(),
  ConfigurationChangeHandler(cfg_prefix.c_str())
{
}

/** Destructor. */
MillThread::~MillThread()
{
}

void
MillThread::init()
{
	load_config();
	arduino_if_ = blackboard->open_for_reading<ArduinoInterface>("Arduino");
	bbil_add_data_interface(arduino_if_);
	blackboard->register_listener(this);
}

void
MillThread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	ArduinoInterface *arduino_if = dynamic_cast<ArduinoInterface*>(interface);
	if (arduino_if) {
		arduino_if->read();
		if (arduino_if->status() == ArduinoInterface::MOVING) {
			has_send = false;
		}
	}
}

void
MillThread::finalize()
{
	// blackboard->unregister_listener(this);
	// blackboard->close(arduino_if_);
}

void
MillThread::loop()
{
	if(has_send) {
		return;
	}
	if(arduino_if_->status() == ArduinoInterface::IDLE) {
		logger->log_warn(name(), "read_packet %d",	arduino_if_->status());
		ArduinoInterface::MoveXYZAbsMessage *move_msg =
			new ArduinoInterface::MoveXYZAbsMessage();
		move_msg->set_x(0.1);
		move_msg->set_y(0.03);
		move_msg->set_z(0.0);
		move_msg->set_target_frame("end_effector_home");
		arduino_if_->msgq_enqueue(move_msg);
		has_send = true;
		return;
	}
}

void
MillThread::load_config()
{
	config->add_change_handler(this);
}

void
MillThread::config_value_changed(const Configuration::ValueIterator *v)
{
}

void
MillThread::config_value_erased(const char *path)
{
}
void
MillThread::config_tag_changed(const char *new_tag)
{
}
void
MillThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v)
{
}
