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
#include "interfaces/GcodeInterface.h"
#include <interfaces/ArduinoInterface.h>

#include <stdlib.h>
#include <sstream>

using namespace fawkes;

/** @class ArduinoComThread "mill_thread.h"
 * Sends Arduino commands based on Gcode commands received via the GcodeInterface.
 * @author Tim Wendt
 */

/** Constructor. */
MillThread::MillThread(std::string     &cfg_name,
                                   std::string     &cfg_prefix)
: Thread("MillThread", Thread::OPMODE_CONTINUOUS),
  BlackBoardInterfaceListener("MillThread(%s)", cfg_prefix.c_str()),
  fawkes::TransformAspect(),
  ConfigurationChangeHandler(cfg_prefix.c_str())
{
	cfg_name_   = cfg_name;
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
	gcode_if_ = blackboard->open_for_writing<GcodeInterface>("Gcode", cfg_name_.c_str());
	bbil_add_data_interface(arduino_if_);
	bbil_add_message_interface(gcode_if_);
	blackboard->register_listener(this);
}

static inline std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    return (a==std::string::npos ? "" : s.substr(a, b-a+1));
}

bool
MillThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	if (message->is_of_type<GcodeInterface::GcodeMessage>()) {
		GcodeInterface::GcodeMessage *gcode_msg = (GcodeInterface::GcodeMessage *)message;
		std::string command = trim(gcode_msg->command());
        for(auto &ch : command) ch = std::toupper(ch);

		logger->log_debug(name(), "Received Gcode command: %s", command.c_str());

		if (command.empty()) {
			logger->log_warn(name(), "Received empty Gcode command, ignoring.");
			return false;
		}

        // Handle mode changes
        if(command.rfind("G90", 0) == 0){
            is_absolute_ = true;
			return true;
        }
        if(command.rfind("G91", 0) == 0){
            is_absolute_ = false;
			return true;
        }

		// Handle movement commands
		if(command.rfind("G1", 0) == 0 || command.rfind("G0", 0) == 0){
            std::istringstream iss(command);
            std::string token;
            // skip the G-code token itself
            iss >> token;

            double x = NAN, y = NAN;
            // parse the rest: tokens like "X1.5" or "Y-2.3"
            while(iss >> token){
                if(token.size()<2) continue;
                char axis = token[0];
                std::string num = token.substr(1);
                try {
                    double v = std::stod(num);
                    if(axis=='X') x = v;
                    else if(axis=='Y') y = v;
                } catch(...){
                    logger->log_error("Warning: bad number in Gcode command: %s", token.c_str());
                }
            }

            if(!std::isnan(x) || !std::isnan(y)){
                move_commands_.push(MoveCommand{ x, y, is_absolute_ });
				return true;
            }
        }
	}

	return false;
}

void
MillThread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	ArduinoInterface *arduino_if = dynamic_cast<ArduinoInterface*>(interface);
	if (arduino_if) {
		arduino_if->read();
		if (arduino_if->is_final()) {
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
	if(!arduino_if_->is_final()) {
		return;
	}
	if (move_commands_.empty()) {
		return;
	}
	MoveCommand cmd = move_commands_.front();
	move_commands_.pop();
	if(cmd.absolute) {
		ArduinoInterface::MoveXYZAbsMessage *move_msg =
			new ArduinoInterface::MoveXYZAbsMessage();
		move_msg->set_x(cmd.x / 1000.0); // Convert mm to meters
		move_msg->set_y(cmd.y / 1000.0);
		move_msg->set_z(0.0);
		move_msg->set_target_frame("end_effector_home");
		logger->log_info(name(), "Moving absolute: x=%f, y=%f", cmd.x, cmd.y);
		arduino_if_->msgq_enqueue(move_msg);
		has_send = true;
	} else {
		ArduinoInterface::MoveXYZRelMessage *move_msg =
			new ArduinoInterface::MoveXYZRelMessage();
		move_msg->set_x(cmd.x / 1000.0); // Convert mm to meters
		move_msg->set_y(cmd.y / 1000.0);
		move_msg->set_z(0.0);
		logger->log_info(name(), "Moving relative: x=%f, y=%f", cmd.x, cmd.y);
		arduino_if_->msgq_enqueue(move_msg);
		has_send = true;
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
