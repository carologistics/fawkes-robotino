/***************************************************************************
 *  direct_com_thread.h - Arduino com thread for direct communication
 *
 *  Created: Mon Apr 04 11:48:36 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_MILL_THREAD_H_
#define __PLUGINS_MILL_THREAD_H_

#include <interfaces/ArduinoInterface.h>
#include "interfaces/GcodeInterface.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <config/change_handler.h>
#include <core/threading/thread.h>
#include <interface/interface.h>
#include <interfaces/JoystickInterface.h>
#include <interfaces/RTODataInterface.h>

#include <memory>

namespace fawkes {
class Clock;
class TimeWait;

class BatteryInterface;
class ArduinoInterface;
class GcodeInterface;
} // namespace fawkes

struct MoveCommand {
    double x;
    double y;
    bool absolute;  // true = absolute (G90), false = relative (G91)
};

class MillThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::ClockAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::BlackBoardInterfaceListener,
                         public fawkes::TransformAspect,
                         public fawkes::ConfigurationChangeHandler
{
public:
	MillThread();
	/**
   * @brief Constructor for the arduino communication thread
   *
   * @param cfg_name Name of the config file
   * @param cfg_prefix Prefix tags to arduino config
   */
	MillThread(std::string &cfg_name, std::string &cfg_prefix);
	virtual ~MillThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) noexcept;
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) throw();

	virtual void config_value_erased(const char *path) override;
	virtual void config_tag_changed(const char *new_tag) override;
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v) override;
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v) override;

private:
	std::string cfg_name_;

	fawkes::ArduinoInterface  *arduino_if_;
	fawkes::GcodeInterface    *gcode_if_;

	std::queue<MoveCommand> move_commands_;
	bool is_absolute_ = true;

	bool has_send = false;
	void load_config();
};

#endif
