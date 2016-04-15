
/***************************************************************************
 *  motor_led_plugin.cpp - Indicate motor status through LED
 *
 *  Created: Fri Apr 15 15:03:30 2016
 *  Copyright  2016  Tim Niemueller [www.niemueller.de]
 *
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

#include <core/plugin.h>

#include "motor_led_thread.h"

using namespace fawkes;

/** Plugin to indicate motor status by LED.
 * @author Tim Niemueller
 */
class MotorLedPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	MotorLedPlugin(Configuration *config)
		: Plugin(config)
	{
		std::string cfg_driver = config->get_string("/hardware/robotino/driver");
	  
		thread_list.push_back(new MotorLedThread());
	}
};

PLUGIN_DESCRIPTION("Indicate motor status through LED")
EXPORT_PLUGIN(MotorLedPlugin)
