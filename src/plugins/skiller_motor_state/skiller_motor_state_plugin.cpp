
/***************************************************************************
 *  skiller_motor_state_plugin.cpp - Indicate skiller & motor state through LED
 *
 *  Created: Fri Jun 14 15:03:30 2019
 *  Copyright  2019  Morian Sonnet
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

#include "skiller_motor_state_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to indicate skiller state by LED.
 * @author Morian Sonnet
 */
class SkillerMotorStatePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	SkillerMotorStatePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new SkillerMotorStateThread());
	}
};

PLUGIN_DESCRIPTION("Indicate skiller and motor state through LED")
EXPORT_PLUGIN(SkillerMotorStatePlugin)
