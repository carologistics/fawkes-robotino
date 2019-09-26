/***************************************************************************
 *  gazsim_gripper_plugin.cpp - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:40:22 2015
 *  Copyright  2015 Frederik Zwilling
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

//#include "gripper_thread.h"
#include "gazsim_gripper_thread.h"

using namespace fawkes;

/** Plugin to simulate gripper
 * @author Frederik Zwilling
 */
class GazsimGripperPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimGripperPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new GazsimGripperThread());
	}
};

PLUGIN_DESCRIPTION("Simulation of a Gripper")
EXPORT_PLUGIN(GazsimGripperPlugin)
