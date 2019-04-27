
/***************************************************************************
 *  lasersim_plugin.cpp - Empty example
 *
 *  Created: Do June 14th, 18:07:14 CEST 2012
 *  Copyright  2012  Daniel Ewert
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

#include "laser_sim_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Template! Makes the robotino move forward for 3 seconds
 * @author Daniel Ewert
 */
class LaserSimPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	LaserSimPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new LaserSim());
	}
};

PLUGIN_DESCRIPTION("simple laser simulation")
EXPORT_PLUGIN(LaserSimPlugin)
