/***************************************************************************
 *  velocity_share_plugin.cpp - A plugin to share positions and velocities
 *  via robot_memory
 *
 *  Created: Sat Jan 20 16:47:28 2018
 *  Copyright  2018  Nicolas Limpert
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

#include "velocity_share_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to share positions and velocities between multiple robots.
 *  The respective positions and velocities are written to an interface
 *  but also published as a ROS message.
 *  Publication as a ROS message can help in providing a more advanced
 *  collision avoidance when ROS move_base is used and the respective info
 *  about other moving robots is processed accordingly.
 * @author Nicolas Limpert
 */
class VelocitySharePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	VelocitySharePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new VelocityShareThread());
	}
};

PLUGIN_DESCRIPTION("Fawkes Velocity Share Plugin to share position and velocities")
EXPORT_PLUGIN(VelocitySharePlugin)
