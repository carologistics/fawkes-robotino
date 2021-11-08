/***************************************************************************
 *  gazsim_object_tracking_plugin.cpp - Plugin simulates object tracking of
 *      workpieces, conveyor belts, and slides while providing curresponding
 *      target frames used for visual servoing in Gazebo
 *
 *  Created: Sun May 16 12:03:10 2021
 *  Copyright  2021  Matteo Tschesche
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

#include "gazsim_object_tracking_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin Provides the Simulated Object Position and Curresponding Target
 *  Frames in Gazebo
 *
 *
 * @author Matteo Tschesche
 */
class GazsimObjectTrackingPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	explicit GazsimObjectTrackingPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ObjectTrackingThread());
	}
};

PLUGIN_DESCRIPTION("Simulation of Object Tracking in Gazebo")
EXPORT_PLUGIN(GazsimObjectTrackingPlugin)
