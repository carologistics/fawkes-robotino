/***************************************************************************
 *  object_tracking.cpp - Thread tracks workpieces, conveyor belts, and slides
 *      using yolo while providing curresponding target frames used for visual
 *      servoing
 *
 *  Created: Tue Jan 25 18:25:15 2022
 *  Copyright  2022  Matteo Tschesche
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

#include "object_tracking_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin Tracks Objects and Provides the Curresponding Target Frames
 *
 *
 * @author Matteo Tschesche
 */
class ObjectTrackingPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	explicit ObjectTrackingPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ObjectTrackingThread());
	}
};

PLUGIN_DESCRIPTION("Object Tracking using YOLO")
EXPORT_PLUGIN(ObjectTrackingPlugin)
