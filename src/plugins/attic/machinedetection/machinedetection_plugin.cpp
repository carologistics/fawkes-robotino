
/***************************************************************************
 *  robotino_ampelvision_plugin.cpp - Robotino AmpelVar Plugin
 *                                   (based on Mid-Size omni_ball)
 *
 *  Created: Thu May 24 17:11:46 2012
 *  Copyright  2007-2012  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
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

#include "machinedetection_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Robotino Machinedetection Plugin.
 * Plugin for processing front-cam images to detect a ampel's location
 * The camera must be calibrated accordingly, so that a shining ampel is the
 * brightest spot!
 * @author Daniel Ewert
 */
class RobotinoMachineDetectionPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	RobotinoMachineDetectionPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new RobotinoMachineDetectionThread());
	}
};

PLUGIN_DESCRIPTION("Robotino Machine detection image processing")
EXPORT_PLUGIN(RobotinoMachineDetectionPlugin)
