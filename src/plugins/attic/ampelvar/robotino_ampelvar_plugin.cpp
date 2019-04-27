
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

#include "pipeline_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Robotino AmpelVar Plugin.
 * Plugin for processing omni-cam images to extract information about the
 * ampel.
 * @author Tim Niemueller
 */
class RobotinoAmpelVarPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	RobotinoAmpelVarPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new RobotinoAmpelVarPipelineThread());
	}
};

PLUGIN_DESCRIPTION("Robotino ampel-var image processing")
EXPORT_PLUGIN(RobotinoAmpelVarPlugin)
