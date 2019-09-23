/***************************************************************************
 *  gazsim_tag_vision_plugin.cpp - Plugin provides ground-truth tag vision
 *
 *  Created: Thu Nov 05 20:16:51 2015
 *  Copyright  2015 Frederik Zwilling
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

#include "gazsim_tag_vision_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin provides ground-truth tag vision
 *
 *
 * @author Frederik Zwilling
 */
class GazsimTagVisionPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimTagVisionPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new TagVisionSimThread());
	}
};

PLUGIN_DESCRIPTION("Simulation of the TagVision results")
EXPORT_PLUGIN(GazsimTagVisionPlugin)
