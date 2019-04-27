
/***************************************************************************
 *  plugin_light_plugin.cpp - Empty example
 *
 *  Created: Mi 23. Mai 18:07:14 CEST 2012
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

#include "light_compass_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Detects a light over brithness threashold, offers distance and direction to
 * the nearest light
 * @author Florian Nolden & Tobias Neumann
 */
class LightCompassPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	LightCompassPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new LightCompassThread());
	}
};

PLUGIN_DESCRIPTION("Gives the distance and direction of the nearest light ")
EXPORT_PLUGIN(LightCompassPlugin)
