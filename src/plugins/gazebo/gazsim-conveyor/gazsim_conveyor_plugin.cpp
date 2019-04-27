/***************************************************************************
 *  gazsim_conveyor_plugin.cpp - Plugin used to simulate a conveyor vision
 *
 *  Created: Fri Jul 10 11:27:12 2015
 *  Copyright  2015 Randolph Maaßen
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

//#include "conveyor_thread.h"
#include "gazsim_conveyor_thread.h"

using namespace fawkes;

/** Plugin to simulate conveyor
 * @author Frederik Zwilling
 */
class GazsimConveyorPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimConveyorPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new GazsimConveyorThread());
	}
};

PLUGIN_DESCRIPTION("Simulation of a Conveyor")
EXPORT_PLUGIN(GazsimConveyorPlugin)
