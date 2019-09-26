/***************************************************************************
 *  gazsim_navgraph_generator_plugin.cpp - Generates navgraph without
 *exploration
 *
 *  Created: Mon Feb 15 11:25:00 2016
 *  Copyright  2016  David Schmidt
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

#include "gazsim_navgraph_generator_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to generate navgraph without exploration phase in simulation
 * @author David Schmidt
 */
class GazsimNavgraphGeneratorPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimNavgraphGeneratorPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new GazsimNavgraphGeneratorThread());
	}
};

PLUGIN_DESCRIPTION("Gazsim Navgraph Generator Plugin for generating navgraph "
                   "without exploration phase")
EXPORT_PLUGIN(GazsimNavgraphGeneratorPlugin)
