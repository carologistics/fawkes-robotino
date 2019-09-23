
/***************************************************************************
 *  navgraph_broker_plugin.cpp
 *
 *  Created on Sat March 29th 12:24:01 2014
 *  Copyright (C) 2014 by Sebastian reuter, Carologistics RoboCup Team]
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

#include "navgraph_broker_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** NavGraph Broker plugin.
 * @author Sebastian Reuter
 */
class NavGraphBrokerPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	NavGraphBrokerPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new NavgraphBrokerThread());
	}
};

PLUGIN_DESCRIPTION("NavGraph Broker")
EXPORT_PLUGIN(NavGraphBrokerPlugin)
