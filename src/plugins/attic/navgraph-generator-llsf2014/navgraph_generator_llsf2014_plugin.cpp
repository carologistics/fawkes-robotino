
/***************************************************************************
 *  navgraph_generator_llsf2014_plugin.cpp - navgraph generator LLSF2014
 *
 *  Created: Tue Jan 13 15:30:15 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_generator_llsf2014_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Automatically generate navgraph for LLSF2014
 * @author Tim Niemueller
 */
class NavGraphGenerator2014Plugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	NavGraphGenerator2014Plugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new NavGraphGenerator2014Thread());
	}
};

PLUGIN_DESCRIPTION("Generate navgraph for LLSF2014 game")
EXPORT_PLUGIN(NavGraphGenerator2014Plugin)
