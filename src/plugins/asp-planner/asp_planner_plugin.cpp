
/***************************************************************************
 *  asp_planner_plugin.cpp - Planner plugin to schedule commands given to the
 *                          asp agent
 *
 *  Created on Thu Aug 18 04:20:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
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

#include "asp_planner_thread.h"

#include <core/plugin.h>

/** ASP planner plugin.
 */
class AspPlannerPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	AspPlannerPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new AspPlannerThread);
		return;
	}
};

PLUGIN_DESCRIPTION("ASP-based planner plugin")
EXPORT_PLUGIN(AspPlannerPlugin)
