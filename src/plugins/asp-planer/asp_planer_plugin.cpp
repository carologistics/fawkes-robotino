
/***************************************************************************
 *  asp_planer_plugin.cpp - Planer plugin to schedule commands given to the
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

#include <core/plugin.h>
#include "asp_planer_thread.h"

/** ASP planer plugin.
 */
class AspPlanerPlugin : public fawkes::Plugin
{
	public:
	/** Constructor.
	* @param config Fawkes configuration
	*/
	AspPlanerPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new AspPlanerThread);
		return;
	}
};


PLUGIN_DESCRIPTION("ASP-based planer plugin")
EXPORT_PLUGIN(AspPlanerPlugin)
