
/***************************************************************************
 *  asp_agent_plugin.cpp - Agent plugin to execute commands given from the
 *                         asp planer
 *
 *  Created on Mon Aug 15 17:39:02 2016
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
#include "asp_agent_thread.h"

/** ASP agent plugin.
 */
class AspAgentPlugin : public fawkes::Plugin
{
	public:
	/** Constructor.
	* @param config Fawkes configuration
	*/
	AspAgentPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new AspAgentThread);
		return;
	}
};


PLUGIN_DESCRIPTION("ASP-based agent plugin")
EXPORT_PLUGIN(AspAgentPlugin)
