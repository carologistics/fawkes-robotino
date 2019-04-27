/***************************************************************************
 *  gazsim_llsf_control_plugin.cpp - Plugin controls the llsf simulation
 *
 *  Created: Thu Oct 03 13:11:58 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "gazsim_llsf_control_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/**
 *Plugin controls the llsf simulation
 *
 * @author Frederik Zwilling
 */
class GazsimLlsfControlPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimLlsfControlPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new LlsfControlSimThread());
	}
};

PLUGIN_DESCRIPTION("Makes control of a llsf game in gazebo")
EXPORT_PLUGIN(GazsimLlsfControlPlugin)
