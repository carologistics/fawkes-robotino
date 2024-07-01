/***************************************************************************
 *  picam_client_plugin.cpp - Picam Client Plugin for Fawkes
 *
 *  Created: Sun Jun 30 17:36:00 2024
 *  Copyright  2024 Daniel Swoboda
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

#include "picam_client_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/**
 * Plugin to create a client that subscribes to a local picam server
 *
 * @author Daniel Swoboda
 */
class PicamClientPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	explicit PicamClientPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new PicamClientThread());
	}
};

PLUGIN_DESCRIPTION("Plugin to create a client that subscribes to a local picam server")
EXPORT_PLUGIN(PicamClientPlugin)
