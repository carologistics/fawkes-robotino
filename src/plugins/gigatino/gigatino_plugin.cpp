
/***************************************************************************
 *  gigatino_plugin.cpp - Plugin for Arduino platform support
 *
 *  Created: Sat Mar 13 15:31:57 2025
 *  Copyright  2025  Tarik Viehmann
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

#include "com_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to provide Arduino platform support for Fawkes.
 * @author Tim Niemueller, Nicolas Limpert
 */
class GigatinoPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GigatinoPlugin(Configuration *config) : Plugin(config)
	{
		std::string prefix = "/gigatino";

		std::string cfg_name   = prefix.substr(1, prefix.length());
		std::string cfg_prefix = prefix + "/";

		cfg_name = cfg_name.substr(0, cfg_name.find("/"));

		GigatinoROSThread *com_thread = new GigatinoROSThread(cfg_name, cfg_prefix);

		thread_list.push_back(com_thread);
	}
};

PLUGIN_DESCRIPTION("Gigatino bridge")
EXPORT_PLUGIN(GigatinoPlugin)
