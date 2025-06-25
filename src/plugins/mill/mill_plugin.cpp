
/***************************************************************************
 *  robotino_plugin.cpp - Plugin for Arduino platform support
 *
 *  Created: Sun Nov 13 15:31:57 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "mill_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to provide Arduino platform support for Fawkes.
 * @author Tim Niemueller, Nicolas Limpert
 */
class MillPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	MillPlugin(Configuration *config) : Plugin(config)
	{
		std::string prefix = "/mill";

		std::string cfg_name   = prefix.substr(1, prefix.length());
		std::string cfg_prefix = prefix + "/";

		cfg_name = cfg_name.substr(0, cfg_name.find("/"));

		MillThread *exec_thread = new MillThread(cfg_name, cfg_prefix);

		thread_list.push_back(exec_thread);
	}
};

PLUGIN_DESCRIPTION("Mill platform support")
EXPORT_PLUGIN(MillPlugin)
