
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

#include "com_thread.h"
#include "tf_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to provide Arduino platform support for Fawkes.
 * @author Tim Niemueller, Nicolas Limpert
 */
class ArduinoPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	ArduinoPlugin(Configuration *config) : Plugin(config)
	{
		std::string prefix = "/arduino";

		std::string cfg_name   = prefix.substr(1, prefix.length());
		std::string cfg_prefix = prefix + "/";

		cfg_name = cfg_name.substr(0, cfg_name.find("/"));

		ArduinoTFThread * tf_thread  = new ArduinoTFThread(cfg_name, cfg_prefix);
		ArduinoComThread *com_thread = new ArduinoComThread(cfg_name, cfg_prefix, tf_thread);

		thread_list.push_back(tf_thread);
		thread_list.push_back(com_thread);
	}
};

PLUGIN_DESCRIPTION("Arduino platform support")
EXPORT_PLUGIN(ArduinoPlugin)
