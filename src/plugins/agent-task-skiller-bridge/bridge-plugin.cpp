
/***************************************************************************
 *  bridge-plugin.cpp - bridge between skiller and agent task proto msgs
 *
 *  Created: Mon May 20 2024
 *  Copyright  2024  Tarik Viehmann
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

#include "bridge-thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Agent task skiller bridge plguin
 * @author Tarik Viehmann
 */
class AgentTaskSkillerBridgePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	AgentTaskSkillerBridgePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new AgentTaskSkillerBridgeThread());
	}
};

PLUGIN_DESCRIPTION("Translate between skiller and agent task protobuf messages")
EXPORT_PLUGIN(AgentTaskSkillerBridgePlugin)
