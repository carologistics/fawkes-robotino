
/***************************************************************************
 *  agent-monitor-plugin.cpp - Monitoring the CLIPS agents in LLSF via webview
 *
 *  Created: Fri May 09 16:04:13 2014
 *  Copyright  2014  Frederik Zwilling
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

#include "agent-monitor-thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Monitoring the CLIPS agents in LLSF via webview
 * @author Frederik Zwilling
 */
class AgentMonitorPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  AgentMonitorPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new AgentMonitorThread());
  }
};


PLUGIN_DESCRIPTION("Monitoring the CLIPS agents in LLSF via webview")
EXPORT_PLUGIN(AgentMonitorPlugin)
