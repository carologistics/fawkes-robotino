
/***************************************************************************
 *  clips_plugin.cpp - Plugin to access CLIPS features
 *
 *  Created: Sat Jun 16 14:36:13 2012 (Mexico City)
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "robotino_agent_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Robotino CLIPS agent plugin.
 * @author Tim Niemueller
 */
class RobotinoClipsAgentPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotinoClipsAgentPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new RobotinoClipsAgentThread());
  }
};


PLUGIN_DESCRIPTION("CLIPS-based Robotino Agent")
EXPORT_PLUGIN(RobotinoClipsAgentPlugin)
