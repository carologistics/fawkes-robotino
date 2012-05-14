
/***************************************************************************
 *  navigator_plugin.cpp - Navigator plugin for Robotino
 *
 *  Created: Mon May 14 17:18:52 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "navigator_thread.h"

using namespace fawkes;

/** Navigator plugin for the Robotino.
 * @author Tim Niemueller
 */
class RobotinoNavigatorPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotinoNavigatorPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NavigatorThread());
  }
};

PLUGIN_DESCRIPTION("Robotino navigator plugin")
EXPORT_PLUGIN(RobotinoNavigatorPlugin)
