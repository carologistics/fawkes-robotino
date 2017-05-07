
/***************************************************************************
 *  controller_plugin.cpp - Robotino Controller Plugin
 *
 *  Created: Fri May 07 09:01:14 2017
 *  Copyright  2017  Christoph Henke
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
#include "controller_thread.h"

using namespace fawkes;

/** Controlling the robot pose
 * @author Christoph Henke
 */
class ControllerPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ControllerPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new ControllerThread());
  }
};

PLUGIN_DESCRIPTION("Controlling the robot pose")
EXPORT_PLUGIN(ControllerPlugin)
