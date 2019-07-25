/***************************************************************************
 *  gazsim_sensing_plugin.cpp - Plugin used to simulate sensing actions
 *
 *  Created: Mon Jul 15 18:40:22 2019
 *  Copyright  2019 Daniel Habering
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

#include "gazsim_sensing_thread.h"

using namespace fawkes;

/** Plugin to simulate sensing actions
 *
 * @author Frederik Zwilling
 */
class GazsimSensingPlugin : public fawkes::Plugin {
public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimSensingPlugin(Configuration *config) : Plugin(config) {
    thread_list.push_back(new GazsimSensingThread());
  }
};

PLUGIN_DESCRIPTION("Simulation of sensing actions")
EXPORT_PLUGIN(GazsimSensingPlugin)
