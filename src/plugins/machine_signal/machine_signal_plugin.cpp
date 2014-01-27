
/***************************************************************************
 *  machine_signal_plugin.cpp - Detect signals using color thresholds
 *
 *  Copyright  2014 Victor Mataré
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

#ifndef MACHINE_SIGNAL_PLUGIN_H_

#include <core/plugin.h>

#include "machine_signal_thread.h"

using namespace fawkes;

class MachineSignalPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
    MachineSignalPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new MachineSignalThread());
  }
};

PLUGIN_DESCRIPTION("Detect signals using color thresholds")
EXPORT_PLUGIN(MachineSignalPlugin)

#endif /* MACHINE_SIGNAL_PLUGIN_H_ */
