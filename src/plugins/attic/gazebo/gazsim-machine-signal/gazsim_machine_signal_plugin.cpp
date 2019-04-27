/***************************************************************************
 *  gazsim_machine_signal_plugin.cpp - Plugin provides
 *     the detected light signals of the llsf-machines
 *
 *  Created: Thu Apr 02 14:43:42 2015
 *  Copyright  2015 Frederik Zwilling
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

#include "gazsim_machine_signal_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin provides the detected light signals of the llsf-machines
 *
 *
 * @author Frederik Zwilling
 */
class GazsimMachineSignalPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimMachineSignalPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new MachineSignalSimThread());
	}
};

PLUGIN_DESCRIPTION("Simulation of the MachineSignal Plugin results")
EXPORT_PLUGIN(GazsimMachineSignalPlugin)
