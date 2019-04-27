
/***************************************************************************
 *  plugin_template_plugin.cpp - Empty example
 *
 *  Created: Mi 23. Mai 18:07:14 CEST 2012
 *  Copyright  2012  Daniel Ewert
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

#include "omnivision_activator_thread.h"

#include <core/plugin.h>
#include <interfaces/SwitchInterface.h>

using namespace fawkes;

/** Activates the Omnivison
 * @author Johannes Rothe
 */
class OmnivisionActivatorPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	OmnivisionActivatorPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new OmnivisionActivatorThread());
	}
};

PLUGIN_DESCRIPTION("Activates the Omnivision Thread")
EXPORT_PLUGIN(OmnivisionActivatorPlugin)
