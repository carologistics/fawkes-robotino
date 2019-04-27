
/***************************************************************************
 *  clips-motor-switch-plugin.cpp - CLIPS support for switching the motors
 *
 *  Created: Thu Apr 25 12:29:18 2013 (Magdeburg)
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "clips-motor-switch-thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** CLIPS motor switching plugin.
 * @author Tim Niemueller
 */
class ClipsMotorSwitchPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	ClipsMotorSwitchPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ClipsMotorSwitchThread());
	}
};

PLUGIN_DESCRIPTION("Motor switching from CLIPS")
EXPORT_PLUGIN(ClipsMotorSwitchPlugin)
