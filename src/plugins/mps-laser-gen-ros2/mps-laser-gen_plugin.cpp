
/***************************************************************************
 *  mps-laser-gen_plugin.cpp - mps-laser-gen
 *
 *  Plugin created: Thu Jun 30 21:54:46 2016
 *  Copyright  2016  Tim Niemueller
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

#include "mps-laser-gen_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to generate virtual laser data based on known MPS positions.
 * @author Tim Niemueller
 */
class MPSLaserGenPlugin : public fawkes::Plugin
{
public:
	/** Constructor
   * @param config Fakwes configuration
   */
	MPSLaserGenPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new MPSLaserGenThread());
	}
};

PLUGIN_DESCRIPTION("Generate virtual laser data for known MPS")
EXPORT_PLUGIN(MPSLaserGenPlugin)
