
/***************************************************************************
 *  laser_front_dist_plugin.cpp - laser_front_dist
 *
 *  Plugin created: Thu Jun 23 22:35:32 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "laser_front_dist_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/**Calculates distance to object in front with average of laser beams in front.
 * @author Frederik Zwilling
 */
class LaserFrontDistPlugin : public fawkes::Plugin
{
public:
	/** Constructor
   * @param config Fakwes configuration
   */
	LaserFrontDistPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new LaserFrontDistThread());
	}
};

PLUGIN_DESCRIPTION("Calculates distance to object in front with average of "
                   "laser beams in front")
EXPORT_PLUGIN(LaserFrontDistPlugin)
