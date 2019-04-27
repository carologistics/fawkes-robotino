/***************************************************************************
 *  gazsim_llsf_statistics_plugin.cpp - Plugin generates a statistic about
 *     a game
 *
 *  Created: Mon Sep 23 17:12:33 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "gazsim_llsf_statistics_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin provides the position of llsf-pucks
 *
 *
 * @author Frederik Zwilling
 */
class GazsimLlsfStatisticsPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	GazsimLlsfStatisticsPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new LlsfStatisticsSimThread());
	}
};

PLUGIN_DESCRIPTION("Makes statistics of a llsf game in gazebo")
EXPORT_PLUGIN(GazsimLlsfStatisticsPlugin)
