/***************************************************************************
 *  box_detect_plugin.cpp - box_detect
 *
 *  Plugin created: May 30 21:54:46 2023

 *  Copyright  2023 Daniel Honies
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
#include "box_detect_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/**
 * @author Daniel Honies
 */
class BoxDetectPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	BoxDetectPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new BoxDetectThread());
	}
};

PLUGIN_DESCRIPTION("Box Detect plugin")
EXPORT_PLUGIN(BoxDetectPlugin)
