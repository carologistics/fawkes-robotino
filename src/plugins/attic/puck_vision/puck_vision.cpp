
/***************************************************************************
 *  puck_vision.cpp
 *
 *  Created: Mo 03. Jun 18:07:14 CEST 2013
 *  Copyright  2013  Florian Nolden
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

#include "puck_vision_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/**
 * @author Florian Nolden & Johannes Rothe
 */
class PuckVision : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	PuckVision(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new PuckVisionThread());
	}
};

PLUGIN_DESCRIPTION("puck detection using a front looking camera")
EXPORT_PLUGIN(PuckVision)
