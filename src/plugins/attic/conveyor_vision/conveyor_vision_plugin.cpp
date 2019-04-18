
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

#include <core/plugin.h>

#include "conveyor_vision_thread.h"

using namespace fawkes;

/**
 * @author Nicolas Limpert & Randolph Maaßen
 */
class ConveyorVision : public fawkes::Plugin {
public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ConveyorVision(Configuration *config) : Plugin(config) {
    thread_list.push_back(new ConveyorVisionThread());
  }
};

PLUGIN_DESCRIPTION("Conveyor Vision plugin")
EXPORT_PLUGIN(ConveyorVision)
