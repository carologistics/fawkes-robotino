
/***************************************************************************
 *  robotino_omnivision_plugin.cpp - Robotino OmniVision Plugin
 *                                   (based on Mid-Size omni_ball)
 *
 *  Created: Thu May 24 17:11:46 2012
 *  Copyright  2007-2012  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
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

#include "pipeline_thread.h"
#include "sensor_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Robotino OmniVision Plugin.
 * Plugin for processing omni-cam images to extract information about the
 * puck and colored field positions relative to the robot.
 * @author Tim Niemueller
 */
class RobotinoOmniVisionPlugin : public fawkes::Plugin
{
 public:

  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotinoOmniVisionPlugin(Configuration *config)
    : Plugin(config)
  {
	RobotinoOmniVisionPipelineThread *t = new RobotinoOmniVisionPipelineThread();
    thread_list.push_back(t);
    thread_list.push_back(new OmniVisionSensorThread(t));
  }
};

PLUGIN_DESCRIPTION("Robotino omni-vision image processing")
EXPORT_PLUGIN(RobotinoOmniVisionPlugin)
