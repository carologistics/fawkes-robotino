
/***************************************************************************
 *  conveyor_pose_plugin.cpp -
 *
 *  Created: Thr 12. April 16:28:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
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

#include "conveyor_pose_thread.h"

using namespace fawkes;

/** Conveyor pose, calculates the pose of the conveyor from a pointcloud (from intel real sense)
 * @author Tobias Neumann
 */
class PCLLoopPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  PCLLoopPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new PCLLoopThread());
  }
};

PLUGIN_DESCRIPTION("PCL loop")
EXPORT_PLUGIN(PCLLoopPlugin)
