
/***************************************************************************
 *  conveyor_pose_plugin.cpp -
 *
 *  Created: Thr 12. April 16:28:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
 *             2018 Victor Mataré
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

#include "workpiece_pose_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Workpiece pose, matches the center of a Yolo Object Detection box to a Pointcloud(from
 * intel real sense)
 * @author Sebastian Eltester
 */
class WorkpiecePosePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	WorkpiecePosePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new WorkpiecePoseThread());
	}
};

PLUGIN_DESCRIPTION("Return pose of center of a Workpiece bounding box in a pointcloud")
EXPORT_PLUGIN(WorkpiecePosePlugin)
