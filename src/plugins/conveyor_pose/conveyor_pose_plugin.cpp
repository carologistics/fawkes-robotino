
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

#include "conveyor_pose_thread.h"
#include "recognition_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Conveyor pose, calculates the pose of the conveyor from a pointcloud (from
 * intel real sense)
 * @author Tobias Neumann
 */
class ConveyorPosePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	ConveyorPosePlugin(Configuration *config) : Plugin(config)
	{
		ConveyorPoseThread *pose_thread = new ConveyorPoseThread();
		RecognitionThread * cg_thread   = new RecognitionThread(pose_thread);
		pose_thread->set_cg_thread(cg_thread);
		thread_list.push_back(pose_thread);
		thread_list.push_back(cg_thread);
	}
};

PLUGIN_DESCRIPTION("Detect the conveyor belt in a pointcloud")
EXPORT_PLUGIN(ConveyorPosePlugin)
