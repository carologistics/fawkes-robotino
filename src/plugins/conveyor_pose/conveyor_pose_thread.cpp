/***************************************************************************
 *  conveyor_pose_thread.cpp - conveyor_pose thread
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

#include "conveyor_pose_thread.h"

#include <cmath>

using namespace fawkes;

/** @class PluginTemplateThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
PCLLoopThread::PCLLoopThread() :
		Thread("PCLLoopThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{

}

void PCLLoopThread::init()
{
  logger->log_info(name(), "Plugin PCLLoop started");

  pcl_ = new pcl::PointCloud<pcl::PointXYZ>();
  pcl_manager->add_pointcloud<pcl::PointXYZ>("PCLLoop", pcl_);
}

void PCLLoopThread::finalize()
{
  pcl_manager->remove_pointcloud("PCLLoop");
}

void PCLLoopThread::loop()
{
  if (pcl_manager->exists_pointcloud("/depth/points")) {
    ros_cloud_ = pcl_manager->get_pointcloud<pcl::PointXYZ>("/depth/points");
    **pcl_ = **ros_cloud_;
  }
}

