
/***************************************************************************
 *  conveyor_pose_thread.h - conveyor_pose
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

#ifndef _CONVEYOR_POSE_THREAD_
#define _CONVEYOR_POSE_THREAD_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/pointcloud.h>

#include <string>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

class ConveyorPoseThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::PointCloudAspect
{
private:
  std::string cloud_in_name_;
  bool cloud_in_registered_;
  pcl::PCLHeader header_;
  fawkes::RefPtr<const Cloud> cloud_in_;
  fawkes::RefPtr<Cloud> cloud_out_plane_;
  fawkes::RefPtr<Cloud> cloud_out_result_;

 /**
  * check if the pointcloud is available
  */
 bool pc_in_check();

 bool is_inbetween(double a, double b, double val);
 CloudPtr cloud_remove_gripper(CloudPtr in);
 CloudPtr cloud_remove_offset_to_front(CloudPtr in);
 CloudPtr cloud_get_plane(CloudPtr in, pcl::ModelCoefficients::Ptr coeff);
 CloudPtr cloud_cluster(CloudPtr in);
 CloudPtr vg(CloudPtr in);
 void cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);

protected:
  virtual void run() { Thread::run(); }

public:
  ConveyorPoseThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

};


#endif
