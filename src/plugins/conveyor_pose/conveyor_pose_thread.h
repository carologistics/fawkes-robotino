
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
#include <aspect/tf.h>

#include <plugins/ros/aspect/ros.h>

#include <interfaces/SwitchInterface.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/LaserLineInterface.h>

#include "visualisation.hpp"

#include <string>
#include <map>

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
  public fawkes::PointCloudAspect,
  public fawkes::ROSAspect,
  public fawkes::TransformAspect
{
private:
  Visualisation * visualisation_;

  std::string cloud_in_name_;
  std::string cloud_out_inter_1_name_;
  std::string cloud_out_result_name_;
  std::vector<std::string> laserlines_names_;
  std::string bb_tag_name_;
  bool cloud_in_registered_;
  pcl::PCLHeader header_;

  fawkes::RefPtr<const Cloud> cloud_in_;
  fawkes::RefPtr<Cloud> cloud_out_inter_1_;
  fawkes::RefPtr<Cloud> cloud_out_result_;

  fawkes::SwitchInterface * bb_enable_switch_;
  fawkes::Position3DInterface * bb_pose_;
  std::vector<fawkes::LaserLineInterface * > laserlines_;
  fawkes::Position3DInterface * bb_tag_;

  bool cfg_enable_switch_;
  bool cfg_use_visualisation_;

 /**
  * check if the pointcloud is available
  */
 bool pc_in_check();
 bool is_enabled() const { return cfg_enable_switch_; }

 void if_read();
 bool laserline_get_best_fit(fawkes::LaserLineInterface * &best_fit);
 Eigen::Vector3f laserline_get_center_transformed(fawkes::LaserLineInterface * ll);

 bool is_inbetween(double a, double b, double val);

 CloudPtr cloud_remove_gripper(CloudPtr in);
 CloudPtr cloud_remove_centroid_based(CloudPtr in, Eigen::Vector4f centroid);
 CloudPtr cloud_remove_offset_to_front(CloudPtr in, fawkes::LaserLineInterface * ll = NULL, bool use_ll = false);
 CloudPtr cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll);
 CloudPtr cloud_get_plane(CloudPtr in, pcl::ModelCoefficients::Ptr coeff);
 CloudPtr cloud_cluster(CloudPtr in);
 CloudPtr vg(CloudPtr in);

 void cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);
 void bb_switch_is_enabled();

 std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> calculate_pose(Eigen::Vector4f centroid, Eigen::Vector3f normal);
 void tf_send_from_pose_if(std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose);
 void pose_write(std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose, int vis_hist);

protected:
  virtual void run() { Thread::run(); }

public:
  ConveyorPoseThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

};


#endif
