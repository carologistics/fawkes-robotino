
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
  class pose {
  public:
    bool valid = true;
    fawkes::tf::Vector3 translation;
    fawkes::tf::Quaternion rotation;
    pose() {
      translation.setX(0);
      translation.setY(0);
      translation.setZ(0);
      rotation.setX(0);
      rotation.setY(0);
      rotation.setZ(0);
      rotation.setW(0);
    }
  };
  Visualisation * visualisation_;

  // cfg values
  std::string cloud_in_name_;
  std::string cloud_out_inter_1_name_;
  std::string cloud_out_result_name_;
  std::string cfg_bb_conveyor_pose_name_;
  std::string cfg_bb_switch_name_;
  std::string conveyor_frame_id_;
  std::vector<std::string> laserlines_names_;

  bool cfg_pose_close_if_no_new_pointclouds_;
//  std::string bb_tag_name_;
  float cfg_pose_diff_;
  float vis_hist_angle_diff_;

  float cfg_gripper_y_min_;
  float cfg_gripper_y_max_;
  float cfg_gripper_z_max_;
  float cfg_gripper_slice_y_min_;
  float cfg_gripper_slice_y_max_;

  float cfg_front_space_;
  float cfg_front_offset_;

  float cfg_bottom_offset_;

  float cfg_left_cut_;
  float cfg_right_cut_;
  float cfg_left_cut_no_ll_;
  float cfg_right_cut_no_ll_;

  float cfg_plane_dist_threshold_;

  float cfg_plane_height_minimum_;
  float cfg_plane_width_minimum_;
  float cfg_normal_z_minimum_;

  float cfg_cluster_tolerance_;
  float cfg_cluster_size_min_;
  float cfg_cluster_size_max_;

  float cfg_voxel_grid_leave_size_;

  uint cfg_allow_invalid_poses_;

  // state vars
  bool enable_pose_;
  bool cfg_enable_switch_;
  bool cfg_debug_mode_;
  bool cfg_enable_product_removal_;
  bool cloud_in_registered_;
  bool cfg_use_visualisation_;
  pcl::PCLHeader header_;
  std::pair<fawkes::tf::Vector3, fawkes::tf::Quaternion> pose_;
  int vis_hist_;

  size_t cfg_pose_avg_hist_size_;
  size_t cfg_pose_avg_min_;

  std::list<pose> poses_;

  // point clouds from pcl_manager
  fawkes::RefPtr<const Cloud> cloud_in_;
  fawkes::RefPtr<Cloud> cloud_out_inter_1_;
  fawkes::RefPtr<Cloud> cloud_out_result_;

  // interfaces write
  fawkes::SwitchInterface * bb_enable_switch_;
  fawkes::Position3DInterface * bb_pose_;

  // interfaces read
  std::vector<fawkes::LaserLineInterface * > laserlines_;
//  fawkes::Position3DInterface * bb_tag_;

 /**
  * check if the pointcloud is available
  */
 bool pc_in_check();
 void bb_pose_conditional_open();
 void bb_pose_conditional_close();

 void pose_add_element(pose element);
 bool pose_get_avg(pose & out);

 void if_read();
 bool laserline_get_best_fit(fawkes::LaserLineInterface * &best_fit);
 Eigen::Vector3f laserline_get_center_transformed(fawkes::LaserLineInterface * ll);

 bool is_inbetween(double a, double b, double val);

 CloudPtr cloud_remove_gripper(CloudPtr in);
 CloudPtr cloud_remove_offset_to_bottom(CloudPtr in);
 CloudPtr cloud_remove_offset_to_front(CloudPtr in, fawkes::LaserLineInterface * ll = NULL, bool use_ll = false);
 CloudPtr cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll);
 CloudPtr cloud_get_plane(CloudPtr in, pcl::ModelCoefficients::Ptr coeff);
 boost::shared_ptr<std::vector<pcl::PointIndices>> cloud_cluster(CloudPtr in);
 CloudPtr cloud_voxel_grid(CloudPtr in);

 std::vector<CloudPtr> cluster_split(CloudPtr in, boost::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices);
 CloudPtr cluster_find_biggest(std::vector<CloudPtr> clouds_in, size_t & id);

 void cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);

 pose calculate_pose(Eigen::Vector4f centroid, Eigen::Vector3f normal);
 void tf_send_from_pose_if(pose pose);
 void pose_write(pose pose);
 Eigen::Quaternion<float> averageQuaternion(Eigen::Vector4f &cumulative, Eigen::Quaternion<float> newRotation, Eigen::Quaternion<float> firstRotation, float addDet);
 Eigen::Quaternion<float> normalizeQuaternion(float x, float y, float z, float w);
 Eigen::Quaternion<float> inverseSignQuaternion(Eigen::Quaternion<float> q);
 bool areQuaternionsClose(Eigen::Quaternion<float> q1, Eigen::Quaternion<float> q2);

protected:
  virtual void run() { Thread::run(); }
  void pose_publish_tf(pose pose);

public:
  ConveyorPoseThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

};


#endif
