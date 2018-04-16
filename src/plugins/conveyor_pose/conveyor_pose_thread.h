
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
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>

#include <config/change_handler.h>

#include <plugins/ros/aspect/ros.h>

#include <interfaces/SwitchInterface.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/LaserLineInterface.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>

#include "visualisation.hpp"

#include <string>
#include <map>
#include <atomic>
#include <set>

#define CFG_PREFIX "/plugins/conveyor_pose"

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

class CorrespondenceGroupingThread;

class ConveyorPoseThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ConfigurationChangeHandler,
  public fawkes::BlackBoardAspect,
  public fawkes::PointCloudAspect,
  public fawkes::ROSAspect,
  public fawkes::TransformAspect
{
private:
  friend class CorrespondenceGroupingThread;

  class pose : public fawkes::tf::Pose {
  public:
    using fawkes::tf::Pose::Pose;
    pose() = delete;
    pose(bool valid, float quality = 0)
      : fawkes::tf::Pose()
      , valid(valid)
      , quality(quality)
    {}

    bool operator == (const pose &other) const {
      return this->getBasis() == other.getBasis()
          && this->getOrigin() == other.getOrigin();
    }

    bool valid;
    float quality;
  };

  struct compare_poses_by_quality {
    bool operator () (const pose &lhs, const pose &rhs) const {
      return lhs.quality >= rhs.quality;
    }
  };

  Visualisation * visualisation_;

  // cfg values
  std::string cfg_if_prefix_;
  std::string cloud_in_name_;
  const std::string cloud_out_raw_name_;
  const std::string cloud_out_trimmed_name_;
  std::string cfg_bb_realsense_switch_name_;
  std::string conveyor_frame_id_;
  std::vector<std::string> laserlines_names_;

  CloudPtr model_;
  CloudPtr model_keypoints_;
  pcl::PointCloud<pcl::Normal>::Ptr model_normals_;
  pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors_;

  pcl::UniformSampling<Point> uniform_sampling_;
  pcl::NormalEstimationOMP<Point, pcl::Normal> norm_est_;
  pcl::SHOTEstimationOMP<Point, pcl::Normal, pcl::SHOT352> descr_est_;

  CorrespondenceGroupingThread *cg_thread_;

  fawkes::Mutex config_mutex_;

  bool cfg_record_model_;
  std::string cfg_model_path_;
  std::string cfg_model_origin_frame_;

  bool cfg_pose_close_if_no_new_pointclouds_;
//  std::string bb_tag_name_;
  std::atomic<float> cfg_pose_diff_;
  std::atomic<float> vis_hist_angle_diff_;

  std::atomic<float> cfg_gripper_y_min_;
  std::atomic<float> cfg_gripper_y_max_;
  std::atomic<float> cfg_gripper_z_max_;
  std::atomic<float> cfg_gripper_slice_y_min_;
  std::atomic<float> cfg_gripper_slice_y_max_;

  std::atomic<float> cfg_front_space_;
  std::atomic<float> cfg_front_offset_;

  std::atomic<float> cfg_left_cut_;
  std::atomic<float> cfg_right_cut_;
  std::atomic<float> cfg_left_cut_no_ll_;
  std::atomic<float> cfg_right_cut_no_ll_;

  std::atomic<float> cfg_voxel_grid_leaf_size_;

  std::atomic<double> cfg_model_ss_;
  std::atomic<double> cfg_scene_ss_;
  std::atomic<double> cfg_rf_rad_;
  std::atomic<double> cfg_descr_rad_;
  std::atomic<double> cfg_cg_size_;
  std::atomic<float> cfg_max_descr_dist_;
  std::atomic<int> cfg_cg_thresh_;
  std::atomic_bool cfg_use_hough_;

  uint cfg_allow_invalid_poses_;

  // state vars
  bool enable_pose_;
  bool cfg_enable_switch_;
  bool cfg_debug_mode_;
  bool cfg_enable_product_removal_;
  bool cloud_in_registered_;
  bool cfg_use_visualisation_;
  pcl::PCLHeader header_;
  int vis_hist_;

  size_t cfg_pose_avg_hist_size_;
  size_t cfg_pose_avg_min_;

  std::set<pose, compare_poses_by_quality> poses_;

  // point clouds from pcl_manager
  fawkes::RefPtr<const Cloud> cloud_in_;
  fawkes::RefPtr<Cloud> cloud_out_raw_;
  fawkes::RefPtr<Cloud> cloud_out_trimmed_;
  fawkes::RefPtr<Cloud> cloud_out_model_;

  // interfaces write
  fawkes::SwitchInterface * bb_enable_switch_;
  fawkes::Position3DInterface * bb_pose_;

  // interfaces read
  std::vector<fawkes::LaserLineInterface * > laserlines_;
  fawkes::SwitchInterface *realsense_switch_;
  fawkes::Time wait_start_;
  fawkes::Time wait_time_;

  fawkes::Mutex pose_mutex_;
  fawkes::Mutex cloud_mutex_;

  CloudPtr trimmed_scene_;
//  fawkes::Position3DInterface * bb_tag_;

 /**
  * check if the pointcloud is available
  */
 bool update_input_cloud();
 void bb_pose_conditional_open();
 void bb_pose_conditional_close();

 void pose_add_element(pose element);
 bool pose_get_avg(pose & out);
 CloudPtr get_scene();

 void if_read();
 bool laserline_get_best_fit(fawkes::LaserLineInterface * &best_fit);
 Eigen::Vector3f laserline_get_center_transformed(fawkes::LaserLineInterface * ll);

 bool is_inbetween(double a, double b, double val);

 CloudPtr cloud_remove_gripper(CloudPtr in);
 CloudPtr cloud_remove_offset_to_bottom(CloudPtr in);
 CloudPtr cloud_remove_offset_to_front(CloudPtr in, fawkes::LaserLineInterface * ll = NULL, bool use_ll = false);
 CloudPtr cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll);
 boost::shared_ptr<std::vector<pcl::PointIndices>> cloud_cluster(CloudPtr in);
 CloudPtr cloud_voxel_grid(CloudPtr in);

 void cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);

 void tf_send_from_pose_if(pose pose);
 void pose_write(pose pose);

 Eigen::Quaternion<float> averageQuaternion(
     Eigen::Vector4f &cumulative,
     Eigen::Quaternion<float> newRotation,
     Eigen::Quaternion<float> firstRotation,
     float addDet);

 Eigen::Quaternion<float> normalizeQuaternion(float x, float y, float z, float w);
 Eigen::Quaternion<float> inverseSignQuaternion(Eigen::Quaternion<float> q);
 bool areQuaternionsClose(Eigen::Quaternion<float> q1, Eigen::Quaternion<float> q2);

 void pose_publish_tf(pose pose);
 void start_waiting();
 bool need_to_wait();

 virtual void config_value_erased(const char *path) override;
 virtual void config_tag_changed(const char *new_tag) override;
 virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v) override;
 virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v) override;

 template<typename T>
 inline void change_val(const std::string &setting, std::atomic<T> &var, const T& val)
 {
   if (var != val) {
     logger->log_info(name(), "Changing %s from %s to %s",
                      setting.c_str(), std::to_string(var).c_str(), std::to_string(val).c_str());
     var = val;
   }
 }

protected:
  virtual void run() override
  { Thread::run(); }

public:
  ConveyorPoseThread();

  virtual void init() override;
  virtual void loop() override ;
  virtual void finalize() override;

  void set_cg_thread(CorrespondenceGroupingThread *cg_thread);

};


#endif
