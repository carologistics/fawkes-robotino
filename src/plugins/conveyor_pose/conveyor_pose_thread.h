
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

//#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>

#include <string>
#include <map>
#include <atomic>
#include <set>
#include <array>



#define CFG_PREFIX "/plugins/conveyor_pose"

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

class RecognitionThread;

namespace fawkes {
    class ConveyorPoseInterface;
    class SwitchInterface;
    class LaserLineInterface;
}

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
  friend class RecognitionThread;

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

  //Visualisation * visualisation_;

  // cfg values
  std::string cfg_if_prefix_;
  std::string cloud_in_name_;
  const std::string cloud_out_raw_name_;
  const std::string cloud_out_trimmed_name_;
  std::string cfg_bb_realsense_switch_name_;
  std::string conveyor_frame_id_;
  std::vector<std::string> laserlines_names_;

  std::atomic_bool icp_cancelled_;

  CloudPtr model_;
  CloudPtr aligned_model_;
  CloudPtr trimmed_scene_;

  pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normals_;
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals_;
  Eigen::Matrix4f initial_tf_;

  std::string current_station_;

  void set_current_station(std::string station);


  //Mapping from station to its correspoinding model path
  std::map<std::string, std::string> station_to_path_;

  // Mapping from station name to preprocessed pointcloud model
  std::map<std::string, pcl::PointCloud<pcl::PointNormal>::Ptr> station_to_model_;


  pcl::NormalEstimationOMP<Point, pcl::PointNormal> norm_est_;


  RecognitionThread *cg_thread_;

  fawkes::Mutex config_mutex_;

  bool cfg_record_model_;
  std::string cfg_model_path_;
  std::string cfg_model_origin_frame_;
  std::string cfg_record_path_;



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

  std::atomic<float> cfg_icp_max_corr_dist_;

  std::atomic<float> cfg_voxel_grid_leaf_size_;
  std::map<std::string, std::array<std::atomic<float>, 3> > cfg_hint_;

  // state vars
  bool cfg_enable_switch_;
  bool cfg_debug_mode_;
  bool cloud_in_registered_;
  pcl::PCLHeader header_;

  //std::set<pose, compare_poses_by_quality> poses_;
  fawkes::tf::Stamped<fawkes::tf::Pose> result_pose_;
  std::atomic<double> result_fitness_;

  // point clouds from pcl_manager
  fawkes::RefPtr<const Cloud> cloud_in_;
  fawkes::RefPtr<Cloud> cloud_out_raw_;
  fawkes::RefPtr<Cloud> cloud_out_trimmed_;
  fawkes::RefPtr<Cloud> cloud_out_model_;

  // interfaces write
  fawkes::SwitchInterface *bb_enable_switch_;
  fawkes::ConveyorPoseInterface *bb_pose_;

  // interfaces read
  std::vector<fawkes::LaserLineInterface * > laserlines_;
  fawkes::SwitchInterface *realsense_switch_;
  fawkes::Time wait_start_;
  fawkes::Time wait_time_;

  fawkes::Mutex bb_mutex_;
  fawkes::Mutex cloud_mutex_;

//  fawkes::Position3DInterface * bb_tag_;

 /**
  * check if the pointcloud is available
  */
 bool update_input_cloud();

 void if_read();
 bool laserline_get_best_fit(fawkes::LaserLineInterface * &best_fit);
 Eigen::Vector3f laserline_get_center_transformed(fawkes::LaserLineInterface * ll);
 fawkes::tf::Stamped<fawkes::tf::Pose> laserline_get_center(fawkes::LaserLineInterface *ll);

 Eigen::Matrix4f guess_initial_tf_from_laserline(fawkes::LaserLineInterface *ll, std::string hint_id);

 bool is_inbetween(double a, double b, double val);

 CloudPtr cloud_remove_gripper(CloudPtr in);
 CloudPtr cloud_remove_offset_to_bottom(CloudPtr in);
 CloudPtr cloud_remove_offset_to_front(CloudPtr in, fawkes::LaserLineInterface * ll = NULL, bool use_ll = false);
 CloudPtr cloud_remove_offset_to_left_right(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll);
 boost::shared_ptr<std::vector<pcl::PointIndices>> cloud_cluster(CloudPtr in);
 CloudPtr cloud_voxel_grid(CloudPtr in);

 void cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);

 void tf_send_from_pose_if(pose pose);
 void pose_write();
 void record_model();

 Eigen::Quaternion<float> averageQuaternion(
     Eigen::Vector4f &cumulative,
     Eigen::Quaternion<float> newRotation,
     Eigen::Quaternion<float> firstRotation,
     float addDet);

 Eigen::Quaternion<float> normalizeQuaternion(float x, float y, float z, float w);
 Eigen::Quaternion<float> inverseSignQuaternion(Eigen::Quaternion<float> q);
 bool areQuaternionsClose(Eigen::Quaternion<float> q1, Eigen::Quaternion<float> q2);

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

  void pose_publish_tf(const fawkes::tf::Pose &pose);
  void start_waiting();
  bool need_to_wait();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ConveyorPoseThread();

  virtual void init() override;
  virtual void loop() override ;
  virtual void finalize() override;

  void set_cg_thread(RecognitionThread *cg_thread);

};


#endif
