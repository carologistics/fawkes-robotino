
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
#include <aspect/syncpoint_manager.h>

#include <interfaces/ConveyorPoseInterface.h>

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
  public fawkes::TransformAspect,
  public fawkes::SyncPointManagerAspect
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> Cloud;
  typedef Cloud::Ptr CloudPtr;
  typedef Cloud::ConstPtr CloudConstPtr;

  ConveyorPoseThread();

  virtual void init() override;
  virtual void loop() override ;
  virtual void finalize() override;

  float cloud_resolution() const;
  void cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);

  void set_cg_thread(RecognitionThread *cg_thread);
  void bb_set_busy(bool busy);

  CloudPtr model_;
  CloudPtr scene_;

  fawkes::Mutex cloud_mutex_;
  fawkes::Mutex bb_mutex_;

  std::atomic_bool have_laser_line_;

  fawkes::RefPtr<Cloud> cloud_out_raw_;
  fawkes::RefPtr<Cloud> cloud_out_trimmed_;
  fawkes::RefPtr<Cloud> cloud_out_model_;

  std::unique_ptr<fawkes::tf::Stamped<fawkes::tf::Pose>> result_pose_;
  std::atomic<double> result_fitness_;

  fawkes::tf::Stamped<fawkes::tf::Pose> initial_guess_laser_odom_;

  bool cfg_debug_mode_;

  const std::string syncpoint_clouds_ready_name;

private:
  // cfg values
  std::string cfg_if_prefix_;
  std::string cloud_in_name_;
  const std::string cloud_out_raw_name_;
  const std::string cloud_out_trimmed_name_;
  std::string cfg_bb_realsense_switch_name_;
  std::string conveyor_frame_id_;
  std::vector<std::string> laserlines_names_;

  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_clouds_ready;

  CloudPtr default_model_;
  CloudPtr trimmed_scene_;

  fawkes::ConveyorPoseInterface::MPS_TYPE current_mps_type_;
  fawkes::ConveyorPoseInterface::MPS_TARGET current_mps_target_;

  bool is_target_shelf();
  bool cfg_force_shelf_;

  void update_station_information(fawkes::ConveyorPoseInterface::SetStationMessage &msg);

  //Mapping from {Type,Target} to its corresponding model path
  std::map<std::pair<fawkes::ConveyorPoseInterface::MPS_TYPE,fawkes::ConveyorPoseInterface::MPS_TARGET>, std::string> type_target_to_path_;

  // Mapping from station name to preprocessed pointcloud model
  std::map<std::pair<fawkes::ConveyorPoseInterface::MPS_TYPE,fawkes::ConveyorPoseInterface::MPS_TARGET>, CloudPtr> type_target_to_model_;

  RecognitionThread *recognition_thread_;

  bool cfg_record_model_;
  std::string cfg_default_model_path_;
  std::string cfg_model_origin_frame_;
  std::string cfg_record_path_;


  std::atomic<float> cfg_left_cut_;
  std::atomic<float> cfg_right_cut_;
  std::atomic<float> cfg_top_cut_;
  std::atomic<float> cfg_bottom_cut_;
  std::atomic<float> cfg_front_cut_;
  std::atomic<float> cfg_back_cut_;

  std::atomic<float> cfg_left_cut_no_ll_;
  std::atomic<float> cfg_right_cut_no_ll_;
  std::atomic<float> cfg_top_cut_no_ll_;
  std::atomic<float> cfg_bottom_cut_no_ll_;
  std::atomic<float> cfg_front_cut_no_ll_;
  std::atomic<float> cfg_back_cut_no_ll_;

  std::atomic<float> cfg_shelf_left_cut_;
  std::atomic<float> cfg_shelf_right_cut_;
  std::atomic<float> cfg_shelf_top_cut_;
  std::atomic<float> cfg_shelf_bottom_cut_;
  std::atomic<float> cfg_shelf_front_cut_;
  std::atomic<float> cfg_shelf_back_cut_;

  std::atomic<float> cfg_shelf_left_cut_no_ll_;
  std::atomic<float> cfg_shelf_right_cut_no_ll_;
  std::atomic<float> cfg_shelf_top_cut_no_ll_;
  std::atomic<float> cfg_shelf_bottom_cut_no_ll_;
  std::atomic<float> cfg_shelf_front_cut_no_ll_;
  std::atomic<float> cfg_shelf_back_cut_no_ll_;

  std::atomic<float> cfg_voxel_grid_leaf_size_;

  std::map<fawkes::ConveyorPoseInterface::MPS_TARGET, std::array<std::atomic<float>, 3>> cfg_target_hint_;
  std::map<fawkes::ConveyorPoseInterface::MPS_TYPE, std::array<std::atomic<float>, 3>> cfg_type_offset_;

  // state vars
  bool cfg_enable_switch_;
  bool cloud_in_registered_;
  pcl::PCLHeader header_;

  // point clouds from pcl_manager
  fawkes::RefPtr<const Cloud> cloud_in_;

  // interfaces write
  fawkes::SwitchInterface *bb_enable_switch_;
  fawkes::ConveyorPoseInterface *bb_pose_;

  // interfaces read
  std::vector<fawkes::LaserLineInterface * > laserlines_;
  fawkes::SwitchInterface *realsense_switch_;
  fawkes::Time wait_start_;
  fawkes::Time wait_time_;

//  fawkes::Position3DInterface * bb_tag_;

 /**
  * check if the pointcloud is available
  */
 bool update_input_cloud();

 void bb_update_switch();
 bool laserline_get_best_fit(fawkes::LaserLineInterface * &best_fit);
 Eigen::Vector3f laserline_get_center_transformed(fawkes::LaserLineInterface * ll);
 fawkes::tf::Stamped<fawkes::tf::Pose> laserline_get_center(fawkes::LaserLineInterface *ll);

 void set_initial_tf_from_laserline(fawkes::LaserLineInterface *ll, fawkes::ConveyorPoseInterface::MPS_TYPE mps_type,fawkes::ConveyorPoseInterface::MPS_TARGET mps_target);

 bool is_inbetween(double a, double b, double val);

 CloudPtr cloud_trim(CloudPtr in, fawkes::LaserLineInterface * ll, bool use_ll);

 boost::shared_ptr<std::vector<pcl::PointIndices>> cloud_cluster(CloudPtr in);
 CloudPtr cloud_voxel_grid(CloudPtr in);

 void pose_write();
 void record_model();

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

  void pose_publish_tf(const fawkes::tf::Stamped<fawkes::tf::Pose> &pose);
  void start_waiting();
  bool need_to_wait();

};

Eigen::Matrix4f pose_to_eigen(const fawkes::tf::Pose &pose);
fawkes::tf::Pose eigen_to_pose(const Eigen::Matrix4f &m);

#endif
