#ifndef CORRESPONDENCE_GROUPING_THREAD_H
#define CORRESPONDENCE_GROUPING_THREAD_H

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/syncpoint_manager.h>

#include <atomic>

#include <pcl/registration/icp_nl.h>
#include <pcl/recognition/hv/hv_papazov.h>

#include "conveyor_pose_thread.h"


class CustomICP : public pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> {
public:
  using pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>::IterativeClosestPointNonLinear;

  double getScaledFitness();
};


class RecognitionThread
    : public fawkes::Thread
    , public fawkes::LoggingAspect
    , public fawkes::TransformAspect
    , public fawkes::SyncPointManagerAspect
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Point = ConveyorPoseThread::Point;
  using Cloud = ConveyorPoseThread::Cloud;
  using CloudPtr = ConveyorPoseThread::CloudPtr;

  RecognitionThread(ConveyorPoseThread *cp_thread);

  virtual void loop() override;
  virtual void init() override;
  void enable();
  void disable();
  void restart();
  bool enabled();

  std::atomic<float> cfg_icp_max_corr_dist_;
  std::atomic<double> cfg_icp_tf_epsilon_;
  std::atomic<double> cfg_icp_refinement_factor_;
  std::array<std::atomic<float>, 3> cfg_icp_conveyor_hint_;
  std::atomic<int> cfg_icp_max_iterations_;
  std::atomic<float> cfg_icp_hv_penalty_thresh_;
  std::atomic<float> cfg_icp_hv_support_thresh_;
  std::atomic<float> cfg_icp_hv_inlier_thresh_;
  std::atomic<float> cfg_icp_shelf_hv_penalty_thresh_;
  std::atomic<float> cfg_icp_shelf_hv_support_thresh_;
  std::atomic<float> cfg_icp_shelf_hv_inlier_thresh_;
  std::atomic<unsigned int> cfg_icp_min_loops_;
  std::atomic<unsigned int> cfg_icp_max_loops_;
  std::atomic_bool cfg_icp_auto_restart_;

private:
  void restart_icp();
  void publish_result();
  void constrainTransformToGround(fawkes::tf::Stamped<fawkes::tf::Pose>& fittedPose_conv);

  ConveyorPoseThread *main_thread_;

  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_clouds_ready_;

  std::atomic_bool enabled_;
  std::atomic_bool do_restart_;

  fawkes::tf::Stamped<fawkes::tf::Pose> initial_guess_icp_odom_;
  Eigen::Matrix4f initial_tf_;
  CloudPtr model_;
  CloudPtr scene_;

  CustomICP icp_;
  pcl::PapazovHV<Point, Point> hypot_verif_;
  Eigen::Matrix4f prev_last_tf_;
  CloudPtr icp_result_;
  Eigen::Matrix4f final_tf_;

  unsigned int iterations_;
  double last_raw_fitness_;
};


#endif // CORRESPONDENCE_GROUPING_THREAD_H
