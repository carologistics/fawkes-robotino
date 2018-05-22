#ifndef CORRESPONDENCE_GROUPING_THREAD_H
#define CORRESPONDENCE_GROUPING_THREAD_H

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
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
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Point = ConveyorPoseThread::Point;
  using Cloud = ConveyorPoseThread::Cloud;
  using CloudPtr = ConveyorPoseThread::CloudPtr;
  using pose = ConveyorPoseThread::pose;

  RecognitionThread(ConveyorPoseThread *cp_thread);

  virtual void loop() override;
  virtual void init() override;
  void enable();
  void disable();
  void restart();

private:
  void restart_icp();
  void publish_result();
  void constrainTransformToGround(fawkes::tf::Stamped<fawkes::tf::Pose>& fittedPose_conv);

  ConveyorPoseThread *main_thread_;

  fawkes::WaitCondition wait_enabled_;
  std::atomic_bool enabled_;
  std::atomic_bool do_restart_;

  fawkes::tf::Stamped<fawkes::tf::Pose> initial_guess_icp_odom_;
  Eigen::Matrix4f initial_tf_;
  CloudPtr model_with_normals_;
  CloudPtr scene_with_normals_;

  CustomICP icp_;
  pcl::PapazovHV<Point, Point> hypot_verif_;
  Eigen::Matrix4f prev_last_tf_;
  CloudPtr icp_result_;
  Eigen::Matrix4f final_tf_;

  unsigned int iterations_;
  double last_raw_fitness_;
};


#endif // CORRESPONDENCE_GROUPING_THREAD_H
