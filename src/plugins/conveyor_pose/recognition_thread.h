#ifndef CORRESPONDENCE_GROUPING_THREAD_H
#define CORRESPONDENCE_GROUPING_THREAD_H

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <atomic>

#include "conveyor_pose_thread.h"

class RecognitionThread
    : public fawkes::Thread
    , public fawkes::LoggingAspect
    , public fawkes::TransformAspect
{
public:
  RecognitionThread(ConveyorPoseThread *cp_thread);

  using pose = ConveyorPoseThread::pose;

  virtual void loop() override;
  virtual void init() override;

private:
  ConveyorPoseThread *main_thread_;

  fawkes::tf::Stamped<fawkes::tf::Pose> initial_guess_icp_odom_;
  double initial_guess_tracked_fitness_;
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
public:
  MyPointRepresentation()
  {
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const override;
};


#endif // CORRESPONDENCE_GROUPING_THREAD_H
