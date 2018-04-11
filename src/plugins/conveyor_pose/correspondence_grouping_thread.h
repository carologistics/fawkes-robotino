#ifndef CORRESPONDENCE_GROUPING_THREAD_H
#define CORRESPONDENCE_GROUPING_THREAD_H

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <aspect/logging.h>
#include <atomic>

#include "conveyor_pose_thread.h"

class CorrespondenceGroupingThread
    : public fawkes::Thread
    , public fawkes::LoggingAspect
{
public:
  CorrespondenceGroupingThread(ConveyorPoseThread *cp_thread);

  using pose = ConveyorPoseThread::pose;

  virtual void loop() override;

  void set_running(bool);

private:
  ConveyorPoseThread *main_thread_;
  fawkes::WaitCondition wait_enabled_;
  std::atomic_bool running_;
};

#endif // CORRESPONDENCE_GROUPING_THREAD_H
