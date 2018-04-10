#ifndef CORRESPONDENCE_GROUPING_THREAD_H
#define CORRESPONDENCE_GROUPING_THREAD_H

#include <core/threading/thread.h>
#include <aspect/logging.h>

#include "conveyor_pose_thread.h"

class CorrespondenceGroupingThread
    : public fawkes::Thread
    , public fawkes::LoggingAspect
{
public:
  CorrespondenceGroupingThread(ConveyorPoseThread *cp_thread);

  using pose = ConveyorPoseThread::pose;

  virtual void loop() override;

private:
  ConveyorPoseThread *main_thread_;
};

#endif // CORRESPONDENCE_GROUPING_THREAD_H
