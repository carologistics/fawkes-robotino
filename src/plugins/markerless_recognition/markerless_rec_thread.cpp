


#include "markerless_rec_thread.h"

#include <tf/types.h>
#include <interfaces/MPSRecognitionInterface.h>

using namespace fawkes;

/** @class MarkerlessRecognitionThread "markerlss_rec_thread.h"
 * Thread to print recognized MPS
 * @author Sebastian Schönitz, Daniel Habering, Carsten Stoffels
 */

/** Constructor. */
MarkerlessRecognitionThread::MarkerlessRecognitionThread()
  : Thread("MarkerlessRecognitionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void
MarkerlessRecognitionThread::init()
{
  pose_if_ = blackboard->open_for_reading<MPSRecognition>("Pose");
}

void
GenesisThread::finalize()
{
  blackboard->close(pose_if_);
}

void
MarkerlessRecognitionThread::loop()
{

  
  if (pose_if_->has_writer()) {
    pose_if_->read();
    double *r = pose_if_->rotation();
    tf::Quaternion pose_q(r[0], r[1], r[2], r[3]);
    logger->log_info(name(), "Pose: (%f,%f,%f)", pose_if_->translation(0),
                     pose_if_->translation(1), tf::get_yaw(pose_q));
  } else {
    logger->log_warn(name(), "No writer for pose interface");
  }

}
