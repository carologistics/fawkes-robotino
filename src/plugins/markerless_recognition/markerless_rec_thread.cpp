


#include "markerless_rec_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
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
  compute_if_ = blackboard->open_for_reading<MPSRecognitionInterface>("Compute");
}

void
MarkerlessRecognitionThread::finalize()
{
 // blackboard->close(compute_if_);
}

void
MarkerlessRecognitionThread::loop()
{

  
 /* if (compute_if_->has_writer()) {
    compute_if_->read();
    double *r = compute_if_->rotation();
    tf::Quaternion pose_q(r[0], r[1], r[2], r[3]);
    logger->log_info(name(), "Pose: (%f,%f,%f)", compute_if_->translation(0),
                     compute_if_->translation(1), tf::get_yaw(pose_q));
  } else {
    logger->log_warn(name(), "No writer for pose interface");
  }
*/
}

void MarkerlessRecognitionThrad::readImage(){ 
	 /* 
	Mat image;
  	image = imread("/home/casto/Carologistic/OpenTC/Training/ExperimentalTestData/BS_Depth_2682.jpg");   // Read the filemi, dev/video_tag should be the path for the RealSense

	// Show it for testing reasons
	namedWindow("image", CV_WINDOW_AUTOSIZE);
    	imshow("image", grey);
	*/
	    // init firevision camera
   	 // CAM swapping not working (??)
        /*if(fv_cam != NULL){
        // free the camera
        fv_cam->stop();
        fv_cam->flush();
        fv_cam->dispose_buffer();
        fv_cam->close();
        delete fv_cam;
        fv_cam = NULL;
    }
    if(fv_cam == NULL){
      std::string connection = this->config->get_string((prefix + "camera").c_str());
        fv_cam = vision_master->register_for_camera(connection.c_str(), this);
        fv_cam->start();
        fv_cam->open();
        this->img_width = fv_cam->pixel_width();
        this->img_height = fv_cam->pixel_height();
    }*/

}
