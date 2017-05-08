


#include "markerless_rec_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <tf/types.h>
#include <interfaces/MPSRecognitionInterface.h>

using namespace fawkes;
using namespace std;
using namespace cv;
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

void MarkerlessRecognitionThread::clear_data()
{
}

void
MarkerlessRecognitionThread::finalize()
{
 blackboard->close(mps_rec_if_);
}



void MarkerlessRecognitionThread::estimate_mps_type(const Probability &prob) const {
	float pmax = 0.;
	float psec = 0.;
	MPSType bestfit;
	for (int i = 0; i < 5; ++i) {
		if (pmax < prob.p[i]) {
			psec = pmax;
			bestfit = (MPSType) i;
			pmax = prob.p[i];
		}
	}

	if(pmax < th_first || psec > th_sec){
		bestfit = NoStationDetected;
	}

	mps_rec_if_->set_final(true);
	mps_rec_if_->set_mpstype(
	        (fawkes::MPSRecognitionInterface::MPSType) bestfit);
	mps_rec_if_->set_p_correct(pmax);
	mps_rec_if_->set_ptot_mpstype(prob.p);
	mps_rec_if_->write();
}

Probability MarkerlessRecognitionThread::recognize_current_pic(std::string image) {
	Probability result;

	return result;
}

void MarkerlessRecognitionThread::recognize_mps() {

}

void MarkerlessRecognitionThread::readImage(){ 
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

void
MarkerlessRecognitionThread::init()
{
  mps_rec_if_ = blackboard->open_for_writing<MPSRecognitionInterface>("/MarkerlessRecognition");
  clear_data();
}



void
MarkerlessRecognitionThread::loop()
{
   while ( ! mps_rec_if_->msgq_empty() ) {
    if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::ClearMessage>() ) {
     	std::cout << "Recieved Clear Message" << std::endl;
	clear_data();
    } else if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::ComputeMessage>() ) {
      	std::cout << "Recieved Compute Message" << std::endl;
 	logger->log_info(name(), "Recognition started");
	MPSRecognitionInterface::TakeDataMessage *m = mps_rec_if_
		->msgq_first<MPSRecognitionInterface::TakeDataMessage>();
	mps_rec_if_->set_msgid(m->id());
	mps_rec_if_->set_final(false);
	mps_rec_if_->write();
	recognize_mps();
    } else if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::TakeDataMessage>() ) {
    	std::cout << "Recieved Take Data Message" << std::endl;
   	readImage();
    }	
    else {
    	logger->log_warn(name(), "Unknown message received");
    }
    mps_rec_if_->msgq_pop();
  }
}


