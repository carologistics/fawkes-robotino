#include "picture_taker_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <interfaces/PictureTakerInterface.h>
#include <dlfcn.h>
#include <pthread.h>
#include <string>
#include <unistd.h>

#define CFG_PREFIX "/plugins/picture_taker/"
#define IMAGE_CHANNELS 3


using namespace fawkes;
using namespace std;
using namespace cv;
/** @class MarkerlessRecognitionThread "markerlss_rec_thread.h"
 * Thread to print recognized MPS
 * @author Sebastian Schönitz, Daniel Habering, Carsten Stoffels
 */

/** Constructor. */
PictureTakerThread::PictureTakerThread()
  : Thread("PictureTakerThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
    fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;

}

void PictureTakerThread::finalize() {
 	blackboard->close(p_t_if_);
  delete fv_cam;
  fv_cam = NULL;
  delete shm_buffer;
  shm_buffer= NULL;
  image_buffer = NULL;
 	ipl = NULL;
}


void PictureTakerThread::init(){
	p_t_if_ = blackboard->open_for_writing<PictureTakerInterface>("PictureTaker");

	std::string prefix = CFG_PREFIX;
	vpath = this->config->get_string((prefix + "vpath").c_str());

	// init firevision camera
    	// CAM swapping not working (??)
    	if(fv_cam != NULL){
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
    	}


	// SHM image buffer
    	if(shm_buffer != NULL) {
        	delete shm_buffer;
        	shm_buffer = NULL;
        	image_buffer = NULL;
    	}

    	shm_buffer = new firevision::SharedMemoryImageBuffer(
        	        shm_id.c_str(),
                	firevision::YUV422_PLANAR,
                	this->img_width,
                	this->img_height
                	);
    	if(!shm_buffer->is_valid()){
       	 	delete shm_buffer;
        	delete fv_cam;
        	shm_buffer = NULL;
        	fv_cam = NULL;
        	throw fawkes::Exception("Shared memory segment not valid");
   	}

    	std::string vframe = this->config->get_string((prefix + "vframe").c_str());
    	shm_buffer->set_frame_id(vframe.c_str());

    	image_buffer = shm_buffer->buffer();
    	ipl =  cvCreateImage(
        	        cvSize(this->img_width,this->img_height),
                	IPL_DEPTH_8U,IMAGE_CHANNELS);

}



void PictureTakerThread::takePictureFromFVcamera(){ 

        logger->log_info(name(),"Taking Picture");
        fv_cam->capture();
        firevision::convert(fv_cam->colorspace(),
                                 firevision::YUV422_PLANAR,
                                 fv_cam->buffer(),
                                 image_buffer,
                                 this->img_width,
                                 this->img_height);
        fv_cam->dispose_buffer();
        //convert img
        firevision::IplImageAdapter::convert_image_bgr(image_buffer, ipl);
        visionMat = cvarrToMat(ipl);
        fawkes::Time now = fawkes::Time();
        std::string image_path = vpath + "_" + std::to_string(now.in_sec()) + ".jpg";
        imwrite(image_path.c_str(), visionMat);
}

void PictureTakerThread::loop(){
	  if(fv_cam == NULL || !fv_cam->ready()){
        	logger->log_info(name(),"Camera not ready");
		      init();
      	  return;
   	}
   	while ( ! p_t_if_->msgq_empty() ) {
		  if ( p_t_if_->msgq_first_is<PictureTakerInterface::TakeAPictureMessage>() ) {

        logger->log_info(name(), "Taking Picture");
        //MPSRecognitionInterface::TakeDataMessage *m = p_t_if_->msgq_first<MPSRecognitionInterface::TakeDataMessage>();
        //p_t_if_->set_msgid(m->id());

        logger->log_info(name(), "Start Recognition");
        takePictureFromFVcamera();
    	}
   	 	else {
    			logger->log_warn(name(), "Unknown message received");
   		}
		  p_t_if_->msgq_pop();
	  }
}

