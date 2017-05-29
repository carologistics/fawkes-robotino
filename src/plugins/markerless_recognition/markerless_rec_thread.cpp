


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

Probability MarkerlessRecognitionThread::recognize_current_pic(const std::string image) {
	Probability result;

	// struct with 5 float values
	// every time: bs, cs, ds, rs, ss

	return result;
}

void  MarkerlessRecognitionThread::recognize_mps() {

        mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) 0 ) ;
	mps_rec_if_->write();
	
	const char* recognizedMPS = mps_rec_if_->tostring_MPSType(mps_rec_if_->mpstype());
 	cout << " Recognized MPS : " << recognizedMPS << std::endl; 	
	//iterates over all stored paths im imageSet_ and calls recognize_current_pic on it
	//then decides which mps was seen in the set of images
}

	
void MarkerlessRecognitionThread::readImage(){ 
 
       	
 	Mat image = imread("/home/casto/Carologistics/OpenTC/Training/ExperimentalTrainingData/BS/BS_Depth_2682.jpg") ; //Read the file , dev/video_tag should be the path for the RealSense

	if(! image.data )                              // Check for invalid input
   	{
        	cout <<  "Could not open or find the image" << std::endl ;
   	}
	try{ // to show , doesn't work (black screen) 
    		
	 	double min;
        	double max;
       		cv::minMaxIdx(image, &min, &max);
        	cv::Mat adjMap;
        	cv::convertScaleAbs(image, adjMap, 255 / max);
        	//cv::imshow("Out", adjMap);
	}
	catch( cv::Exception& e )
	{
    		const char* err_msg = e.what();
  		std::cout << "exception caught: " << err_msg << std::endl;
	}
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


