


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

void
MarkerlessRecognitionThread::init()
{
  readImage();
  mps_rec_if_ = blackboard->open_for_writing<MPSRecognitionInterface>("MarkerlessRecognition");
}

void
MarkerlessRecognitionThread::finalize()
{
 blackboard->close(mps_rec_if_);
}

void
MarkerlessRecognitionThread::loop()
{
   while ( ! mps_rec_if_->msgq_empty() ) {
    if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::ClearMessage>() ) {
     	std::cout << "Recieved Clear Message" << std::endl;
    } else if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::ComputeMessage>() ) {
      	std::cout << "Recieved Compute Message" << std::endl;
    } else if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::TakeDataMessage>() ) {
    	std::cout << "Recieved Take Data Message" << std::endl;
    }
    mps_rec_if_->msgq_pop();
  }
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
MarkerlessRecognitionThread::writeTotalProbability(Probability prob)
{
}

void 
MarkerlessRecognitionThread::writeWinProbability(float prob)
{
}

void
MarkerlessRecognitionThread::writeState(bool state)
{
}

void
MarkerlessRecognitionThread::writeWinMPS(MPSType mps)
{
}


