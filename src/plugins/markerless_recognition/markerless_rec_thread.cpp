


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
