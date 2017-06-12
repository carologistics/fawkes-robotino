


#include "markerless_rec_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <tf/types.h>
#include <interfaces/MPSRecognitionInterface.h>
#include "/home/Sagre/tensorflow/tensorflow/c/c_api.h"
#include <dlfcn.h>
#include <pthread.h>
#include <string>


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
  /*  fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;
*/
}

void MarkerlessRecognitionThread::clear_data()
{
}

void
MarkerlessRecognitionThread::finalize()
{
 blackboard->close(mps_rec_if_);
/*  delete fv_cam;
  fv_cam = NULL;
  delete shm_buffer;
  shm_buffer= NULL;
  image_buffer = NULL;
  ipl = NULL;*/
}

int MarkerlessRecognitionThread::checkProbability(Probability prob){
	for(int i = 0; i<5; ++i){
		if(prob.p[i]<0) return -1;

	}
	return 0;
}

void MarkerlessRecognitionThread::estimate_mps_type(const Probability &prob) {
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
	std::cout << "MPS: " << bestfit << std::endl;

	//segfault when trying to write to the blackboard
	
//	mps_rec_if_->set_final(true);
//	mps_rec_if_->set_mpstype(
//	        (fawkes::MPSRecognitionInterface::MPSType) bestfit);
//	mps_rec_if_->set_p_correct(pmax);
	//mps_rec_if_->set_ptot_mpstype(prob.p);
	//mps_rec_if_->write();
	
}

Probability MarkerlessRecognitionThread::recognize_current_pic(const std::string image) {
	Probability result;
	my_function evaluate;
	void *handle;
  
	//Open shared library
	std::string lib = home + "/fawkes-robotino/lib/tensorflowWrapper.so";
    	handle = dlopen(lib.c_str(),RTLD_NOW);
	if(!handle){
		fprintf(stderr, "%s\n", dlerror());
		return result;
	}

	//set function pointer
        *(void**)(&evaluate) = dlsym(handle,"evaluateImage");
	char* error;
	if((error=dlerror())!=NULL) {
		fprintf(stderr, "%s\n", error);
		return result;		
	}
	
	//path to the image that have to be tested
	std::string imPath = (home) + image;
		
	//path to the trained graph
	std::string grPath = (home) + "/TrainedData/output_graph_600.pb";
	
	//path to the the trained labels
	std::string laPath = (home) + "/TrainedData/output_labels_600.txt";

	//evaluates the current image
        result = evaluate(imPath.c_str(), grPath.c_str(), laPath.c_str());


	if(checkProbability(result)==-1){
		std::cout << "Classification failed\n";
		return result;
	}
	else{
		for(int i = 0; i < 5; ++i){
			std::cout << "Result: " << result.p[i] << std::endl;
		}
		estimate_mps_type(result);
	}
	
	//Still bugged. Seg fault if we try to call dlclose()
	//dlclose(handle);
   
	// struct with 5 float values
	// every time: bs, cs, ds, rs, ss
	return result;
}

void  MarkerlessRecognitionThread::recognize_mps() {

	cout << " Start of method recognize_mps() " << std::endl; 
        mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) 5 ) ;
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
  home.assign(getenv("HOME"),strlen(getenv("HOME")));
	
   recognize_current_pic("/TestData/BS/BS_9.jpg");

  mps_rec_if_ = blackboard->open_for_writing<MPSRecognitionInterface>("/MarkerlessRecognition");
  clear_data();
}

void MarkerlessRecognitionThread::setupCamera(){ 
/*       
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
    std::string frame = this->config->get_string((prefix + "frame").c_str());
    shm_buffer->set_frame_id(frame.c_str());

    image_buffer = shm_buffer->buffer();
    ipl =  cvCreateImage(
                cvSize(this->img_width,this->img_height),
                IPL_DEPTH_8U,IMAGE_CHANNELS);


    // set up marker
    world_pos_z_average = 0.;
    max_marker = 16;
    // this->markers_ = new std::vector<alvar::MarkerData>(); 
    // this->tag_interfaces = new TagPositionList(this->blackboard,this->max_marker,frame,this->name(),this->logger, this->clock, this->tf_publisher);
*/
}


void
MarkerlessRecognitionThread::loop(){
/*
   if(fv_cam == NULL || !fv_cam->ready()){
        logger->log_info(name(),"Camera not ready");
	setupCamera();
      return;
   }
*/
    
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
	cout << " Start recognize " << std::endl; 
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


