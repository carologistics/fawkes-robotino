#include "markerless_rec_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <tf/types.h>
#include <interfaces/MPSRecognitionInterface.h>
//#include "/home/Sagre/tensorflow/tensorflow/c/c_api.h"
#include <dlfcn.h>
#include <pthread.h>
#include <string>


#define CFG_PREFIX "/plugins/conveyor_vision/"
#define IMAGE_CHANNELS 3


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
    VisionAspect(VisionAspect::CYCLIC),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE), 
    ConfigurationChangeHandler(CFG_PREFIX),
    fawkes::TransformAspect(fawkes::TransformAspect::ONLY_PUBLISHER,"conveyor")
{
    fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;

}


// config handling
void MarkerlessRecognitionThread::config_value_erased(const char *path) {};
void MarkerlessRecognitionThread::config_tag_changed(const char *new_tag) {};
void MarkerlessRecognitionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
void MarkerlessRecognitionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v){}; 


void MarkerlessRecognitionThread::finalize() {
 	blackboard->close(mps_rec_if_);
        delete fv_cam;
  	fv_cam = NULL;
  	delete shm_buffer;
  	shm_buffer= NULL;
  	image_buffer = NULL;
 	ipl = NULL;
}

	

Probability MarkerlessRecognitionThread::recognize_current_pic(const std::string image) {
	
        std::cout << "recognize current pic" << std::endl;

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


//	if(checkProbability(result)==-1){
//		std::cout << "Classification failed\n";
//		return result;
//	}
//	else{
//		for(int i = 0; i < 5; ++i){
//			std::cout << "Result: " << result.p[i] << std::endl;
//		}	
//			estimate_mps_type(result);
//	}
	
	//Still bugged. Seg fault if we try to call dlclose()
	//dlclose(handle);
   
	// struct with 5 float values
	// every time: bs, cs, ds, rs, ss 
        std::cout << "Result " <<  result.p[0] << std::endl;

	return result;
}

float  MarkerlessRecognitionThread::recognize_mps() {


	takePictureFromFVcamera(); 
        Probability recognition_result = recognize_current_pic(frameToRecognize); 
	
	float maximum = 0; 
	for(int i = 0; i < 4; i++){ 
	 	if(recognition_result.p[i] < recognition_result.p[i+1]){ 
                     maximum = i+1; 
	     	}
	}

        mps_rec_if_->set_final(true);
	mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) maximum );
	mps_rec_if_->write();	

	return maximum;
	//cout << " Start of method recognize_mps() " << std::endl; 
        //mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) 5 ) ;
	//mps_rec_if_->write();
	
	//const char* recognizedMPS = mps_rec_if_->tostring_MPSType(mps_rec_if_->mpstype());
 	//cout << " Recognized MPS : " << recognizedMPS << std::endl; 	
	//iterates over all stored paths im imageSet_ and calls recognize_current_pic on it
	//then decides which mps was seen in the set of images
}


void MarkerlessRecognitionThread::init(){
      	
	home.assign(getenv("HOME"),strlen(getenv("HOME")));
       	recognize_current_pic("/TestData/BS/BS_9.jpg");

       	mps_rec_if_ = blackboard->open_for_writing<MPSRecognitionInterface>("/MarkerlessRecognition");
       	//clear_data();
         

}

void MarkerlessRecognitionThread::setupCamera(){ 


   std::string prefix = CFG_PREFIX;
   std::string mps_cascade_name = (string)config->get_string((prefix + "classifier_file"));
   if( !mps_cascade.load( std::string(CONFDIR) + "/" + mps_cascade_name ) ){ printf("--(!)Error loading\n"); return; };

	
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
    frameToRecognize = this->config->get_string((prefix + "frame").c_str());
    shm_buffer->set_frame_id(frameToRecognize.c_str());

    image_buffer = shm_buffer->buffer();
    ipl =  cvCreateImage(
                cvSize(this->img_width,this->img_height),
                IPL_DEPTH_8U,IMAGE_CHANNELS);



}

void MarkerlessRecognitionThread::takePictureFromFVcamera(){ 

	//capture 
	fv_cam->capture(); 
	firevision::convert(fv_cam->colorspace(), 
			    firevision::YUV422_PLANAR,
			    fv_cam->buffer(),
			    image_buffer,
			    this->img_width,
			    this->img_height);
	fv_cam->dispose_buffer();

	//convert 
	
	firevision::IplImageAdapter::convert_image_bgr(image_buffer, ipl);
        frame  = cvarrToMat(ipl);


}


void MarkerlessRecognitionThread::loop(){

   	if(fv_cam == NULL || !fv_cam->ready()){
        	logger->log_info(name(),"Camera not ready");
		setupCamera();
      	return;
   	}

    
   	while ( ! mps_rec_if_->msgq_empty() ) {
     			
    		
		if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::ComputeMessage>() ) {
      			
			std::cout << "Recieved Compute Message" << std::endl;
 			logger->log_info(name(), "Recognition started");
			MPSRecognitionInterface::TakeDataMessage *m = mps_rec_if_->msgq_first<MPSRecognitionInterface::TakeDataMessage>();
			mps_rec_if_->set_msgid(m->id());
			mps_rec_if_->set_final(false);
			mps_rec_if_->write();
			cout << " Start recognize " << std::endl; 
			recognize_mps();
    		
		} else if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::TakeDataMessage>() ) {
    	
	 		std::cout << "Recieved Take Data Message" << std::endl;
   			takePictureFromFVcamera(); 
			//readImage();
    		}	
   	 	else {
    			logger->log_warn(name(), "Unknown message received");
    		}
    			mps_rec_if_->msgq_pop();
    
  		}
}

/*
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
}*/

