#include "image_rec_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <tf/types.h>
#include <interfaces/ImageRecognitionInterface.h>
#include <dlfcn.h>
#include <pthread.h>
#include <string>
#include <unistd.h>

#include <fstream>
#include <iterator>
#include <algorithm>
#define CFG_PREFIX "/plugins/image_recognition/"
#define IMAGE_CHANNELS 3


using namespace fawkes;
using namespace std;
using namespace cv;

/** @class ImageRecognitionThread "image_rec_thread.h"
 * Thread to print recognized MPS
 * @author Sebastian Schönitz, Daniel Habering, Carsten Stoffels
 */


/** Constructor. */
ImageRecognitionThread::ImageRecognitionThread()
  : Thread("ImageRecognitionThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC),
    ConfigurationChangeHandler(CFG_PREFIX),
    fawkes::TransformAspect(fawkes::TransformAspect::ONLY_PUBLISHER,"image_recognition")
{
    fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;
	
}

void ImageRecognitionThread::finalize() {
	//printf("Closing Plugin\n");
	blackboard->close(mps_rec_if_);
        delete fv_cam;
  	fv_cam = NULL;
  	delete shm_buffer;
  	shm_buffer= NULL;
  	image_buffer = NULL;
 	ipl = NULL;
	dlclose(handle);
	//printf("Do I get here?\n");
}
int ImageRecognitionThread::checkResult(Probability prob){
	for(int i = 0; i < CLASS_COUNT; i++){

		if(prob.p[i]<0 || prob.p[i]>1){
			return -1;
		}
	}
	return 0;
}
	
Probability ImageRecognitionThread::recognize_current_pic(const std::string image) {
	Probability result(CLASS_COUNT);
	
	for(int i = 0; i < CLASS_COUNT; i++){
		result.p[i]=0.;
	}
	//path to the image that have to be tested
	std::string imPath = image;		
	

	//TODO

	//evaluates the current image
	std::vector<float> retVec = evaluate(imPath.c_str(), graph.c_str(), labels.c_str());

	Probability ret(CLASS_COUNT);
	for(int i = 0; i < CLASS_COUNT; i++){
		ret.p[i] = retVec[i];
	}


	if(checkResult(ret)==0){
		result = ret;
	}
	else {
		logger->log_info(name(), "Classification failed");
	}

	return result;
}


int ImageRecognitionThread::recognize() {

	takePictureFromFVcamera(); 
	if(vpath.empty()) return -1;

        //Probability recognition_result = recognize_current_pic(vpath); 
	Probability recognition_result = recognize_current_pic("/home/Sagre/Validation/CS-I/vision_14.jpg");
	int recResult = 0;

	int maximum = 0; 
	int second = CLASS_COUNT-1;
	for(int i = 0; i < CLASS_COUNT; i++){ 
	 	if(i==maximum) continue;
		if(recognition_result.p[i] >= recognition_result.p[maximum]){ 
		 	second = maximum;
			maximum = i;
	     	}
		else {
		       	if(recognition_result.p[i] > recognition_result.p[second]){
				second = i;
			}
		}
	}

	if(checkResult(recognition_result)!=0){
		mps_rec_if_->set_final(true);
		mps_rec_if_->set_recclass((fawkes::ImageRecognitionInterface::RecClass) 0);
		mps_rec_if_->write();
		return -1;
	}

	if(recognition_result.p[maximum] < THRESHOLD_UPPER || recognition_result.p[second] > THRESHOLD_LOWER)
	{
		logger->log_info(name(),"Failed Threshold: %f %f",recognition_result.p[maximum],recognition_result.p[second]);
		recResult = 0;
	        mps_rec_if_->set_final(true);
    		mps_rec_if_->set_recclass((fawkes::ImageRecognitionInterface::RecClass) recResult) ;	
    		mps_rec_if_->write();	
		return recResult;
	}
	else{
		recResult = maximum;
	}	

        mps_rec_if_->set_final(true);
	mps_rec_if_->set_recclass((fawkes::ImageRecognitionInterface::RecClass) (recResult+1));
	logger->log_info(name(), " Finished recognizing: %s ",classes[recResult].c_str()); 
	mps_rec_if_->write();	
	
	return recResult;

}


void ImageRecognitionThread::init(){
      	
	home.assign(getenv("HOME"),strlen(getenv("HOME")));   
	mps_rec_if_ = blackboard->open_for_writing<ImageRecognitionInterface>("/ImageRecognition");

	std::string prefix = CFG_PREFIX;
	vpath = this->config->get_string((prefix + "vpath").c_str()); 

	graph = this->config->get_string((prefix + "graph").c_str());
	labels = this->config->get_string((prefix + "labels").c_str());

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

	//Open shared library
	std::string lib = home + this->config->get_string((prefix + "tensorflow_lib").c_str());
    	handle = dlopen(lib.c_str(),RTLD_LAZY|RTLD_GLOBAL);
	if(!handle){
        		fprintf(stderr, "%s\n", dlerror());
		
	}

	
	//set function pointer
        *(void**)(&evaluate) = dlsym(handle,"evaluateImage");
	char* error;
	if((error=dlerror())!=NULL) {
		fprintf(stderr, "%s\n", error);
	}

	init_function initGraph;

	*(void**)(&initGraph) = dlsym(handle,"initGraph");
	if((error=dlerror())!=NULL) {
		fprintf(stderr, "%s\n", dlerror());
	}
	

    	graph = (home) + graph;
	labels = (home) + labels;


	//Read classes from labels file
	ifstream labelsFile(labels);
	
	istreambuf_iterator<char> i_file(labelsFile);
	istreambuf_iterator<char> eof;

	std::string buffer;

	while (i_file != eof){
	
		if(*i_file == '\n')
		{
			std::string str(buffer);
			classes.push_back(str);
			buffer.clear();
		}
		else {
			buffer += *i_file;
		}
		
		++i_file;

	}
	CLASS_COUNT  = classes.size();
	
	//Init Tensorflow 
	initGraph(graph.c_str(),labels.c_str());
	
	//Take 4 pictures to overcome intial realsense misbehaviour
	takePictureFromFVcamera();
	takePictureFromFVcamera();
	takePictureFromFVcamera();
	takePictureFromFVcamera();
	recognize();
}



void ImageRecognitionThread::takePictureFromFVcamera(){ 

	
	//get img form fv
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
        imwrite(vpath.c_str(), visionMat);
		

}

void ImageRecognitionThread::loop(){
   	
	if(fv_cam == NULL || !fv_cam->ready()){
        	logger->log_info(name(),"Camera not ready");
		init(); 
      	return;
   	}

   	while ( ! mps_rec_if_->msgq_empty() ) {
     			
		if ( mps_rec_if_->msgq_first_is<ImageRecognitionInterface::RecognitionMessage>() ) {
      			

 			logger->log_info(name(), "Received Recognition Message");
		
			mps_rec_if_->set_final(false);
			mps_rec_if_->write();

 			logger->log_info(name(), "Start Recognition");
			recognize();
    		
    		}	
   	 	else {
    			logger->log_warn(name(), "Unknown message received");

    		}
    			
		mps_rec_if_->msgq_pop();
	}
}

