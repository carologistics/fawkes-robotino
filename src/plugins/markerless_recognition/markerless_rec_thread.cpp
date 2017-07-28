#include "markerless_rec_thread.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <tf/types.h>
#include <interfaces/MPSRecognitionInterface.h>
//#include "/home/Sagre/tensorflow/tensorflow/c/c_api.h"
#include <dlfcn.h>
#include <pthread.h>
#include <string>
#include <unistd.h>

#define CFG_PREFIX "/plugins/markerless_recognition/"
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
    ConfigurationChangeHandler(CFG_PREFIX),
    fawkes::TransformAspect(fawkes::TransformAspect::ONLY_PUBLISHER,"markerless_reocognition")
{
    fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;

}

void MarkerlessRecognitionThread::finalize() {
 	blackboard->close(mps_rec_if_);
        delete fv_cam;
  	fv_cam = NULL;
  	delete shm_buffer;
  	shm_buffer= NULL;
  	image_buffer = NULL;
 	ipl = NULL;
}
int checkResult(Probability prob){
	for(int i = 0; i < MPS_COUNT; i++){

		if(prob.p[i]<0 || prob.p[i]>1){
			return -1;
		}
	}
	return 0;
}
	
Probability MarkerlessRecognitionThread::recognize_current_pic(const std::string image) {
	
	
	Probability result;
	for(int i = 0; i < MPS_COUNT; i++){
		result.p[i]=0.;
	}
	//path to the image that have to be tested
	std::string imPath = image;		
	
	//path to the trained graph
    	std::string grPath = (home) + "/tensorflow/TrainedData/output_graph_CS.pb";
	
	//path to the the trained labels
	std::string laPath = (home) + "/tensorflow/TrainedData/output_labels_CS.txt";

	//evaluates the current image
	std::vector<float> retVec = evaluate(MPS_COUNT,imPath.c_str(), grPath.c_str(), laPath.c_str(),false);
	
	for(int i = 0; i < (int)retVec.size(); i++){
		std::cout << "retVec: " << retVec[i] << std::endl;
	}
	Probability ret;
	for(int i = 0; i < MPS_COUNT; i++){
		ret.p[i] = retVec[i];
	}


	if(checkResult(ret)==0){
		for(int i = 0; i < MPS_COUNT; ++i){
 			logger->log_info(name(), "Result: %f", ret.p[i] );
		}	
		result = ret;
	}
	else {
		logger->log_info(name(), "Classification failed");
	}

	return result;
}

Probability MarkerlessRecognitionThread::recheck_mps(const std::string image){
	logger->log_info(name(), "Started further recognition, to distinguish between RS and CS");
	Probability result;
	for(int i = 0; i < MPS_COUNT; i++){
		result.p[i]=0.;
	}
	
	//path to the image that have to be tested
	std::string imPath = image;		
	
	//path to the trained graph
    	std::string grPath = (home) + "/tensorflow/TrainedData/output_graph_RSCS.pb";
	
	//path to the the trained labels
	std::string laPath = (home) + "/tensorflow/TrainedData/output_labels_RSCS.txt";

	//evaluates the current image

	std::vector<float> retVec = evaluate(MPS_COUNT,imPath.c_str(), grPath.c_str(), laPath.c_str(),true);
	Probability ret;
	for(int i = 0; i < MPS_COUNT; i++){
		ret.p[i] = retVec[i];
	}

	if(checkResult(ret)==0){
		for(int i = 0; i < MPS_COUNT; ++i){
 			logger->log_info(name(), "Result: %f", ret.p[i] );
		}	
		result = ret;
	}
	else {
		logger->log_info(name(), "Classification failed");
	}

	return result;

}

int MarkerlessRecognitionThread::recognize_mps() {

	logger->log_info(name(), " Start of recognize_mps"); 

	takePictureFromFVcamera(); 
	if(vpath.empty()) return -1;

        Probability recognition_result = recognize_current_pic(vpath); 
	int station = 0;

	int maximum = 0; 
	int second = MPS_COUNT-1;
	for(int i = 0; i < MPS_COUNT; i++){ 
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
		mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) 0);
		mps_rec_if_->write();
		return -1;
	}

	if(recognition_result.p[maximum] < THRESHOLD_UPPER || recognition_result.p[second] > THRESHOLD_LOWER)
	{
		logger->log_info(name(),"Failed Threshold: %f %f",recognition_result.p[maximum],recognition_result.p[second]);
		station = 0;
	}
	else{
		/*if(maximum == CS || maximum == RS){
			recognition_result = recheck_mps(vpath);
			for(int i = 0; i < MPS_COUNT; i++){ 
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
				mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) 0);
				mps_rec_if_->write();
				return -1;
			}

		}
*/
		station = maximum+1;
	}	

        mps_rec_if_->set_final(true);
	if(MPS_COUNT>5) 	mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) ((int)(station+1)/2) );
	else mps_rec_if_->set_mpstype((fawkes::MPSRecognitionInterface::MPSType) (station));
	mps_rec_if_->write();	
	
	logger->log_info(name(), " Finished recognizing "); 
	return station;

}


void MarkerlessRecognitionThread::init(){
      	
	home.assign(getenv("HOME"),strlen(getenv("HOME")));   
	mps_rec_if_ = blackboard->open_for_writing<MPSRecognitionInterface>("/MarkerlessRecognition");

	std::string prefix = CFG_PREFIX;
	vpath = this->config->get_string((prefix + "vpath").c_str()); 
	dpath = this->config->get_string((prefix + "dpath").c_str()); 	

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
	std::string lib = home + "/tensorflow/bazel-bin/tensorflow/tf_wrapper/tensorflowWrapper.so";
    	handle = dlopen(lib.c_str(),RTLD_NOW);
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
//	init_function init_twoGraph;

	*(void**)(&initGraph) = dlsym(handle,"init_fiveGraph");
	if((error=dlerror())!=NULL) {
		fprintf(stderr, "%s\n", dlerror());
	}

	/**(void**)(&init_twoGraph) = dlsym(handle,"init_twoGraph");
	if((error=dlerror())!=NULL) {
		fprintf(stderr, "%s\n", dlerror());
	}

	*/
	//path to the trained graph
    	std::string grPath = (home) + "/tensorflow/TrainedData/output_graph_CS.pb";
	
	//path to the the trained labels
	std::string laPath = (home) + "/tensorflow/TrainedData/output_labels_CS.txt";

	initGraph(grPath.c_str(),laPath.c_str());
	
/*	//path to the trained graph
    	grPath = (home) + "/tensorflow/TrainedData/output_graph_RSCS.pb";
	
	//path to the the trained labels
	laPath = (home) + "/tensorflow/TrainedData/output_labels_RSCS.txt";


	init_twoGraph(grPath.c_str(),laPath.c_str());
	*/
	recognize_mps();
}



void MarkerlessRecognitionThread::takePictureFromFVcamera(){ 

        logger->log_info(name(),"Taking Picture");
	
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

void MarkerlessRecognitionThread::loop(){
   	
	if(fv_cam == NULL || !fv_cam->ready()){
        	logger->log_info(name(),"Camera not ready");
		init(); 
      	return;
   	}

   	while ( ! mps_rec_if_->msgq_empty() ) {
     			
		if ( mps_rec_if_->msgq_first_is<MPSRecognitionInterface::ComputeMessage>() ) {
      			

 			logger->log_info(name(), "Received Compute Message");
		
			//MPSRecognitionInterface::TakeDataMessage *m = mps_rec_if_->msgq_first<MPSRecognitionInterface::TakeDataMessage>();
			//mps_rec_if_->set_msgid(m->id());
			mps_rec_if_->set_final(false);
			mps_rec_if_->write();

 			logger->log_info(name(), "Start Recognition");
			recognize_mps();
    		
    		}	
   	 	else {
    			logger->log_warn(name(), "Unknown message received");

    		}
    			
		mps_rec_if_->msgq_pop();
	}
}

