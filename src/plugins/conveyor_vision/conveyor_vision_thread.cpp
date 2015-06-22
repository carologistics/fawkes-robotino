/***************************************************************************
 *  tag_vision_thread.cpp - Thread to print the robot's position to the log
 *
 *  Created: Thu Sep 27 14:31:11 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "conveyor_vision_thread.h"

#include <tf/types.h>
#include <interfaces/Position3DInterface.h>

//#include <opencv/cv.h>
#include <math.h>

#define CFG_PREFIX "/plugins/conveyor_vision/"
#define IMAGE_CHANNELS 3

using namespace fawkes;
using namespace std;
using namespace cv;

/** @class ConveyorVisionThread "tag_vision_thread.h"
 * Thread to identify AR Tags and provid e their position
 * @author Nicolas Limpert & Randolph Maaßen
 */

/** Constructor. */
ConveyorVisionThread::ConveyorVisionThread()
  : Thread("ConveyorVisionThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC),
    ConfigurationChangeHandler(CFG_PREFIX),
    fawkes::TransformAspect(fawkes::TransformAspect::ONLY_PUBLISHER,"tags")
{
    fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;
//    this->markers_ = NULL;
}

void
ConveyorVisionThread::init()
{
   std::string prefix = CFG_PREFIX;
   std::string mps_cascade_name = (string)config->get_string((prefix + "classifier_file"));
   if( !mps_cascade.load( std::string(CONFDIR) + "/" + mps_cascade_name ) ){ printf("--(!)Error loading\n"); return; };

//   fawkes::Position3DInterface* puck_if_ = NULL;
    try {
            mps_conveyor_if_ = blackboard->open_for_writing<fawkes::Position3DInterface>("mps_conveyor");
//            puck_if_->set_frame(camera_info_.frame.c_str());
            mps_conveyor_if_->write();
//            puck_interfaces_.push_back(puck_if_);
    }catch (std::exception &e) {
            finalize();
            throw;
    }

    config->add_change_handler(this);
    // load config
    // config prefix in string for concatinating
//    std::string prefix = CFG_PREFIX;
    // log, that we open load the config
    logger->log_info(name(),"loading config");
    // load alvar camera calibration
//    if(!alvar_cam.SetCalib(config->get_string((prefix + "classifier_file").c_str()).c_str(),0,0,FILE_FORMAT_DEFAULT))
//    {
//      this->logger->log_warn(this->name(),"Faild to load calibration file");
//    }
//    // load marker size and apply it
//    marker_size = config->get_uint((prefix + "marker_size").c_str());
//    detector.SetMarkerSize(marker_size);

    //Image Buffer ID
    shm_id = config->get_string((prefix + "shm_image_id").c_str());

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

    //set camera resolution
//    alvar_cam.SetRes(this->img_width, this->img_height);

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
    max_marker = 16;
//    this->markers_ = new std::vector<alvar::MarkerData>();
//    this->tag_interfaces = new TagPositionList(this->blackboard,this->max_marker,frame,this->name(),this->logger, this->clock, this->tf_publisher);

}

void
ConveyorVisionThread::finalize()
{
  vision_master->unregister_thread(this);
  config->rem_change_handler(this);
  // free the markers
//  this->markers_->clear();
//  delete this->markers_;
  delete fv_cam;
  fv_cam = NULL;
  delete shm_buffer;
  shm_buffer= NULL;
  image_buffer = NULL;
//  cvReleaseImage(&ipl);
  ipl = NULL;
//  delete this->tag_interfaces;
}

void
ConveyorVisionThread::loop()
{
    if(!cfg_mutex.try_lock()){
        //logger->log_info(name(),"Skipping loop");
        return;
    }
    if(fv_cam == NULL || !fv_cam->ready()){
        logger->log_info(name(),"Camera not ready");
        init();
        return;
    }
    //logger->log_info(name(),"entering loop");
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
    frame = Mat(ipl);
    detect();
    mps_conveyor_if_->write();
    //get marker from img
//    get_marker();

//    this->tag_interfaces->update_blackboard(this->markers_);

    cfg_mutex.unlock();
}

//void
//ConveyorVisionThread::get_marker()
//{
//    // detect makres on image
////    detector.Detect(ipl,&alvar_cam);
//    // reset currently saved markers
//    this->markers_->clear();
//    // fill output array
//    for(alvar::MarkerData &tmp_marker: *(this->detector.markers))
//    {
//        Pose tmp_pose = tmp_marker.pose;
//        //skip the marker, if the pose is directly on the camera (error)
//        if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 && tmp_pose.translation[2]<1){
//            continue;
//        }
//        this->markers_->push_back(tmp_marker);
//        // add up to markers
//        tmp_marker.Visualize(ipl,&alvar_cam);
//    }
//    firevision::IplImageAdapter::convert_image_yuv422_planar(ipl,image_buffer);
//}

// config handling
void ConveyorVisionThread::config_value_erased(const char *path) {};
void ConveyorVisionThread::config_tag_changed(const char *new_tag) {};
void ConveyorVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
void ConveyorVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
  if(cfg_mutex.try_lock()){
    try{
      std::string prefix = CFG_PREFIX;
      // log, that we open load the config
      logger->log_info(name(),"loading config");
      // load alvar camera calibration
//      alvar_cam.SetCalib(config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),0,0,FILE_FORMAT_DEFAULT);
      // load marker size and apply it
//      marker_size = config->get_uint((prefix + "marker_size").c_str());
//      detector.SetMarkerSize(marker_size);
    }
    catch(fawkes::Exception &e){
      logger->log_error(name(), e);
    }
  }
  // gets called for every changed entry... so init is called once per change.
  cfg_mutex.unlock();
}

/** @function detectAndDisplay */
void ConveyorVisionThread::detect()
{
  std::vector<Rect> faces;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  mps_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
  int visibility_history = mps_conveyor_if_->visibility_history();
  
  // ignore images that probably contain more than one conveyor
  if (faces.size() == 1) {
    // width of object at one meters distance
    float focal_length_mm = 50;
    float obj_height_mm = 50;
    float im_height_px = frame.rows;
    float obj_height_px = faces[0].height;
    float sensor_height_mm = 2; // 1/4" in mm
    float height_width_sqrt = sqrt(frame.rows * frame.rows + frame.cols * frame.cols);
    float opening_angle = 0.86325;
    float pixels_per_rad = opening_angle / height_width_sqrt;
//    float norm_width = 0.47 / 78;
//    float norm_height = 0.4 / 85;
    Point center( faces[0].x + faces[0].width*0.5, faces[0].y + faces[0].height*0.5 );
    float dist_from_center_px = frame.cols / 2 - center.x;
    float obj_deg = -(dist_from_center_px * pixels_per_rad);
    float distance = (focal_length_mm * obj_height_mm * im_height_px) / (obj_height_px * sensor_height_mm);
    printf("found face: x: %d, y: %d, width: %d, height: %d distance:%f, obj_deg: %f\n", faces[0].x, faces[0].y, faces[0].width, faces[0].height, distance, obj_deg);
    mps_conveyor_if_->set_translation(0, 0);
    mps_conveyor_if_->set_translation(1, 0);
    mps_conveyor_if_->set_translation(2, 0);
    mps_conveyor_if_->set_rotation(0, obj_deg);
    mps_conveyor_if_->set_rotation(1, 0);
    //interface->set_timestamp(&p->cart.stamp);
    mps_conveyor_if_->set_frame("base_conveyor");
    mps_conveyor_if_->set_visibility_history(visibility_history);
    if (visibility_history >= 0) {
      mps_conveyor_if_->set_visibility_history(visibility_history + 1);
    } else {
      mps_conveyor_if_->set_visibility_history(1);
    }
  } else {
    // puck not visible
    if (visibility_history <= 0) {
            mps_conveyor_if_->set_visibility_history(visibility_history - 1);
    } else {
            mps_conveyor_if_->set_visibility_history(-1);
    }

  }
  
//  for( size_t i = 0; i < faces.size(); i++ )
//  {
////    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
////    cout << "num faces: " << faces.size() << endl;
////    rectangle(frame, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0,0,255));
////    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
//
////    Mat faceROI = frame_gray( faces[i] );
////    std::vector<Rect> eyes;
//
//    //-- In each face, detect eyes
////    eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );
//
////    for( size_t j = 0; j < eyes.size(); j++ )
////     {
//////       Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
//////       int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
////       rectangle(frame, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0,0,255));
//////       circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
////     }
//  }
//  //-- Show what you got
//  imshow( window_name, frame );
 }