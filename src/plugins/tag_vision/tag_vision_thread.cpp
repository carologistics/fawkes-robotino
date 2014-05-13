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

#include "tag_vision_thread.h"

#include <tf/types.h>
#include <interfaces/Position3DInterface.h>

#define CFG_PREFIX "/plugins/tag_vision/"

using namespace fawkes;
using namespace alvar;

/** @class TagVisionThread "tag_vision_thread.h"
 * Thread to identify AR Tags and provid e their position
 * @author Nicolas Limpert & Randolph Maaßen
 */

/** Constructor. */
TagVisionThread::TagVisionThread()
  : Thread("TagVisionThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC),
    ConfigurationChangeHandler(CFG_PREFIX)
{
}

void
TagVisionThread::init()
{
    config->add_change_handler(this);
    // load config
    loadConfig();
    // init firevision camera
    // CAM swapping not working (??)
    if(fvCam != NULL){
        // free the camera
        fvCam->stop();
        fvCam->flush();
        fvCam->dispose_buffer();
        fvCam->close();
        delete fvCam;
        fvCam = NULL;
    }
    if(fvCam == NULL){
        fvCam = vision_master->register_for_camera(fvCamInfo.connection.c_str(), this);
        fvCam->start();
        fvCam->open();
        fvCamInfo.img_width = fvCam->pixel_width();
        fvCamInfo.img_height = fvCam->pixel_height();
    }

    // SHM image buffer
    if(shm_buffer != NULL) {
        delete shm_buffer;
        shm_buffer = NULL;
        imageBuffer = NULL;
    }

    shm_buffer = new firevision::SharedMemoryImageBuffer(
                shmID.c_str(),
                firevision::YUV422_PLANAR,
                fvCamInfo.img_width,
                fvCamInfo.img_height
                );
    if(!shm_buffer->is_valid()){
        delete shm_buffer;
        delete fvCam;
        shm_buffer = NULL;
        fvCam = NULL;
        throw fawkes::Exception("Shared memory segment not valid");
    }
    shm_buffer->set_frame_id(fvCamInfo.frame.c_str());

    imageBuffer = shm_buffer->buffer();
    ipl =  cvCreateImage(
                cvSize(fvCamInfo.img_width,fvCamInfo.img_height),
                IPL_DEPTH_8U,1);

    // set up poses
    maxPoses = 16;
    poses = new Pose[maxPoses];
}

void
TagVisionThread::finalize()
{
  blackboard->close(pose_if_);
  delete ipl;
  delete poses;
}

void
TagVisionThread::loop()
{
    if(cfg_mutex.try_lock()){
        //logger->log_info(name(),"Skipping loop");
        return;
    }
    if(fvCam == NULL || !fvCam->ready()){
        logger->log_info(name(),"Camera not ready");
        init();
        return;
    }
    //logger->log_info(name(),"entering loop");
    //get img form fv
    fvCam->capture();
    firevision::convert(fvCam->colorspace(),
                        firevision::YUV422_PLANAR,
                        fvCam->buffer(),
                        imageBuffer,
                        fvCamInfo.img_width,
                        fvCamInfo.img_height);
    fvCam->dispose_buffer();
    //convert img
    firevision::IplImageAdapter::convert_image_bgr(imageBuffer, ipl);
    //get poses from img
    size_t got = getMarkerPoses(ipl);
    for(size_t i = 0;i < got; i++){
        //logger->log_info(name(),"Tag %i: %f,%f,%f",poses[0].translation[0],poses[0].translation[1],poses[0].translation[2]);
    }

/*
    if (pose_if_->has_writer()) {
        pose_if_->read();
        double *r = pose_if_->rotation();
        tf::Quaternion pose_q(r[0], r[1], r[2], r[3]);
        logger->log_info(name(), "Pose: (%f,%f,%f)", pose_if_->translation(0),
                     pose_if_->translation(1), tf::get_yaw(pose_q));
    } else {
        logger->log_warn(name(), "No writer for pose interface");
    }*/
    cfg_mutex.unlock();
}

size_t
TagVisionThread::getMarkerPoses(IplImage *img)
{
    // detect makres on image
    detector.Detect(img,&alvarCam);
    // marker count
    size_t filled = 0;
    // fill output array
    for(size_t i=0;(i < detector.markers->size() && i < maxPoses);i++){
        poses[i]=(*(detector.markers))[i].pose;
        MarkerData data = detector.markers->at(i);
        logger->log_info(name(),"Tag id: %lu, %f,%f,%f",data.GetId(),data.pose.translation[0],data.pose.translation[1],data.pose.translation[2]);
        // add up to markers
        filled++;
    }
    return filled;
}

void
TagVisionThread::loadConfig(){
    // save prefix in string for concatinating
    std::string prefix = CFG_PREFIX;
    std::string prefix_static_transforms_ = config->get_string((prefix + "transform").c_str());
    // log, that we open load the config
    logger->log_info(name(),"loading config");
    // load alvar camera calibration
    alvarCam.SetCalib(config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),0,0,FILE_FORMAT_DEFAULT);
    // load marker size and apply it
    markerSize = config->get_uint((prefix + "marker_size").c_str());
    detector.SetMarkerSize(markerSize);

    //load camera informations
    fvCamInfo.connection = config->get_string((prefix + "camera").c_str());
    fvCamInfo.position_x = config->get_float(( prefix_static_transforms_ + "trans_x").c_str());
    fvCamInfo.position_y = config->get_float(( prefix_static_transforms_ + "trans_y").c_str());
    fvCamInfo.position_z = config->get_float(( prefix_static_transforms_ + "trans_z").c_str());
    fvCamInfo.position_pitch = config->get_float((prefix_static_transforms_ + "rot_pitch").c_str());
    fvCamInfo.opening_angle_horizontal = config->get_float((prefix + "camera_opening_angle_horizontal").c_str());
    fvCamInfo.opening_angle_vertical   = config->get_float((prefix + "camera_opening_angle_vertical").c_str());
    // Calculate visible Area
    fvCamInfo.angle_horizontal_to_opening_ = (1.57 - fvCamInfo.position_pitch) - (fvCamInfo.opening_angle_vertical / 2) ;
    fvCamInfo.visible_lenght_x_in_m_       = fvCamInfo.position_z * (tan(fvCamInfo.opening_angle_vertical + fvCamInfo.angle_horizontal_to_opening_) - tan(fvCamInfo.angle_horizontal_to_opening_));
    fvCamInfo.offset_cam_x_to_groundplane_ = fvCamInfo.position_z * tan(fvCamInfo.angle_horizontal_to_opening_);
    fvCamInfo.visible_lenght_y_in_m_       = fvCamInfo.position_z * tan (fvCamInfo.opening_angle_horizontal /2 );
    fvCamInfo.frame = config->get_string((prefix_static_transforms_ + "child_frame").c_str());

    //Image Buffer ID
    shmID = config->get_string((prefix + "shm_image_id").c_str());
}

// config handling
void TagVisionThread::config_value_erased(const char *path) {};
void TagVisionThread::config_tag_changed(const char *new_tag) {};
void TagVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
void TagVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{

    if(cfg_mutex.try_lock()){
        try{
            loadConfig();
        }
        catch(fawkes::Exception &e){
            logger->log_error(name(), e);
        }
    }
     // gets called for every changed entry... so init is called once per change.
    cfg_mutex.unlock();
}
