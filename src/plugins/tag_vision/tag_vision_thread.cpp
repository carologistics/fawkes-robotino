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

#include <opencv/cv.h>

#define CFG_PREFIX "/plugins/tag_vision/"
#define IMAGE_CAHNNELS 3

using namespace fawkes;
using namespace alvar;
using namespace std;

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
    markers = NULL;
    fv_cam = NULL;
    shm_buffer = NULL;
    image_buffer = NULL;
    ipl = NULL;
}

void
TagVisionThread::init()
{
    config->add_change_handler(this);
    // load config
    loadConfig();
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
        fv_cam = vision_master->register_for_camera(fv_cam_info.connection.c_str(), this);
        fv_cam->start();
        fv_cam->open();
        fv_cam_info.img_width = fv_cam->pixel_width();
        fv_cam_info.img_height = fv_cam->pixel_height();
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
                fv_cam_info.img_width,
                fv_cam_info.img_height
                );
    if(!shm_buffer->is_valid()){
        delete shm_buffer;
        delete fv_cam;
        shm_buffer = NULL;
        fv_cam = NULL;
        throw fawkes::Exception("Shared memory segment not valid");
    }
    shm_buffer->set_frame_id(fv_cam_info.frame.c_str());

    image_buffer = shm_buffer->buffer();
    ipl =  cvCreateImage(
                cvSize(fv_cam_info.img_width,fv_cam_info.img_height),
                IPL_DEPTH_8U,IMAGE_CAHNNELS);


    // set up marker
    max_marker = 16;
    markers = new MarkerData[max_marker];
    tag_interfaces.resize(max_marker,NULL);
}

void
TagVisionThread::finalize()
{
  blackboard->close(pose_if_);
  delete [] markers;
  markers = NULL;
  delete fv_cam;
  fv_cam = NULL;
  delete shm_buffer;
  shm_buffer= NULL;
  delete image_buffer;
  image_buffer = NULL;
  free(ipl);
  ipl = NULL;
  for(size_t i =0; i < tag_interfaces.size(); i++){
      if(tag_interfaces[i] != NULL){
          blackboard->close(tag_interfaces[i]);
      }
  }
}

void
TagVisionThread::loop()
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
                        fv_cam_info.img_width,
                        fv_cam_info.img_height);
    fv_cam->dispose_buffer();
    //convert img
    firevision::IplImageAdapter::convert_image_bgr(image_buffer, ipl);
    //get marker from img
    size_t got = get_marker();
    //for(size_t i = 0;i < got; i++){
    //    logger->log_info(name(),"Tag %i: %f,%f,%f",markers[i].GetId(),markers[i].pose.translation[0],markers[i].pose.translation[1],markers[i].pose.translation[2]);
    //}

    update_blackboard(got);

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
TagVisionThread::get_marker()
{
    // detect makres on image
    detector.Detect(ipl,&alvar_cam);
    // marker count
    size_t filled = 0;
    // fill output array
    for(size_t i=0;(i < detector.markers->size() && i < max_marker);i++){
        Marker tmp_marker = detector.markers->at(i);
        Pose tmp_pose = tmp_marker.pose;
        //skip the marker, if the pose is directly on the camera (error)
        if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 && tmp_pose.translation[2]<1){
            continue;
        }
        markers[i] = detector.markers->at(i);
        // add up to markers
        filled++;
        markers[i].Visualize(ipl,&alvar_cam);
    }
    firevision::IplImageAdapter::convert_image_yuv422_planar(ipl,image_buffer);
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
    alvar_cam.SetCalib(config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),0,0,FILE_FORMAT_DEFAULT);
    // load marker size and apply it
    marker_size = config->get_uint((prefix + "marker_size").c_str());
    detector.SetMarkerSize(marker_size);

    //load camera informations
    fv_cam_info.connection = config->get_string((prefix + "camera").c_str());
    fv_cam_info.position_x = config->get_float(( prefix_static_transforms_ + "trans_x").c_str());
    fv_cam_info.position_y = config->get_float(( prefix_static_transforms_ + "trans_y").c_str());
    fv_cam_info.position_z = config->get_float(( prefix_static_transforms_ + "trans_z").c_str());
    fv_cam_info.position_pitch = config->get_float((prefix_static_transforms_ + "rot_pitch").c_str());
    fv_cam_info.opening_angle_horizontal = config->get_float((prefix + "camera_opening_angle_horizontal").c_str());
    fv_cam_info.opening_angle_vertical   = config->get_float((prefix + "camera_opening_angle_vertical").c_str());
    // Calculate visible Area
    fv_cam_info.angle_horizontal_to_opening_ = (1.57 - fv_cam_info.position_pitch) - (fv_cam_info.opening_angle_vertical / 2) ;
    fv_cam_info.visible_lenght_x_in_m_       = fv_cam_info.position_z * (tan(fv_cam_info.opening_angle_vertical + fv_cam_info.angle_horizontal_to_opening_) - tan(fv_cam_info.angle_horizontal_to_opening_));
    fv_cam_info.offset_cam_x_to_groundplane_ = fv_cam_info.position_z * tan(fv_cam_info.angle_horizontal_to_opening_);
    fv_cam_info.visible_lenght_y_in_m_       = fv_cam_info.position_z * tan (fv_cam_info.opening_angle_horizontal /2 );
    fv_cam_info.frame = config->get_string((prefix_static_transforms_ + "child_frame").c_str());

    //Image Buffer ID
    shm_id = config->get_string((prefix + "shm_image_id").c_str());
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

void TagVisionThread::create_tag_interface(size_t position){
    Position3DInterface *new_interface = NULL;
    string interface_name = "tag_" + to_string(position);
    try{
        new_interface = blackboard->open_for_writing<Position3DInterface>(interface_name.c_str());
        new_interface->set_visibility_history(-9999);
        new_interface->set_frame(fv_cam_info.frame.c_str());
        new_interface->write();
        tag_interfaces[position] = new_interface;
    }
    catch (std::exception &e){
        finalize();
        throw;
    }
}

void TagVisionThread::update_blackboard(size_t marker_count){
    for(size_t i = 0; i < max_marker; i++){
        if(i>marker_count){
            if(tag_interfaces[i]==NULL){
                continue;
            }
            else{
                tag_interfaces[i]->set_visibility_history(0);
            }
        }
        else{
            //unsigned long id = markers[i].GetId();

            if(tag_interfaces[i]==NULL){
                create_tag_interface(i);
            }
            //temporary the rotation quaternion in wxyz
            double quat[4];
            //temp mat to get cv data
            CvMat mat;
            //angles in heading attitude bank
            double rot[3];
            //create the mat
            cvInitMatHeader(&mat, 4, 1, CV_64F, quat);
            //get the quaternion of this pose in the mat
            markers[i].pose.GetQuaternion(&mat);
            //get the temporary quaternion in wxyz order to calculate angles
            quat[1] = CV_MAT_ELEM(mat, double, 1, 0);
            quat[2] = CV_MAT_ELEM(mat, double, 2, 0);
            quat[3] = CV_MAT_ELEM(mat, double, 3, 0);
            quat[0] = CV_MAT_ELEM(mat, double, 0, 0);
                //calculate the angles from the quaternion
            markers[i].pose.QuatToEul(quat,rot);
            //publish the angles
            tag_interfaces[i]->set_rotation(ROT::X,rot[ROT::X]);
            tag_interfaces[i]->set_rotation(ROT::Y,rot[ROT::Y]);
            tag_interfaces[i]->set_rotation(ROT::Z,rot[ROT::Z]);
            tag_interfaces[i]->set_rotation(ROT::W,0);
            //publish the translation
            tag_interfaces[i]->set_translation(0,markers[i].pose.translation[0]);
            tag_interfaces[i]->set_translation(1,markers[i].pose.translation[1]);
            tag_interfaces[i]->set_translation(2,markers[i].pose.translation[2]);

            tag_interfaces[i]->set_frame(fv_cam_info.frame.c_str());
            tag_interfaces[i]->set_visibility_history(1);

            tag_interfaces[i]->write();
        }
    }
}
