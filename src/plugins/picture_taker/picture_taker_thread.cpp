/***************************************************************************
 *  picture_taker_thread.cpp - Thread to take a picture
 *  Created: Thu May 7 10:10:00 2017
 *  Copyright  2019  Daniel Habering
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

#include "picture_taker_thread.h"
#include "opencv2/opencv.hpp"
#include <dlfcn.h>
#include <interfaces/PictureTakerInterface.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>
#include <string>
#include <unistd.h>

#define CFG_PREFIX "/plugins/picture_taker/"
#define IMAGE_CHANNELS 3

using namespace fawkes;
using namespace std;
using namespace cv;
/** @class MarkerlessRecognitionThread 'picture_taker_thread.h'
 * Thread to print recognized MPS
 * @author Sebastian Schönitz, Daniel Habering, Carsten Stoffels
 */

/** Constructor. */
PictureTakerThread::PictureTakerThread()
    : Thread("PictureTakerThread", Thread::OPMODE_WAITFORWAKEUP),
      VisionAspect(VisionAspect::CYCLIC) {
  fv_cam = NULL;
  shm_buffer = NULL;
  image_buffer = NULL;
  ipl = NULL;
}

void PictureTakerThread::finalize() {
  blackboard->close(p_t_if_);
  delete fv_cam;
  fv_cam = NULL;
  delete shm_buffer;
  shm_buffer = NULL;
  image_buffer = NULL;
  ipl = NULL;
}

void PictureTakerThread::init() {
    p_t_if_ =
        blackboard->open_for_writing<PictureTakerInterface>("PictureTaker");

    std::string prefix = CFG_PREFIX;
    vpath = this->config->get_string((prefix + "vpath").c_str());

    // init firevision camera
    // CAM swapping not working (??)
    if (fv_cam != NULL) {
      // free the camera
      fv_cam->stop();
      fv_cam->flush();
      fv_cam->dispose_buffer();
      fv_cam->close();
      delete fv_cam;
      fv_cam = NULL;
    }
    if (fv_cam == NULL) {
      std::string connection =
          this->config->get_string((prefix + "camera").c_str());
      fv_cam = vision_master->register_for_camera(connection.c_str(), this);
      fv_cam->start();
      fv_cam->open();
      this->img_width = fv_cam->pixel_width();
      this->img_height = fv_cam->pixel_height();
    }

    // SHM image buffer
    if (shm_buffer != NULL) {
      delete shm_buffer;
      shm_buffer = NULL;
      image_buffer = NULL;
    }

    shm_buffer = new firevision::SharedMemoryImageBuffer(
        shm_id.c_str(), firevision::YUV422_PLANAR, this->img_width,
        this->img_height);
    if (!shm_buffer->is_valid()) {
      delete shm_buffer;
      delete fv_cam;
      shm_buffer = NULL;
      fv_cam = NULL;
      throw fawkes::Exception("Shared memory segment not valid");
    }

    std::string vframe = this->config->get_string((prefix + "vframe").c_str());
    shm_buffer->set_frame_id(vframe.c_str());

    image_buffer = shm_buffer->buffer();
    ipl = cvCreateImage(cvSize(this->img_width, this->img_height), IPL_DEPTH_8U,
                        IMAGE_CHANNELS);

}

void PictureTakerThread::takePictureFromFVcamera(std::string name,
                                                 std::string side) {
  fv_cam->capture();
  firevision::convert(fv_cam->colorspace(), firevision::YUV422_PLANAR,
                      fv_cam->buffer(), image_buffer, this->img_width,
                      this->img_height);
  fv_cam->dispose_buffer();
  // convert img
  firevision::IplImageAdapter::convert_image_bgr(image_buffer, ipl);
  visionMat = cvarrToMat(ipl);
  fawkes::Time now = fawkes::Time();
  std::string image_path = vpath + "_" + name + "_" + side + "_" +
                           std::to_string(now.in_sec()) + ".jpg";
  imwrite(image_path.c_str(), visionMat);
}

void PictureTakerThread::loop() {
  try {
    if (fv_cam == NULL || !fv_cam->ready()) {
      logger->log_info(name(), "Camera not ready");
      init();
      return;
    }
    p_t_if_->read();
    while (!p_t_if_->msgq_empty()) {
      if (p_t_if_->msgq_first_is<PictureTakerInterface::TakePictureMessage>()) {
        PictureTakerInterface::TakePictureMessage *msg =
            p_t_if_->msgq_first<PictureTakerInterface::TakePictureMessage>();
        takePictureFromFVcamera(std::string(msg->mps_name()),
                                std::string(msg->mps_side()));
      } else {
        logger->log_warn(name(), "Unknown message received");
      }
      p_t_if_->msgq_pop();
    }
  } catch (...) {
    // Catch everything in order to not crash anything important
  }
}
