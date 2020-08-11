/***************************************************************************
 *  picture_taker_thread.h - Thread to take a picture
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

#ifndef __PLUGINS_PICTURE_TAKER_THREAD_H_
#define __PLUGINS_PICTURE_TAKER_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <string>

// firevision camera
#include <fvcams/camera.h>
#include <fvclassifiers/simple.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>

#include <fvutils/adapters/iplimage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <iostream>
#include <stdio.h>

namespace fawkes {
class PictureTakerInterface;
}

namespace firevision {
class Camera;
class SharedMemoryImageBuffer;
} // namespace firevision

/*! \class PictureTakerThread
 *
 *  Docs for MyClassName
 */

class PictureTakerThread : public fawkes::Thread,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::VisionAspect,
                           public fawkes::ClockAspect {
public:
  PictureTakerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

private:
  void readImage();
  void setupCamera();
  void takePictureFromFVcamera(std::string name);

  fawkes::PictureTakerInterface *p_t_if_;

  // firevision camera
  firevision::Camera *fv_cam;

  // firevision image buffer
  firevision::SharedMemoryImageBuffer *shm_buffer;
  unsigned char *image_buffer;

  /// Image Buffer Id
  std::string shm_id;

  // cv image
  IplImage *ipl;

  // Width of the image
  unsigned int img_width;
  // Height of the image
  unsigned int img_height;

  // cv::Mat frame;
  cv::Mat visionMat;

  std::string vframe;

  std::string vpath;
};

#endif
