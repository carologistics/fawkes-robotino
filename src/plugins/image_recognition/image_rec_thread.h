/***************************************************************************
 *  image_rec_thread.h - Thread to print recognized Image
 *
 *  Created: Thu May 7 10:10:00 2017
 *  Copyright  2017  Sebastian Schönitz, Daniel Habering, Carsten Stoffels
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


#ifndef __PLUGINS_MARKERLESS_MARKERLESS_RECOGNITION_THREAD_H_
#define __PLUGINS_MARKERLESS_MARKERLESS_RECOGNITION_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/vision.h>
#include <aspect/tf.h>
#include <string>

// config handling
#include <config/change_handler.h>


// firevision camera
#include <fvcams/camera.h>
#include <fvutils/base/roi.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvclassifiers/simple.h>

#include <fvutils/adapters/iplimage.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//interface
#include <interfaces/ImageRecognitionInterface.h>
#include <iostream>
#include <stdio.h>


#define THRESHOLD_UPPER 0.8
#define THRESHOLD_LOWER 0.1

namespace fawkes {
  class ImageRecognitionInterface;
  class Position3DInterface;
}

namespace firevision {
    class Camera;
    class SharedMemoryImageBuffer;
}

struct Probability
{
	std::vector<float>p;
	Probability(int count){
		p.resize(count);
	}
};

typedef std::vector<float> (*my_function)(const char*, const char*, const char*);
typedef void (*init_function)(const char*, const char*);

enum ImageType {
	BS,
	CS,
	DS,
	RS,
	SS,
	NoStationDetected,
	CorR
};

class ImageRecognitionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurationChangeHandler,
  public fawkes::ClockAspect,
  public fawkes::TransformAspect
{
public:
  ImageRecognitionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
  
 private:

  int checkResult(Probability);
  void clear_data();
  Probability recognize_current_pic(const std::string image);
  int recognize();
  void readImage();	
  void setupCamera();
  void takePictureFromFVcamera();

  fawkes::ImageRecognitionInterface *mps_rec_if_;


  // firevision camera
  firevision::Camera *fv_cam;

  // firevision image buffer
  firevision::SharedMemoryImageBuffer *shm_buffer;
  unsigned char *image_buffer;

  /// Image Buffer Id
  std::string shm_id;

  // Shared library tensoflow 
  my_function evaluate;
  void *handle;
  

// config handling
  void config_value_erased(const char *path) {};
  void config_tag_changed(const char *new_tag) {};
  void config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
  void config_value_changed(const fawkes::Configuration::ValueIterator *v){}; 


  // cv image
   IplImage *ipl;

  // Width of the image
  unsigned int img_width;
  // Height of the image
  unsigned int img_height;
  int CLASS_COUNT=0;
  std::string path_prefix_;
  std::string home;
  
  //cv::Mat frame;
  cv::Mat visionMat;
  cv::Mat depthMat; 

  std::string vframe; 

  std::string vpath; 

  std::string graph;
  std::string labels;

  std::vector<std::string>classes;
};

#endif
