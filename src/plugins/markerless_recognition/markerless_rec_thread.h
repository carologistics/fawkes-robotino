/***************************************************************************
 *  markerless_rec_thread.h - Thread to print recognized MPS
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
#include <interfaces/TagVisionInterface.h>
#include <iostream>
#include <stdio.h>

namespace fawkes {
  class MPSRecognitionInterface;
  class Position3DInterface;
}

namespace firevision {
    class Camera;
    class SharedMemoryImageBuffer;
}

struct Probability
{
	float p[5];
};

typedef Probability (*my_function)(const char*, const char*, const char*);

enum MPSType {
	BS,
	CS,
	DS,
	RS,
	SS,
	NoStationDetected
};

class MarkerlessRecognitionThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::VisionAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurationChangeHandler,
  public fawkes::TransformAspect
{


public:
  MarkerlessRecognitionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
  
 private:

  // load config from file
  void load_config();
  // function to get the markers from an image
  void get_marker();
  //  store the alvar markers, containing the poses
  //  std::vector<alvar::MarkerData> *markers_;
  /// maximum markers to detect, size for the markers array
  size_t max_marker;

  // mutex for config access
  fawkes::Mutex cfg_mutex;

  void clear_data();
  Probability recognize_current_pic(const std::string image);
  float recognize_mps();
  void estimate_mps_type(const Probability &prob);
  void readImage();	
  int  checkProbability(Probability prob);
  void setupCamera();
  void takePictureFromFVcamera();

  fawkes::MPSRecognitionInterface *mps_rec_if_;

  cv::Mat frame;
  cv::CascadeClassifier mps_cascade;

  // Current Frame to Evaluate
  std::string frameToRecognize;
  // firevision camera
  firevision::Camera *fv_cam;

  // firevision image buffer
  firevision::SharedMemoryImageBuffer *shm_buffer;
  unsigned char *image_buffer;

  // Things copied from Convery_vision maybe not needed
  std::deque<float> world_pos_z_measurements;
  std::deque<float> world_pos_y_measurements;
  std::deque<float> world_pos_x_measurements;
  std::deque<int> conveyor_average_horizontal_diff;
  std::deque<int> conveyor_average_horizontal_start;
  std::deque<int> conveyor_average_vertical_start;
  float world_pos_z_average;
  float world_pos_y_average;
  float world_pos_x_average;
  /// Image Buffer Id
  std::string shm_id;
  float obj_realworld_distance;
  float obj_realworld_width;
  float obj_realworld_pixels;
  unsigned int num_frames_for_average;
  float conveyor_distance_threshold;
  bool visualization_enabled;
  bool use_hough_lines_;
  int hough_lines_averaging_count_;
  int binary_threshold_min_;
  int binary_threshold_max_;
  int binary_threshold_average_min_;
  int binary_threshold_average_max_;
  int max_binary_value_;
  int threshold_type_;
  int morph_element_;
  int morph_size_;
  int morph_operator_;
  int canny_threshold_;
  unsigned int line_mean_;
  float conveyor_realworld_distance;
  float conveyor_realworld_pixels;
  float conveyor_realworld_width;


    // config handling
  virtual void config_value_erased(const char *path);
  virtual void config_tag_changed(const char *new_tag);
  virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);

  // cv image
   IplImage *ipl;

  // Width of the image
  unsigned int img_width;
  // Height of the image
  unsigned int img_height;

  std::string path_prefix_;
  std::vector<std::string> imageSet_;

  std::string home;

  float th_first = 0.8;
  float th_sec = 0.5;
};

#endif
