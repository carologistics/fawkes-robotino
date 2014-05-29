/***************************************************************************
 *  tag_vision_thread.h - Thread to print the robot's position to the log
 *
 *  Created: Thu Sep 27 14:27:09 2012
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

#ifndef __PLUGINS_TAG_VISION_TAG_VISION_THREAD_H_
#define __PLUGINS_TAG_VISION_TAG_VISION_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/vision.h>

// config handling
#include <config/change_handler.h>

// cv is needed for image conversion to alvar
#include <cv.h>
// alvar marker detection to get poses
#include <alvar/MarkerDetector.h>

// firevision camera
#include <fvcams/camera.h>
#include <fvutils/base/roi.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvclassifiers/simple.h>

#include <fvutils/adapters/iplimage.h>

namespace fawkes {
  class Position3DInterface;
}

namespace firevision {
    class Camera;
    class SharedMemoryImageBuffer;
    struct camera_info{
        std::string connection;
        std::string frame;
        float opening_angle_horizontal;
        float opening_angle_vertical;
        unsigned int img_width;
        unsigned int img_height;
        float position_x;
        float position_y;
        float position_z;
        float position_pitch;
        float offset_cam_x_to_groundplane_;
        float angle_horizontal_to_opening_;
        float visible_lenght_x_in_m_;
        float visible_lenght_y_in_m_;
        firevision::ROI fullimage;
};

}

class TagVisionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurationChangeHandler
{
 public:
  TagVisionThread();
  // marker size accasors
  // thread functions
  virtual void init();
  virtual void loop();
  virtual void finalize();


 private:
  // load config from file
  void loadConfig();
  // fawkes 3d pose for publishing on blackboard
  fawkes::Position3DInterface *pose_if_;
  // the marker detector in alvar
  alvar::MarkerDetector<alvar::MarkerData> detector;
  // the camera the detector uses
  alvar::Camera alvar_cam;
  // the size of a marker in millimeter
  uint marker_size;
  // function to get the markers from an image
  size_t get_marker();
  // store the alvar markers, containing the poses
  alvar::MarkerData *markers;
  // maximum markers to detect, size for the markers array
  size_t max_marker;

  // mutex for config access
  fawkes::Mutex cfg_mutex;

  // firevision camera
  firevision::Camera *fv_cam;
  // info about the firevision camera, needed to connect
  firevision::camera_info fv_cam_info;
  // firevision image buffer
  firevision::SharedMemoryImageBuffer *shm_buffer;
  unsigned char *image_buffer;
  // Image Buffer Id
  std::string shm_id;

  // config handling
  virtual void config_value_erased(const char *path);
  virtual void config_tag_changed(const char *new_tag);
  virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);

  //cv image
  IplImage *ipl;

  //blackboard communication
  void create_tag_interface(size_t position);
  std::vector<fawkes::Position3DInterface *> tag_interfaces;
  void update_blackboard(size_t marker_count);

  enum ROT{
      X=0,
      Y=1,
      Z=2,
      W=3
  };
};

#endif
