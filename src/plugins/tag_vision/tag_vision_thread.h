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
#include <aspect/tf.h>

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

//interface
#include <interfaces/TagVisionInterface.h>

#include "tag_position_list.h"

#define MAX_MARKERS 16

namespace fawkes {
  class Position3DInterface;
}

namespace firevision {
    class Camera;
    class SharedMemoryImageBuffer;
}

class TagVisionThread
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
  TagVisionThread();
  // marker size accasors
  // thread functions
  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  /// load config from file
  void loadConfig();
  /// the marker detector in alvar
  alvar::MarkerDetector<alvar::MarkerData> detector;
  /// the camera the detector uses
  alvar::Camera alvar_cam;
  /// the size of a marker in millimeter
  uint marker_size;
  /// function to get the markers from an image
  void get_marker();
  /// store the alvar markers, containing the poses
  std::vector<alvar::MarkerData> *markers_;
  /// maximum markers to detect, size for the markers array
  size_t max_marker;

  /// mutex for config access
  fawkes::Mutex cfg_mutex;

  /// firevision camera
  firevision::Camera *fv_cam;
  /// firevision image buffer
  firevision::SharedMemoryImageBuffer *shm_buffer;
  unsigned char *image_buffer;
  /// Image Buffer Id
  std::string shm_id;

  // config handling
  virtual void config_value_erased(const char *path);
  virtual void config_tag_changed(const char *new_tag);
  virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);

  /// cv image
  IplImage *ipl;

  /// blackboard communication
  TagPositionList *tag_interfaces;

  /// Width of the image
  unsigned int img_width;
  /// Height of the image
  unsigned int img_height;

};

#endif
