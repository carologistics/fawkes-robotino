
/***************************************************************************
 *  pipeline_thread.h - Robotino AmpelVision Pipeline Thread
 *
 *  Created: Thu May 24 17:15:10 2012
 *  Copyright  2005-2012  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
 *             2005       Martin Heracles
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

#ifndef __PLUGINS_ROBOTINO_AMPELVAR_PIPELINE_THREAD_H_
#define __PLUGINS_ROBOTINO_AMPELVAR_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>
#include <utils/math/types.h>

#include <aspect/clock.h>

#include <string>

namespace firevision {
  class Camera;
  class ScanlineModel;
  class Bulb;
  class SharedMemoryImageBuffer;
  class Drawer;
}
namespace fawkes {
  class SwitchInterface;
  class Laser360Interface;
  class Position3DInterface;
}

class RobotinoAmpelVarPipelineThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ClockAspect
{
 public:
  RobotinoAmpelVarPipelineThread();
  virtual ~RobotinoAmpelVarPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  firevision::Camera                  *cam_;
  firevision::ScanlineModel           *scanline_;
  firevision::Bulb		              *mirror_;
  firevision::SharedMemoryImageBuffer *shm_buffer_;

  unsigned char *buffer_;

  unsigned int img_width_;
  unsigned int img_height_;

  firevision::colorspace_t cspace_from_;
  firevision::colorspace_t cspace_to_;

  fawkes::SwitchInterface *ampel_red_if_;
  fawkes::SwitchInterface *ampel_orange_if_;
  fawkes::SwitchInterface *ampel_green_if_;
  
  fawkes::Laser360Interface *laser_if_;
  fawkes::Position3DInterface *laser_pos_if_;

  fawkes::SwitchInterface *check_ampel_if_;

  fawkes::polar_coord_2d_t pol;
  const fawkes::polar_coord_2d_t * map;

  std::list< firevision::ROI > *rois_;

  std::string cfg_prefix_;
  std::string cfg_camera_;
  std::string cfg_mirror_file_;
  std::string cfg_frame_;
  std::string cfg_laser_;
  float cfg_camera_height_;
  bool cfg_debug_buffer_;

  float laserdistances[90];
  float *distances;
  float d;
  float theta;

  float height_red;
  float height_orange;
  float height_green;
  float distance_ampel;
  float distance_red;
  float distance_orange;
  float distance_green;
  float offset_laser;

  unsigned int px_position_red_x;
  unsigned int px_position_orange_x;
  unsigned int px_position_green_x;

  unsigned int px_position_red_y;
  unsigned int px_position_orange_y;
  unsigned int px_position_green_y;

  unsigned int current_x;
  unsigned int current_y;

  double ampel_x;
  double ampel_y;

  unsigned int w;
  unsigned int h;
 
  bool is_red;
  bool is_orange;
  bool is_green;
  bool gefunden;
  bool check_ampel;

  float delta_r;
  float delta_r_next;

  unsigned int bucket[64];
  unsigned int luminance; //Y in YUV
  unsigned int luminance_threshold;

  /*
   *  ROI_colors enthaelt die Regionen der Ampelfarben
   */
  firevision::ROI * ROI_colors[3]; //0 == red, 1 == orange, 3 == green

  fawkes::point_t center;
  fawkes::Time starttime;


  firevision::Drawer *drawer_;
};

#endif
