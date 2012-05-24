
/***************************************************************************
 *  pipeline_thread.h - Robotino OmniVision Pipeline Thread
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

#ifndef __PLUGINS_ROBOTINO_OMNIVISION_PIPELINE_THREAD_H_
#define __PLUGINS_ROBOTINO_OMNIVISION_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>

#include <string>

namespace firevision {
  class Camera;
  class ScanlineModel;
  class ColorModel;
  class MirrorModel;
  class SimpleColorClassifier;
  class RelativePositionModel;
  class SharedMemoryImageBuffer;
  class Drawer;
}
namespace fawkes {
  class Position3DInterface;
}

class RobotinoOmniVisionPipelineThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  RobotinoOmniVisionPipelineThread();
  virtual ~RobotinoOmniVisionPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  firevision::Camera                  *cam_;
  firevision::ScanlineModel           *scanline_;
  firevision::ColorModel              *cm_;
  firevision::MirrorModel             *mirror_;
  firevision::RelativePositionModel   *rel_pos_;
  firevision::SimpleColorClassifier   *classifier_;
  firevision::SharedMemoryImageBuffer *shm_buffer_;

  unsigned char *buffer_;

  unsigned int img_width_;
  unsigned int img_height_;

  firevision::colorspace_t cspace_from_;
  firevision::colorspace_t cspace_to_;

  bool             puck_visible_;
  unsigned int     puck_image_x_;
  unsigned int     puck_image_y_;
  fawkes::point_t  mass_point_;
  float            min_dist_;

  fawkes::Position3DInterface *puck_if_;

  std::list< firevision::ROI > *rois_;

  std::string cfg_prefix_;
  std::string cfg_camera_;
  std::string cfg_mirror_file_;
  std::string cfg_colormap_file_;
  std::string cfg_frame_;
  float       cfg_cam_height_;
  float       cfg_puck_radius_;

  firevision::Drawer *drawer_;
};

#endif
