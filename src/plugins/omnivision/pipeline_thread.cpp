
/***************************************************************************
 *  pipeline_thread.h - Robotino OmniVision Pipeline Thread
 *
 *  Created: Thu May 24 17:17:46 2012
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

#include "pipeline_thread.h"

#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/draw/drawer.h>

#include <fvmodels/mirror/mirrormodel.h>
#include <fvmodels/relative_position/omni_relative.h>
#include <fvmodels/global_position/omni_global.h>
#include <fvmodels/scanlines/grid.h>
#include <fvmodels/color/lookuptable.h>
#include <fvmodels/mirror/bulb.h>

#include <fvclassifiers/simple.h>
#include <fvcams/camera.h>
#include <interfaces/Position3DInterface.h>
#include <utils/system/hostinfo.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>

#include <stdlib.h>
#include <cstdio>
#include <cmath>

#include <string>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** @class RobotinoOmniVisionPipelineThread "pipeline_thread.h"
 * Puck detector thread.
 *
 * @author Tim Niemueller (From ancient times and now again)
 * @author Daniel Beck (Mid-Size base)
 */


/** Constructor. */
RobotinoOmniVisionPipelineThread::RobotinoOmniVisionPipelineThread()
  : Thread("RobotinoOmniVisionThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
  scanline_ = NULL;
  cm_ = NULL;
  mirror_ = NULL;
  rel_pos_ = NULL;
  classifier_ = NULL;
  shm_buffer_ = NULL;
  puck_if_ = NULL;

  cspace_to_ = YUV422_PLANAR;

  drawer_ = 0;
}


/** Destructor. */
RobotinoOmniVisionPipelineThread::~RobotinoOmniVisionPipelineThread()
{
}


/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
RobotinoOmniVisionPipelineThread::init()
{
  cfg_prefix_        = "/hardware/robotino/omnivision/";
  cfg_camera_        = config->get_string((cfg_prefix_ + "camera").c_str());
  cfg_frame_         = config->get_string((cfg_prefix_ + "frame").c_str());
  cfg_cam_height_    = config->get_float((cfg_prefix_ + "camera_height").c_str());
  cfg_puck_radius_   = config->get_float((cfg_prefix_ + "puck_radius").c_str());
  cfg_mirror_file_   = std::string(CONFDIR) + "/" +
    config->get_string((cfg_prefix_ + "mirror_file").c_str());
  cfg_colormap_file_ = std::string(CONFDIR) + "/" +
    config->get_string((cfg_prefix_ + "colormap_file").c_str());

  // camera
  cam_ = vision_master->register_for_camera( cfg_camera_.c_str(), this );

  // interface
  try {
    puck_if_ = blackboard->open_for_writing<Position3DInterface>("OmniPuck");
    puck_if_->set_frame(cfg_frame_.c_str());
    puck_if_->write();
  } catch (Exception &e) {
    delete cam_;
    cam_ = NULL;
    e.append("Opening puck interface for writing failed");
    throw;
  }

  // image properties
  img_width_ = cam_->pixel_width();
  img_height_ = cam_->pixel_height();
  cspace_from_ = cam_->colorspace();

  // other stuff that might throw an exception
  try {
    // mirror calibration
    mirror_ = new Bulb(cfg_mirror_file_.c_str(), "omni-mirror",
                       true /* destroy on delete */ );

    // colormodel
    cm_ = new ColorModelLookupTable(cfg_colormap_file_.c_str(),
                                    "omni-colormap",
                                    true /* destroy on delete */ );

    // SHM image buffer
    shm_buffer_ = new SharedMemoryImageBuffer("omni-processed", cspace_to_,
                                              img_width_, img_height_);
    if (! shm_buffer_->is_valid() ) {
      throw Exception("Shared memory segment not valid");
    }
    shm_buffer_->set_frame_id(cfg_frame_.c_str());
    buffer_ = shm_buffer_->buffer();

  } catch (Exception& e) {
    delete mirror_;
    mirror_ = NULL;
    delete cm_;
    cm_ = NULL;
    delete shm_buffer_;
    shm_buffer_ = NULL;
    blackboard->close(puck_if_);
    puck_if_ = NULL;
    delete cam_;
    cam_ = NULL;
    throw;
  }

  // scanline_ model
  scanline_ = new ScanlineGrid( img_width_, img_height_, 5, 5 );

  // position model
  rel_pos_ = new OmniRelative(mirror_);

  // classifier
  classifier_ = new SimpleColorClassifier(scanline_, cm_, 0, 30);

  // drawer
  drawer_ = new Drawer();
  drawer_->set_buffer(buffer_, img_width_, img_height_);
  drawer_->set_color(0, 127, 127);
}


/** Thread finalization. */
void
RobotinoOmniVisionPipelineThread::finalize()
{
  delete drawer_;
  delete classifier_;
  delete rel_pos_;
  delete scanline_;
  delete shm_buffer_;
  delete cm_;
  delete mirror_;

  try {
    puck_if_->set_visibility_history(0);
    puck_if_->write();
    blackboard->close(puck_if_);
  } catch (Exception &e) {
    e.append("Closing puck interface failed");
    throw;
  }

  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  delete cam_;
}


/** Process image to detect objects.
 * Retrieves a new image from the camera and uses the classifier
 * determine objects according to the colormap. Publish the
 * position of the closest such object as puck position.
 */
void
RobotinoOmniVisionPipelineThread::loop()
{
  cam_->capture();
  convert(cspace_from_, cspace_to_, cam_->buffer(), buffer_, img_width_, img_height_);
  cam_->dispose_buffer();

  puck_visible_ = false;

  // run classifier
  classifier_->set_src_buffer( buffer_, img_width_, img_height_ );

  rois_ = classifier_->classify();

  // post-process ROIs
  if (rois_->empty()) {
    //       logger->log_warn(name(), "Could not find any ROIs in image");
    shm_buffer_->set_circle_found( false );
  } else {
    // if we have at least one ROI
    puck_visible_ = true;

    // find the puck candidate that is closest to the robot
    min_dist_ = 1000000.f;
    // for each ROI
    std::list< ROI >::iterator winner_roi = rois_->end();
    std::list< firevision::ROI >::iterator r;
    for (r = rois_->begin(); r != rois_->end(); r++) {
      // if ROI contains puck
      if (r->hint == H_BALL) {
        // calculate mass point of puck
        classifier_->get_mass_point_of_color( &(*r), &mass_point_ );
        // update puck position
        rel_pos_->set_center( mass_point_.x, mass_point_.y );
        rel_pos_->calc_unfiltered();

        if (rel_pos_->get_distance() < min_dist_) {
          min_dist_ = rel_pos_->get_distance();
          puck_image_x_ = mass_point_.x;
          puck_image_y_ = mass_point_.y;
          winner_roi = r;
        }
      }
    }

    if ( puck_visible_ ) {
      rel_pos_->set_center( puck_image_x_, puck_image_y_ );
      drawer_->draw_circle(puck_image_x_, puck_image_y_, 8);

      if ( rel_pos_->is_pos_valid() ) { rel_pos_->calc(); }
      else { puck_visible_ = false; }
    }

    if ( puck_visible_ && (winner_roi != rois_->end())) {
      shm_buffer_->set_circle_found( true );
      shm_buffer_->set_circle( puck_image_x_, puck_image_y_, 10 );
      shm_buffer_->set_roi( winner_roi->start.x,
                            winner_roi->start.y,
                            winner_roi->width,
                            winner_roi->height );
    } else {
      shm_buffer_->set_circle_found( false );
      shm_buffer_->set_roi( 0, 0, 0, 0 );
      shm_buffer_->set_circle( 0, 0, 0 );
    }

    // clean up
    rois_->clear();
    delete rois_;
  }

  // write data to interface
  int visibility_history = puck_if_->visibility_history();

  if (puck_visible_) {
    float distance  = rel_pos_->get_distance();
    float cam_angle = atan2f( cfg_cam_height_, distance );
    distance -= cfg_puck_radius_ / tan( cam_angle );
    
    float puck_x = cos( rel_pos_->get_bearing() ) * distance;
    float puck_y = sin( rel_pos_->get_bearing() ) * distance;
    
    if (visibility_history >= 0) {
      puck_if_->set_visibility_history(visibility_history + 1);
    } else {
      puck_if_->set_visibility_history(1);
    }

    double trans[3] = { puck_x, puck_y, 0 };
    double rot[4] = { 0, 0, 0, 1 };
    puck_if_->set_translation(trans);
    puck_if_->set_rotation(rot);

  } else {
    if (visibility_history <= 0) {
      puck_if_->set_visibility_history(visibility_history - 1);
    } else {
      puck_if_->set_visibility_history(-1);
    }
  }
  
  puck_if_->write();
}
