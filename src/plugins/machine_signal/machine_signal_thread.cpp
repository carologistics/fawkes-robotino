/***************************************************************************
 *  machine_signal_thread.cpp - Detect signals using color thresholds
 *
 *  Copyright  2014 Victor Mataré
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

#include "machine_signal_thread.h"
#include <fvfilters/colorthreshold.h>
#include <fvfilters/roidraw.h>
#include <aspect/logging.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/scalers/lossy.h>
#include <fvmodels/scanlines/grid.h>
#include <cstring>

using namespace fawkes;
using namespace firevision;

#define __CFG_PREFIX "/plugins/machine_signal"

MachineSignalThread::MachineSignalThread()
    : Thread("MachineSignal", Thread::OPMODE_WAITFORWAKEUP),
      VisionAspect(VisionAspect::CYCLIC),
      ConfigurationChangeHandler(__CFG_PREFIX)
{
  width_ = 0;
  height_ = 0;
  camera_ = NULL;
  shmbuf_cam_ = NULL;
  cls_red_.colormodel = NULL;
  cls_red_.classifier = NULL;
  cls_red_.filter = NULL;
  cls_red_.color_expect = C_RED;
  cls_green_.colormodel = NULL;
  cls_green_.classifier = NULL;
  cls_green_.filter = NULL;
  cls_green_.color_expect = C_GREEN;
  cfg_changed_ = false;
}


void MachineSignalThread::config_tag_changed(const char *new_tag) {}
void MachineSignalThread::config_comment_changed(const Configuration::ValueIterator *v) {}

void MachineSignalThread::config_value_changed(const Configuration::ValueIterator *v)
{
  if (v->valid()) {
    std::string path = v->path();
    std::string sufx = path.substr(strlen(__CFG_PREFIX));
    std::string color_pfx = sufx.substr(0, sufx.substr(1).find("/")+1);
    std::string full_pfx = __CFG_PREFIX + color_pfx;
    color_classifier_t_ *classifier;

    if (color_pfx == "/red" || color_pfx == "/green") {
      if (color_pfx == "/red")
        classifier = &cls_red_;
      else if (color_pfx == "/green")
        classifier = &cls_green_;

      std::string opt = path.substr(full_pfx.length());

      if (opt == "/reference_color")
        test_set_cfg_value(&(classifier->cfg_ref_col), v->get_uints());
      else if (opt == "/saturation_thresh")
        test_set_cfg_value(&(classifier->cfg_sat_thresh), v->get_int());
      else if (opt == "/chroma_thresh")
        test_set_cfg_value(&(classifier->cfg_chroma_thresh), v->get_int());
      else if (opt == "/min_points")
        test_set_cfg_value(&(classifier->cfg_roi_min_points), v->get_uint());
      else if (opt == "/basic_roi_size")
        test_set_cfg_value(&(classifier->cfg_roi_basic_size), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        test_set_cfg_value(&(classifier->cfg_roi_neighborhood_min_match), v->get_uint());
      else if (opt == "/grow_by")
        test_set_cfg_value(&(classifier->cfg_roi_grow_by), v->get_uint());
    }
  }
}

void MachineSignalThread::config_value_erased(const char *path)
{}


static inline bool check_color_vector(std::vector<uint> &v)
{
  if (v.size() != 3) return false;
  for (std::vector<uint>::iterator it = v.begin(); it != v.end(); it++) {
    if (*it > 0xff) return false;
  }
  return true;
}


void MachineSignalThread::setup_classifier(color_classifier_t_ *color_data)
{
  delete color_data->colormodel;
  color_data->colormodel = new ColorModelSimilarity();
  if(!check_color_vector(color_data->cfg_ref_col)) {
    throw Exception("reference_color_ setting must be a list with 3 values from 0 to 255!");
  }
  RGB_t color = {
      (unsigned char)color_data->cfg_ref_col.at(0),
      (unsigned char)color_data->cfg_ref_col.at(1),
      (unsigned char)color_data->cfg_ref_col.at(2)
  };
  color_data->colormodel->add_color(color_data->color_expect, color, color_data->cfg_chroma_thresh, color_data->cfg_sat_thresh);

  delete color_data->classifier;
  color_data->classifier = new SimpleColorClassifier(
      new ScanlineGrid(width_, height_, 1, 1),
      color_data->colormodel,
      color_data->cfg_roi_min_points,
      color_data->cfg_roi_basic_size,
      false,
      color_data->cfg_roi_neighborhood_min_match,
      color_data->cfg_roi_grow_by,
      color_data->color_expect);

  delete color_data->filter;
  color_data->filter = new FilterColorThreshold(color, color_data->cfg_chroma_thresh, color_data->cfg_sat_thresh);
}



void MachineSignalThread::init()
{
  // Setup camera
  camera_ = vision_master->register_for_camera(
      config->get_string((__CFG_PREFIX "/camera")).c_str(), this);
  width_ = camera_->pixel_width();
  height_ = camera_->pixel_height();
  shmbuf_cam_= new SharedMemoryImageBuffer("signal", YUV422_PLANAR, width_, height_);

  // Setup RED classifier
  cls_red_.cfg_ref_col = config->get_uints(__CFG_PREFIX "/red/reference_color");
  cls_red_.cfg_chroma_thresh = config->get_int(__CFG_PREFIX "/red/chroma_thresh");
  cls_red_.cfg_sat_thresh = config->get_int(__CFG_PREFIX "/red/saturation_thresh");
  cls_red_.cfg_roi_min_points = config->get_int(__CFG_PREFIX "/red/min_points");
  cls_red_.cfg_roi_basic_size = config->get_int(__CFG_PREFIX "/red/basic_roi_size");
  cls_red_.cfg_roi_neighborhood_min_match = config->get_int(__CFG_PREFIX "/red/neighborhood_min_match");
  cls_red_.cfg_roi_grow_by = config->get_int(__CFG_PREFIX "/red/grow_by");
  setup_classifier(&cls_red_);
  cls_red_.shmbuf = new SharedMemoryImageBuffer("signal_red", YUV422_PLANAR, width_, height_);

  // Setup GREEN classifier
  cls_green_.cfg_ref_col = config->get_uints(__CFG_PREFIX "/green/reference_color");
  cls_green_.cfg_chroma_thresh = config->get_int(__CFG_PREFIX "/green/chroma_thresh");
  cls_green_.cfg_sat_thresh = config->get_int(__CFG_PREFIX "/green/saturation_thresh");
  cls_green_.cfg_roi_min_points = config->get_int(__CFG_PREFIX "/green/min_points");
  cls_green_.cfg_roi_basic_size = config->get_int(__CFG_PREFIX "/green/basic_roi_size");
  cls_green_.cfg_roi_neighborhood_min_match = config->get_int(__CFG_PREFIX "/green/neighborhood_min_match");
  cls_green_.cfg_roi_grow_by = config->get_int(__CFG_PREFIX "/green/grow_by");
  setup_classifier(&cls_green_);
  cls_green_.shmbuf = new SharedMemoryImageBuffer("signal_green", YUV422_PLANAR, width_, height_);

  cfg_changed_ = true;

  config->add_change_handler(this);
}


void MachineSignalThread::finalize()
{
  vision_master->unregister_thread(this);
  delete camera_;
  delete shmbuf_cam_;
  delete cls_red_.shmbuf;
  delete cls_green_.shmbuf;
  cleanup_classifiers();
}


void MachineSignalThread::cleanup_classifiers()
{
  delete cls_red_.colormodel;
  delete cls_red_.filter;
  delete cls_red_.classifier;
  delete cls_green_.colormodel;
  delete cls_green_.filter;
  delete cls_green_.classifier;
}


void MachineSignalThread::loop()
{
  FilterROIDraw *draw;
  std::list<ROI> *rois_R, *rois_G;

  if (unlikely(cfg_changed_)) {
    setup_classifier(&cls_red_);
    setup_classifier(&cls_green_);
    cfg_changed_ = false;
  }

  camera_->capture();

  cls_red_.classifier->set_src_buffer(camera_->buffer(), width_, height_);
  rois_R = cls_red_.classifier->classify();
  cls_red_.filter->set_src_buffer(camera_->buffer(),
      ROI::full_image( width_, height_));
  cls_red_.filter->set_dst_buffer(cls_red_.shmbuf->buffer(),
      ROI::full_image( width_, height_));
  cls_red_.filter->apply();
  draw = new FilterROIDraw(rois_R, FilterROIDraw::DASHED_HINT);
  draw->set_src_buffer(cls_red_.shmbuf->buffer(), ROI::full_image(width_, height_), 0);
  draw->set_dst_buffer(cls_red_.shmbuf->buffer(), NULL);
  draw->apply();

  cls_green_.classifier->set_src_buffer(camera_->buffer(), width_, height_);
  rois_G = cls_green_.classifier->classify();
  cls_green_.filter->set_src_buffer(camera_->buffer(),
      ROI::full_image( width_, height_));
  cls_green_.filter->set_dst_buffer(cls_green_.shmbuf->buffer(),
      ROI::full_image( width_, height_));
  cls_green_.filter->apply();
  draw = new FilterROIDraw(rois_G, FilterROIDraw::DASHED_HINT);
  draw->set_src_buffer(cls_green_.shmbuf->buffer(), ROI::full_image(width_, height_), 0);
  draw->set_dst_buffer(cls_green_.shmbuf->buffer(), NULL);
  draw->apply();


  camera_->dispose_buffer();
}

