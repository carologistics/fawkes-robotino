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
#include <core/threading/mutex_locker.h>
#include <cmath>

using namespace fawkes;
using namespace firevision;

#define CFG_PREFIX_ "/plugins/machine_signal"

MachineSignalThread::MachineSignalThread()
    : Thread("MachineSignal", Thread::OPMODE_WAITFORWAKEUP),
      VisionAspect(VisionAspect::CYCLIC),
      ConfigurationChangeHandler(CFG_PREFIX_)
{
  cam_width_ = 0;
  cam_height_ = 0;
  camera_ = NULL;
  cls_red_.colormodel = NULL;
  cls_red_.classifier = NULL;
  cls_red_.color_expect = C_RED;
  cls_green_.colormodel = NULL;
  cls_green_.classifier = NULL;
  cls_green_.color_expect = C_GREEN;
  cfg_changed_ = false;
  cam_changed_ = false;
  shmbuf_ = NULL;
}


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
  if(!check_color_vector(color_data->cfg_ref_col)) {
    throw Exception("reference_color_ setting must be a list with 3 values from 0 to 255!");
  }

  delete color_data->classifier;
  delete color_data->colormodel;

  color_data->colormodel = new ColorModelSimilarity();
  RGB_t color = {
      (unsigned char)color_data->cfg_ref_col.at(0),
      (unsigned char)color_data->cfg_ref_col.at(1),
      (unsigned char)color_data->cfg_ref_col.at(2)
  };
  color_data->colormodel->add_color(color_data->color_expect, color, color_data->cfg_chroma_thresh, color_data->cfg_sat_thresh);

  color_data->classifier = new SimpleColorClassifier(
      new ScanlineGrid(cam_width_, cam_height_, 2, 2),
      color_data->colormodel,
      color_data->cfg_roi_min_points,
      color_data->cfg_roi_basic_size,
      false,
      color_data->cfg_roi_neighborhood_min_match,
      color_data->cfg_roi_grow_by,
      color_data->color_expect);
}



void MachineSignalThread::init()
{
  // Configure camera
  cfg_camera_ = config->get_string(CFG_PREFIX_ "/camera");
  setup_camera();

  shmbuf_ = new SharedMemoryImageBuffer("machine_signal", YUV422_PLANAR, cam_width_, cam_height_);

  // Configure RED classifier
  cls_red_.cfg_ref_col = config->get_uints(CFG_PREFIX_ "/red/reference_color");
  cls_red_.cfg_chroma_thresh = config->get_int(CFG_PREFIX_ "/red/chroma_thresh");
  cls_red_.cfg_sat_thresh = config->get_int(CFG_PREFIX_ "/red/saturation_thresh");
  cls_red_.cfg_roi_min_points = config->get_int(CFG_PREFIX_ "/red/min_points");
  cls_red_.cfg_roi_basic_size = config->get_int(CFG_PREFIX_ "/red/basic_roi_size");
  cls_red_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX_ "/red/neighborhood_min_match");
  cls_red_.cfg_roi_grow_by = config->get_int(CFG_PREFIX_ "/red/grow_by");

  // Configure GREEN classifier
  cls_green_.cfg_ref_col = config->get_uints(CFG_PREFIX_ "/green/reference_color");
  cls_green_.cfg_chroma_thresh = config->get_int(CFG_PREFIX_ "/green/chroma_thresh");
  cls_green_.cfg_sat_thresh = config->get_int(CFG_PREFIX_ "/green/saturation_thresh");
  cls_green_.cfg_roi_min_points = config->get_int(CFG_PREFIX_ "/green/min_points");
  cls_green_.cfg_roi_basic_size = config->get_int(CFG_PREFIX_ "/green/basic_roi_size");
  cls_green_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX_ "/green/neighborhood_min_match");
  cls_green_.cfg_roi_grow_by = config->get_int(CFG_PREFIX_ "/green/grow_by");

  setup_classifier(&cls_red_);
  setup_classifier(&cls_green_);

  config->add_change_handler(this);
}

void MachineSignalThread::setup_camera()
{
  camera_ = vision_master->register_for_camera(cfg_camera_.c_str(), this);
  cam_width_ = camera_->pixel_width();
  cam_height_ = camera_->pixel_height();
#ifdef __FIREVISION_CAMS_FILELOADER_H_
  camera_->capture();
#endif
}

void MachineSignalThread::finalize()
{
  cleanup_camera();
  cleanup_classifiers();
  vision_master->unregister_thread(this);
  delete shmbuf_;
}

void MachineSignalThread::cleanup_camera()
{
#ifdef __FIREVISION_CAMS_FILELOADER_H_
  camera_->dispose_buffer();
#endif
  delete camera_;
}

void MachineSignalThread::cleanup_classifiers()
{
  delete cls_red_.colormodel;
  delete cls_red_.classifier;
  delete cls_green_.colormodel;
  delete cls_green_.classifier;
}


void MachineSignalThread::loop()
{
  FilterROIDraw *draw;
  std::list<ROI> *rois_R, *rois_G;

  if (unlikely(cfg_changed_)) {
    MutexLocker lock(&cfg_mutex_);
    setup_classifier(&cls_red_);
    setup_classifier(&cls_green_);
    cfg_changed_ = false;
  }

#ifndef __FIREVISION_CAMS_FILELOADER_H_
  camera_->capture();
#endif

  cls_red_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
  rois_R = cls_red_.classifier->classify();
  cls_green_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
  rois_G = cls_green_.classifier->classify();

  memcpy(shmbuf_->buffer(), camera_->buffer(), shmbuf_->data_size());

  // sort ROIs to optimize red-green matching
  rois_R->sort(sort_rois_by_x_);
  rois_G->sort(sort_rois_by_x_);

  if (rois_R->empty() || rois_G->empty()) return;

  std::list<signal_rois_t_> *signal_rois = create_signal_rois(rois_R, rois_G);
  signal_rois->clear();

  draw = new FilterROIDraw(&all_rois_, FilterROIDraw::DASHED_HINT);
  draw->set_src_buffer(shmbuf_->buffer(), ROI::full_image(cam_width_, cam_height_), 0);
  draw->set_dst_buffer(shmbuf_->buffer(), NULL);
  draw->apply();

  #ifndef __FIREVISION_CAMS_FILELOADER_H_
  camera_->dispose_buffer();
#endif
}

std::list<MachineSignalThread::signal_rois_t_> *MachineSignalThread::create_signal_rois(
    std::list<ROI> *rois_R,
    std::list<ROI> *rois_G)
{
  std::list<MachineSignalThread::signal_rois_t_> *rv = new std::list<MachineSignalThread::signal_rois_t_>();

  for (std::list<ROI>::iterator it_R = rois_R->begin(); it_R != rois_R->end(); it_R++) {

    if (!(roi_width_ok(it_R) && roi_aspect_ok(it_R)))
      continue;

    it_R->height = it_R->width;

    for (std::list<ROI>::iterator it_G = rois_G->begin(); it_G != rois_G->end(); it_G++) {

      if (rois_delivery_zone(it_R, it_G)) {
        // We're looking at the delivery zone. Don't expect to find any usable green ROIs
        // here, so we set the rest manually.
        uint start_y = it_R->start.y + it_R->width; // add width since it's more reliable
        ROI *roi_R = new ROI(*it_R);
        ROI *roi_Y = new ROI(it_R->start.x, start_y, it_R->width, it_R->height, it_R->image_width, it_R->image_height);
        roi_Y->color = C_YELLOW;
        ROI *roi_G = new ROI(it_R->start.x, start_y + it_R->height, it_R->width, it_R->height, it_R->image_width,
            it_R->image_height);
        roi_G->color = C_GREEN;

        rv->push_back({roi_R, roi_Y, roi_G});
        all_rois_.push_back(roi_R);
        all_rois_.push_back(roi_Y);
        all_rois_.push_back(roi_G);

        // Done with this signal, no point in looking for any further green ROIs.
        break;
      }

      if (roi_width_ok(it_G) && rois_similar_width(it_R, it_G) && rois_x_aligned(it_R, it_G)) {
        // Here it_G should have a pretty suitable green ROI.
        uint start_x = (it_R->start.x + it_G->start.x) / 2;
        ROI *roi_Y = new ROI(start_x, it_R->start.y + it_R->height, it_R->width, it_R->height, it_R->image_width,
            it_R->image_height);
        roi_Y->color = C_YELLOW;

        ROI *roi_R = new ROI(*it_R);
        ROI *roi_G = new ROI(*it_G);
        rv->push_back({roi_R, roi_Y, roi_G});
        all_rois_.push_back(*roi_R);
        all_rois_.push_back(*roi_Y);
        all_rois_.push_back(*roi_G);

      }
    }
  }
  return rv;
}

bool MachineSignalThread::rois_delivery_zone(std::list<ROI>::iterator red, std::list<ROI>::iterator green) {
  return (green->contains(red->start.x, red->start.y)
            && green->contains(red->start.x + red->get_width(),
                red->start.y + red->get_height()))
            || green->width > cam_width_/4;
}

bool MachineSignalThread::roi_width_ok(std::list<ROI>::iterator r)
{ return r->width < cam_width_/4; }

bool MachineSignalThread::rois_similar_width(std::list<ROI>::iterator r1, std::list<ROI>::iterator r2) {
  float width_ratio = (float)r1->width / (float)r2->width;
  return width_ratio >= 1/MAX_ROI_ASPECT_SKEW && width_ratio <= MAX_ROI_ASPECT_SKEW;
}

bool MachineSignalThread::rois_x_aligned(std::list<ROI>::iterator r1, std::list<ROI>::iterator r2) {
  float avg_width = (r1->width + r2->width) / 2.0f;
  return abs(r1->start.x - r2->start.x) / avg_width < 0.4;
}

bool MachineSignalThread::roi_aspect_ok(std::list<firevision::ROI>::iterator r) {
  float aspect_ratio = (float)r->width / (float)r->height;
  return aspect_ratio > 1/MAX_ROI_ASPECT_SKEW && aspect_ratio < MAX_ROI_ASPECT_SKEW;
}


void MachineSignalThread::config_value_erased(const char *path) {}
void MachineSignalThread::config_tag_changed(const char *new_tag) {}
void MachineSignalThread::config_comment_changed(const Configuration::ValueIterator *v) {}

void MachineSignalThread::config_value_changed(const Configuration::ValueIterator *v)
{
  if (v->valid()) {
    std::string path = v->path();
    std::string sufx = path.substr(strlen(CFG_PREFIX_));
    std::string color_pfx = sufx.substr(0, sufx.substr(1).find("/")+1);
    std::string full_pfx = CFG_PREFIX_ + color_pfx;
    std::string opt = path.substr(full_pfx.length());

    MutexLocker lock(&cfg_mutex_);

    if (color_pfx == "/red" || color_pfx == "/green") {
      color_classifier_t_ *classifier = NULL;
      if (color_pfx == "/red")
        classifier = &cls_red_;
      else if (color_pfx == "/green")
        classifier = &cls_green_;

      std::string opt = path.substr(full_pfx.length());

      if (opt == "/reference_color")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_ref_col), v->get_uints());
      else if (opt == "/saturation_thresh")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_sat_thresh), v->get_int());
      else if (opt == "/chroma_thresh")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_chroma_thresh), v->get_int());
      else if (opt == "/min_points")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_roi_min_points), v->get_uint());
      else if (opt == "/basic_roi_size")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_roi_basic_size), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_roi_neighborhood_min_match), v->get_uint());
      else if (opt == "/grow_by")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_roi_grow_by), v->get_uint());
    }
  }
}

