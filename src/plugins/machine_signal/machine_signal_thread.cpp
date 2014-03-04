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
#include <aspect/logging.h>
#include <fvutils/color/colorspaces.h>
#include <cstring>
#include <core/threading/mutex_locker.h>
#include <cmath>
#include <cfloat>

using namespace fawkes;
using namespace firevision;

MachineSignalThread::MachineSignalThread()
    : Thread("MachineSignal", Thread::OPMODE_WAITFORWAKEUP),
      VisionAspect(VisionAspect::CYCLIC),
      ConfigurationChangeHandler(CFG_PREFIX)
{
  cam_width_ = 0;
  cam_height_ = 0;
  cfg_light_on_threshold_ = 0;
  camera_ = NULL;

  cls_red_.colormodel = NULL;
  cls_red_.classifier = NULL;
  cls_red_.scanline_grid = NULL;
  cls_red_.color_expect = C_RED;
  cls_green_.colormodel = NULL;
  cls_green_.classifier = NULL;
  cls_green_.scanline_grid = NULL;
  cls_green_.color_expect = C_GREEN;

  cfg_changed_ = false;
  cam_changed_ = false;

  shmbuf_ = NULL;
  shmbuf_cam_ = NULL;

  light_classifier_ = NULL;
  light_colormodel_ = NULL;
  light_scangrid_ = NULL;
  cfg_light_on_min_neighborhood_ = 0;
  cfg_light_on_min_points_ = 0;

  black_classifier_ = NULL;
  black_colormodel_ = NULL;
  black_scangrid_ = NULL;
  cfg_black_min_neighborhood_ = 0;
  cfg_black_min_points_ = 0;
  cfg_black_threshold_ = 0;

  roi_drawer_ = NULL;
  color_filter_ = NULL;
  cfg_roi_max_aspect_ratio_ = 1.7;
  combined_colormodel_ = NULL;
  last_second_ = NULL;
  buflen_ = 0;
  bb_signal_compat_ = NULL;
}



void MachineSignalThread::setup_classifier(color_classifier_context_t_ *color_data)
{
  delete color_data->classifier;
  delete color_data->colormodel;
  delete color_data->scanline_grid;

  // Update the color class used by the combined color model for the tuning filter
  color_data->color_class->chroma_threshold = color_data->cfg_chroma_thresh;
  color_data->color_class->saturation_threshold = color_data->cfg_sat_thresh;
  color_data->color_class->set_reference(color_data->cfg_ref_col);

  color_data->colormodel = new ColorModelSimilarity();
  color_data->colormodel->add_color(color_data->color_class);
  color_data->scanline_grid = new ScanlineGrid(
    cam_width_, cam_height_,
    color_data->cfg_scangrid_x_offset, color_data->cfg_scangrid_y_offset);

  color_data->classifier = new SimpleColorClassifier(
      color_data->scanline_grid,
      color_data->colormodel,
      color_data->cfg_roi_min_points,
      color_data->cfg_roi_basic_size,
      false,
      color_data->cfg_roi_neighborhood_min_match,
      0,
      color_data->color_expect);
}



void MachineSignalThread::init()
{
  // Configure camera
  cfg_camera_ = config->get_string(CFG_PREFIX "/camera");
  setup_camera();

  shmbuf_ = new SharedMemoryImageBuffer("machine_signal", YUV422_PLANAR, cam_width_, cam_height_);
  shmbuf_cam_ = new SharedMemoryImageBuffer("machine_cam", YUV422_PLANAR, cam_width_, cam_height_);
  roi_drawer_ = new FilterROIDraw(&drawn_rois_, FilterROIDraw::DASHED_HINT);

  // Configure RED classifier
  cls_red_.cfg_ref_col = config->get_uints(CFG_PREFIX "/red/reference_color");
  cls_red_.cfg_chroma_thresh = config->get_int(CFG_PREFIX "/red/chroma_thresh");
  cls_red_.cfg_sat_thresh = config->get_int(CFG_PREFIX "/red/saturation_thresh");
  cls_red_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/red/min_points");
  cls_red_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/red/basic_roi_size");
  cls_red_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX "/red/neighborhood_min_match");
  cls_red_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/red/scangrid_x_offset");
  cls_red_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/red/scangrid_y_offset");

  cls_red_.color_class = new ColorModelSimilarity::color_class_t(
    cls_red_.color_expect, cls_red_.cfg_ref_col, cls_red_.cfg_chroma_thresh, cls_red_.cfg_sat_thresh);
  setup_classifier(&cls_red_);

  // Configure GREEN classifier
  cls_green_.cfg_ref_col = config->get_uints(CFG_PREFIX "/green/reference_color");
  cls_green_.cfg_chroma_thresh = config->get_int(CFG_PREFIX "/green/chroma_thresh");
  cls_green_.cfg_sat_thresh = config->get_int(CFG_PREFIX "/green/saturation_thresh");
  cls_green_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/green/min_points");
  cls_green_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/green/basic_roi_size");
  cls_green_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX "/green/neighborhood_min_match");
  cls_green_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/green/scangrid_x_offset");
  cls_green_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/green/scangrid_y_offset");

  cls_green_.color_class = new ColorModelSimilarity::color_class_t(
    cls_green_.color_expect, cls_green_.cfg_ref_col, cls_green_.cfg_chroma_thresh, cls_green_.cfg_sat_thresh);
  setup_classifier(&cls_green_);

  // Configure brightness classifier
  cfg_light_on_threshold_ = config->get_uint(CFG_PREFIX "/bright_light/min_brightness");
  cfg_light_on_min_points_ = config->get_uint(CFG_PREFIX "/bright_light/min_points");
  cfg_light_on_min_neighborhood_ = config->get_uint(CFG_PREFIX "/bright_light/neighborhood_min_match");
  cfg_light_on_min_area_cover_ = config->get_float(CFG_PREFIX "/bright_light/min_area_cover");

  // Configure black classifier
  cfg_black_threshold_ = config->get_uint(CFG_PREFIX "/black/max_luminance");
  cfg_black_min_neighborhood_ = config->get_uint(CFG_PREFIX "/black/neighborhood_min_match");
  cfg_black_min_points_ = config->get_uint(CFG_PREFIX "/black/min_points");

  // Other config
  cfg_roi_max_aspect_ratio_ = config->get_float(CFG_PREFIX "/roi_max_aspect_ratio");
  cfg_roi_max_width_ratio_ = config->get_float(CFG_PREFIX "/roi_max_r2g_width_ratio");
  cfg_roi_xalign_ = config->get_float(CFG_PREFIX "/roi_xalign_by_width");
  cfg_tuning_mode_ = config->get_bool(CFG_PREFIX "/tuning_mode");
  cfg_max_jitter_ = config->get_float(CFG_PREFIX "/max_jitter");
  cfg_draw_processed_rois_ = config->get_bool(CFG_PREFIX "/draw_processed_rois");

  // Setup combined ColorModel for tuning filter
  combined_colormodel_ = new ColorModelSimilarity();
  combined_colormodel_->add_color(cls_red_.color_class);
  combined_colormodel_->add_color(cls_green_.color_class);
  color_filter_ = new FilterColorThreshold(combined_colormodel_);

  // Setup luminance classifier for light on/off detection
  light_scangrid_ = new ScanlineGrid(cam_width_, cam_height_, 1, 1);
  light_colormodel_ = new ColorModelLuminance(cfg_light_on_threshold_);
  light_classifier_ = new SimpleColorClassifier(
      light_scangrid_,
      light_colormodel_,
      cfg_light_on_min_points_,
      8,
      false,
      cfg_light_on_min_neighborhood_,
      0,
      C_WHITE);
  light_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

  // Setup black classifier
  black_scangrid_ = new ScanlineGrid(cam_width_, cam_height_, 1, 1);
  black_colormodel_ = new ColorModelBlack(cfg_black_threshold_);
  black_classifier_ = new SimpleColorClassifier(
    black_scangrid_,
    black_colormodel_,
    cfg_black_min_points_,
    6,
    false,
    cfg_black_min_neighborhood_,
    0,
    C_BLACK);
  black_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

  // Initialize frame rate detection
  uint loop_time = config->get_uint("/fawkes/mainapp/desired_loop_time");
  float fps = 1 / ((float)loop_time / 1000000.0);
  buflen_ = (unsigned int) pow(2, ceil(log2(fps)));
  logger->log_info(name(), "Buffer length: %d", buflen_);
  last_second_ = new Time(clock);

  // Open required blackboard interfaces
  for (int i = 0; i < MAX_SIGNALS; i++) {
    std::string iface_name = "machine_signal_";
    iface_name += std::to_string(i);
    bb_signal_states_.push_back(blackboard->open_for_writing<RobotinoLightInterface>(iface_name.c_str()));
  }
  bb_signal_compat_ = blackboard->open_for_writing<RobotinoLightInterface>("Light_State");

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
#ifdef __FIREVISION_CAMS_FILELOADER_H_
  camera_->dispose_buffer();
#endif
  delete camera_;
  delete roi_drawer_;
  delete last_second_;

  delete cls_red_.colormodel;
  delete cls_red_.classifier;
  delete cls_red_.scanline_grid;
  delete cls_red_.color_class;
  delete cls_green_.colormodel;
  delete cls_green_.classifier;
  delete cls_green_.scanline_grid;
  delete cls_green_.color_class;

  vision_master->unregister_thread(this);

  for (std::vector<RobotinoLightInterface *>::iterator bb_it = bb_signal_states_.begin();
      bb_it != bb_signal_states_.end(); bb_it++) {
    blackboard->close(*bb_it);
  }
  blackboard->close(bb_signal_compat_);

  delete shmbuf_;
  delete shmbuf_cam_;

  delete light_scangrid_;
  delete light_colormodel_;
  delete light_classifier_;

  delete black_scangrid_;
  delete black_colormodel_;
  delete black_classifier_;

  delete combined_colormodel_;
  delete color_filter_;
}


void MachineSignalThread::loop()
{
  std::list<ROI> *rois_R, *rois_G;
  MutexLocker lock(&cfg_mutex_);

  Time now(clock);
  if (now - last_second_ >= 125) {
    logger->log_error(name(), "Running too slow. Ignoring this frame!");
    return;
  }
  delete last_second_;
  last_second_ = new Time(clock);

  // Reallocate classifiers if their config changed
  if (unlikely(cfg_changed_)) {

    setup_classifier(&cls_red_);
    setup_classifier(&cls_green_);

    delete light_classifier_;
    light_classifier_ = new SimpleColorClassifier(
        light_scangrid_,
        new ColorModelLuminance(cfg_light_on_threshold_),
        cfg_light_on_min_points_,
        8,
        false,
        cfg_light_on_min_neighborhood_,
        0,
        C_WHITE);

    delete black_classifier_;
    delete black_colormodel_;
    black_colormodel_ = new ColorModelBlack(cfg_black_threshold_);
    black_classifier_ = new SimpleColorClassifier(
      black_scangrid_,
      black_colormodel_,
      cfg_black_min_points_,
      6,
      false,
      cfg_black_min_neighborhood_,
      0,
      C_BLACK);

    cfg_changed_ = false;
  }

#ifndef __FIREVISION_CAMS_FILELOADER_H_
  camera_->capture();
#endif

  if (unlikely(cfg_tuning_mode_)) {
    // Untreated copy of the cam image
    memcpy(shmbuf_cam_->buffer(), camera_->buffer(), shmbuf_cam_->data_size());
  }

  light_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
  black_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

  // Classify red & green in full picture
  cls_red_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
  rois_R = cls_red_.classifier->classify();

  if (rois_R->empty()) return;

  cls_green_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
  rois_G = cls_green_.classifier->classify();

  if (unlikely(cfg_tuning_mode_)) {
    drawn_rois_.clear();

    // Visualize color similarities in tuning buffer
    color_filter_->set_src_buffer(camera_->buffer(), ROI::full_image(cam_width_, cam_height_));
    color_filter_->set_dst_buffer(shmbuf_->buffer(), ROI::full_image(shmbuf_->width(), shmbuf_->height()));
    color_filter_->apply();

    if (!cfg_draw_processed_rois_) {
      drawn_rois_.insert(drawn_rois_.end(), rois_R->begin(), rois_R->end());
      drawn_rois_.insert(drawn_rois_.end(), rois_G->begin(), rois_G->end());
    }
  }

  // Create and group ROIs that make up the red, yellow and green lights of a signal
  std::list<signal_rois_t_> *signal_rois;
  bool at_delivery;

  if ((at_delivery = rois_G->begin()->width > cam_width_/3)) {
    signal_rois = create_delivery_rois(rois_R);
  }
  else {
    signal_rois = create_signal_rois(rois_R, rois_G);
  }

  // Reset all known signals to not-seen
  for (std::list<signal_state_t_>::iterator known_signal = known_signals_.begin();
      known_signal != known_signals_.end(); ++known_signal) {
    if (++(known_signal->unseen) > 1)
      known_signal->visibility = -1;
  }

  // Go through all signals from this frame...
  { std::list<signal_rois_t_>::iterator signal_it = signal_rois->begin();
  for (uint i=0; i < MAX_SIGNALS && signal_it != signal_rois->end(); ++i) {
    try {
      frame_state_t_ frame_state({
        get_light_state(signal_it->red_roi),
            get_light_state(signal_it->yellow_roi),
            get_light_state(signal_it->green_roi),
            signal_it->red_roi->start
      });

      // Signals with all lights off are impossible...
      if (frame_state.red || frame_state.yellow || frame_state.green) {
        // ... and match them to known signals based on the max_jitter tunable
        float dist_min = FLT_MAX;
        std::list<signal_state_t_>::iterator best_match;
        for (std::list<signal_state_t_>::iterator known_signal = known_signals_.begin();
            known_signal != known_signals_.end(); ++known_signal) {
          float dist = known_signal->distance(frame_state);
          if (dist < dist_min) {
            best_match = known_signal;
            dist_min = dist;
          }
        }
        if (dist_min < cfg_max_jitter_) {
          best_match->update(frame_state, signal_it);
        }
        else {
          // No historic match was found for the current signal
          signal_state_t_ *cur_state = new signal_state_t_(buflen_);
          cur_state->update(frame_state, signal_it);
          known_signals_.push_front(*cur_state);
          delete cur_state;
        }

        signal_it->red_roi->set_image_width(cam_width_);
        signal_it->red_roi->set_image_height(cam_height_);
        signal_it->yellow_roi->set_image_width(cam_width_);
        signal_it->yellow_roi->set_image_height(cam_height_);
        signal_it->green_roi->set_image_width(cam_width_);
        signal_it->green_roi->set_image_height(cam_height_);

        if (unlikely(cfg_tuning_mode_ && cfg_draw_processed_rois_)) {
          drawn_rois_.push_back(signal_it->red_roi);
          drawn_rois_.push_back(signal_it->yellow_roi);
          drawn_rois_.push_back(signal_it->green_roi);
        }
      }

      delete signal_it->red_roi;
      signal_it->red_roi = NULL;
      delete signal_it->yellow_roi;
      signal_it->yellow_roi = NULL;
      delete signal_it->green_roi;
      signal_it->green_roi = NULL;
    }
    catch (OutOfBoundsException &e){
      logger->log_error(name(), "Signal at %d,%d: Invalid ROI: %s",
        signal_it->red_roi->start.x, signal_it->red_roi->start.y, e.what_no_backtrace());
    }
    signal_it++;
  }
  }

  delete signal_rois;
  signal_rois = NULL;

  if (unlikely(cfg_tuning_mode_)) {
    // Visualize the signals and bright spots we found
    roi_drawer_->set_rois(&drawn_rois_);
    roi_drawer_->set_src_buffer(shmbuf_->buffer(), ROI::full_image(cam_width_, cam_height_), 0);
    roi_drawer_->set_dst_buffer(shmbuf_->buffer(), NULL);
    roi_drawer_->apply();
  }

  // Throw out the signals with the worst visibility histories
  known_signals_.sort(sort_signal_states_by_visibility_);
  while (known_signals_.size() > MAX_SIGNALS)
    known_signals_.pop_back();

  // Then sort geometrically
  if (at_delivery) known_signals_.sort(sort_signal_states_by_x_);
  else known_signals_.sort(sort_signal_states_by_area_);

  // Update blackboard with the current information
  std::list<signal_state_t_>::iterator known_signal = known_signals_.begin();
  for (int i = 0; i < MAX_SIGNALS && known_signal != known_signals_.end(); i++) {
    bb_signal_states_[i]->set_red(known_signal->red);
    bb_signal_states_[i]->set_yellow(known_signal->yellow);
    bb_signal_states_[i]->set_green(known_signal->green);
    bb_signal_states_[i]->set_visibility_history(
      known_signal->unseen > 1 ? -1 : known_signal->visibility);
    bb_signal_states_[i]->set_ready(known_signal->visibility >= (long int) buflen_/2);
    bb_signal_states_[i]->write();

    known_signal++;
  }
  bb_signal_compat_->set_red(known_signals_.begin()->red);
  bb_signal_compat_->set_yellow(known_signals_.begin()->yellow);
  bb_signal_compat_->set_green(known_signals_.begin()->green);
  bb_signal_compat_->set_visibility_history(
    known_signals_.begin()->unseen > 1 ? -1 : known_signals_.begin()->visibility);
  bb_signal_compat_->set_ready(known_signals_.begin()->visibility >= (long int) buflen_/2);
  bb_signal_compat_->write();

  delete rois_R;
  delete rois_G;

#ifndef __FIREVISION_CAMS_FILELOADER_H_
  camera_->dispose_buffer();
#endif
}

/**
 * Determine if a given color ROI can be considered lit according to the luminance model.
 * @param light the ROI to be considered
 * @return true if "lit" according to our criteria, false otherwise.
 */
bool MachineSignalThread::get_light_state(firevision::ROI *light)
{
  light_scangrid_->set_roi(light);
  light_scangrid_->reset();
  std::list<ROI> *bright_rois = light_classifier_->classify();

  for (std::list<ROI>::iterator roi_it = bright_rois->begin(); roi_it != bright_rois->end(); ++roi_it) {
    float area_ratio = (float)(roi_it->width * roi_it->height) / (float)(light->width * light->height);
    if (roi_aspect_ok(roi_it) && area_ratio > cfg_light_on_min_area_cover_) {
      if (unlikely(cfg_tuning_mode_ && cfg_draw_processed_rois_))
        drawn_rois_.push_back(*roi_it);
      delete bright_rois;
      return true;
    }
  }
  delete bright_rois;
  return false;
}

/**
 * Look for red and green ROIs that are likely to be part of a single signal. A red and a green ROI are considered
 * a match if the red one is above the green one and there's space for a yellow one to fit in between. Unmatched ROIs
 * and those that violate certain (configurable) sanity criteria are thrown out.
 * @param rois_R A list of red ROIs
 * @param rois_G A list of green ROIs
 * @return A list of ROIs in a struct that contains matching red, yellow and green ROIs
 */
std::list<MachineSignalThread::signal_rois_t_> *MachineSignalThread::create_signal_rois(
    std::list<ROI> *rois_R,
    std::list<ROI> *rois_G)
{
  std::list<MachineSignalThread::signal_rois_t_> *rv = new std::list<MachineSignalThread::signal_rois_t_>();

  for (std::list<ROI>::iterator it_R = rois_R->begin(); it_R != rois_R->end(); ++it_R) {

    if (!(roi_width_ok(it_R) && roi_aspect_ok(it_R)))
      continue;

    it_R->height = it_R->width;

    for (std::list<ROI>::iterator it_G = rois_G->begin(); it_G != rois_G->end(); ++it_G) {

      if (roi_width_ok(it_G) && rois_x_aligned(it_R, it_G) &&
          rois_vspace_ok(it_R, it_G)) {
        if (!rois_similar_width(it_R, it_G)) {
          int wdiff = it_G->width - it_R->width;
          it_G->start.x += wdiff/2;
          it_G->width -= wdiff/2;
        }
        // Once we got through here it_G should have a pretty sensible green ROI.
        it_G->height = it_G->width;
        uint start_x = (it_R->start.x + it_G->start.x) / 2;
        uint height = (it_R->height + it_G->height) / 2;
        uint width = (it_R->width + it_G->width) / 2;
        uint r_end_y = it_R->start.y + it_R->height;
        uint start_y = r_end_y + (int)(it_G->start.y - r_end_y - height)/2;
        ROI *roi_Y = new ROI(start_x, start_y,
          width, height,
          it_R->image_width, it_R->image_height);
        roi_Y->color = C_YELLOW;

        ROI *roi_R = new ROI(*it_R);
        ROI *roi_G = new ROI(*it_G);
        rv->push_back({roi_R, roi_Y, roi_G});
      }
    }
  }
  return rv;
}

std::list<MachineSignalThread::signal_rois_t_> *MachineSignalThread::create_delivery_rois(
  std::list<ROI> *rois_R)
{
  std::list<MachineSignalThread::signal_rois_t_> *rv = new std::list<MachineSignalThread::signal_rois_t_>();

  std::list<ROI>::iterator it_R = rois_R->begin();

  while(it_R != rois_R->end()) {
    if (it_R->start.y > cam_height_/2) {
      it_R = rois_R->erase(it_R);
    }
    else ++it_R;
  }

  rois_R->sort(sort_rois_by_area_);

  unsigned int i = 0;
  for (it_R = rois_R->begin(); it_R != rois_R->end() && i++ < MAX_SIGNALS; ++it_R) {
    try {

      ROI *roi_R = new ROI(*it_R);

      long int start_x, start_y;

      start_y = (long int)it_R->start.y - (long int)it_R->width/3;
      if (start_y < 0) start_y = 0;
      ROI check_black_top(it_R->start.x, start_y, it_R->width, it_R->width/2,
        cam_width_, cam_height_);

      if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
        drawn_rois_.push_back(check_black_top);

      black_scangrid_->set_roi(&check_black_top);
      std::list<ROI> *black_rois_top = black_classifier_->classify();

      if (!black_rois_top->empty()) {
        if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
          drawn_rois_.insert(drawn_rois_.end(), black_rois_top->begin(), black_rois_top->end());
        ROI black_top = *(black_rois_top->begin());
        delete black_rois_top;

        if (black_top.width > roi_R->width * 0.5)
          roi_R->width = (black_top.width + roi_R->width) / 2;
        //roi_R->start.y = black_top.start.y + black_top.height;
      }
      roi_R->height = roi_R->width;

      start_y = (long int)roi_R->start.y + (long int)roi_R->width; // add width since it's more reliable than height
      ROI *roi_Y = new ROI(roi_R->start.x, start_y, roi_R->width, roi_R->height, roi_R->image_width, roi_R->image_height);
      roi_Y->color = C_YELLOW;
      ROI *roi_G = new ROI(roi_R->start.x, start_y + roi_R->height, roi_R->width, roi_R->height, roi_R->image_width,
        roi_R->image_height);
      roi_G->color = C_GREEN;

      ROI check_black_bottom(*roi_Y);
      start_x = (long int)roi_Y->start.x - (long int)roi_Y->width/4;
      if (start_x < 0) start_x = 0;
      check_black_bottom.set_start(start_x, roi_Y->start.y + roi_Y->height/2);
      check_black_bottom.set_width(roi_Y->width * 1.5);
      check_black_bottom.set_height(roi_Y->height * 2.5);
      check_black_bottom.color = C_BACKGROUND;

      if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
        drawn_rois_.push_back(check_black_bottom);

      black_scangrid_->set_roi(&check_black_bottom);
      std::list<ROI> *black_rois_bottom = black_classifier_->classify();

      if (!black_rois_bottom->empty()) {
        ROI black_bottom = *(black_rois_bottom->begin());
        delete black_rois_bottom;
        unsigned int height_adj = (black_bottom.start.y - roi_R->start.y) / 3;
        roi_R->height = height_adj;
        roi_Y->height = height_adj;
        roi_G->height = height_adj;
      }

      ROI check_black_green(*roi_G);
      check_black_green.color = C_BACKGROUND;
      if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
        drawn_rois_.push_back(check_black_green);
      black_scangrid_->set_roi(&check_black_green);
      std::list<ROI> *black_in_green = black_classifier_->classify();

      unsigned int black_in_green_area = 0;
      unsigned int green_area = roi_G->width * roi_G->height;

      if (!black_in_green->empty()) {
        black_in_green_area = black_in_green->begin()->width * black_in_green->begin()->height;
        if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
          drawn_rois_.insert(drawn_rois_.end(), black_in_green->begin(), black_in_green->end());
      }

      roi_G->set_image_width(cam_width_);
      roi_G->set_image_height(cam_height_);

      if (((float)black_in_green_area / (float)green_area) < 0.2) {
        rv->push_back({roi_R, roi_Y, roi_G});

      }
    }
    catch (OutOfBoundsException &e) {
      logger->log_error(name(), "%s", e.what());
    }
  }

  cls_green_.scanline_grid->set_roi(it_R->full_image(cam_width_, cam_height_));
  cls_green_.scanline_grid->reset();

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
  return width_ratio >= 1/cfg_roi_max_width_ratio_ && width_ratio <= cfg_roi_max_width_ratio_;
}

bool MachineSignalThread::rois_x_aligned(std::list<ROI>::iterator r1, std::list<ROI>::iterator r2) {
  float avg_width = (r1->width + r2->width) / 2.0f;
  return abs(r1->start.x - r2->start.x) / avg_width < cfg_roi_xalign_;
}

bool MachineSignalThread::roi_aspect_ok(std::list<ROI>::iterator r) {
  float aspect_ratio = (float)r->width / (float)r->height;
  return aspect_ratio > 1/cfg_roi_max_aspect_ratio_ && aspect_ratio < cfg_roi_max_aspect_ratio_;
}

bool MachineSignalThread::rois_vspace_ok(std::list<ROI>::iterator r1, std::list<ROI>::iterator r2) {
  float avg_height = (r1->height + r2->height) / 2.0f;
  int dist = r2->start.y - (r1->start.y + r1->height);
  return dist > 0.6 * avg_height && dist < 1.7 * avg_height;
}


void MachineSignalThread::config_value_erased(const char *path) {}
void MachineSignalThread::config_tag_changed(const char *new_tag) {}
void MachineSignalThread::config_comment_changed(const Configuration::ValueIterator *v) {}

void MachineSignalThread::config_value_changed(const Configuration::ValueIterator *v)
{
  if (v->valid()) {
    std::string path = v->path();
    std::string sufx = path.substr(strlen(CFG_PREFIX));
    std::string color_pfx = sufx.substr(0, sufx.substr(1).find("/")+1);
    std::string full_pfx = CFG_PREFIX + color_pfx;
    std::string opt = path.substr(full_pfx.length());

    MutexLocker lock(&cfg_mutex_);

    if (color_pfx == "/red" || color_pfx == "/green") {
      color_classifier_context_t_ *classifier = NULL;
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
      else if (opt == "/scangrid_x_offset")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_scangrid_x_offset), v->get_uint());
      else if (opt == "/scangrid_y_offset")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(classifier->cfg_scangrid_y_offset), v->get_uint());
    }
    else if (color_pfx == "/bright_light") {
      if (opt == "/min_brightness")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(cfg_light_on_threshold_), v->get_uint());
      else if (opt == "/min_points")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(cfg_light_on_min_points_), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(cfg_light_on_min_neighborhood_), v->get_uint());
      else if (opt == "/min_area_cover")
        cfg_light_on_min_area_cover_ = v->get_float();
    }
    else if (color_pfx == "/black") {
      if (opt == "/max_luminance")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(cfg_black_threshold_), v->get_uint());
      else if (opt == "/min_points")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(cfg_black_min_points_), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        cfg_changed_ = cfg_changed_ || test_set_cfg_value(&(cfg_black_min_neighborhood_), v->get_uint());
    }
    else if (color_pfx == "") {
      if (opt == "/roi_max_aspect_ratio")
        cfg_roi_max_aspect_ratio_ = v->get_float();
      else if (opt == "/roi_max_r2g_width_ratio")
        cfg_roi_max_width_ratio_ = v->get_float();
      else if (opt == "/roi_xalign_by_width")
        cfg_roi_xalign_ = v->get_float();
      else if (opt == "/tuning_mode")
        cfg_tuning_mode_ = v->get_bool();
      else if (opt == "/max_jitter")
        cfg_max_jitter_ = v->get_float();
      else if (opt == "/draw_processed_rois")
        cfg_draw_processed_rois_ = v->get_bool();
    }
  }
}

