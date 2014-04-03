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

#include "pipeline_thread.h"
#include <fvfilters/colorthreshold.h>
#include <aspect/logging.h>
#include <fvutils/color/colorspaces.h>
#include <cstring>
#include <core/threading/mutex_locker.h>
#include <cmath>
#include <cfloat>

using namespace fawkes;
using namespace firevision;

MachineSignalPipelineThread::MachineSignalPipelineThread()
    : Thread("MachineSignal", Thread::OPMODE_CONTINUOUS),
      VisionAspect(VisionAspect::CONTINUOUS),
      ConfigurationChangeHandler(CFG_PREFIX)
{
  new_data_ = false;

  cam_width_ = 0;
  cam_height_ = 0;
  cfg_light_on_threshold_ = 0;
  camera_ = NULL;

  time_wait_ = NULL;
  cfg_fps_ = 0;
  desired_frametime_ = 0;

  cfy_ctxt_red_.colormodel = NULL;
  cfy_ctxt_red_.classifier = NULL;
  cfy_ctxt_red_.scanline_grid = NULL;
  cfy_ctxt_red_.color_expect = C_RED;
  cfy_ctxt_red_delivery_.colormodel = NULL;
  cfy_ctxt_red_delivery_.classifier = NULL;
  cfy_ctxt_red_delivery_.scanline_grid = NULL;
  cfy_ctxt_red_delivery_.color_expect = C_RED;
  cfy_ctxt_green_.colormodel = NULL;
  cfy_ctxt_green_.classifier = NULL;
  cfy_ctxt_green_.scanline_grid = NULL;
  cfy_ctxt_green_.color_expect = C_GREEN;

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
  bb_delivery_switch_ = NULL;
  bb_enable_switch_ = NULL;
  cfg_enable_switch_ = false;
  cfg_delivery_mode_ = delivery_switch_t_::AUTO;
}

MachineSignalPipelineThread::~MachineSignalPipelineThread() {}

bool MachineSignalPipelineThread::color_data_consistent(
  color_classifier_context_t_ *color_data) {
  bool rv = (color_data->cfg_ref_col.size() == (3 * color_data->cfg_chroma_thresh.size()))
      && (color_data->cfg_ref_col.size() == (3 * color_data->cfg_sat_thresh.size()))
      && (color_data->cfg_ref_col.size() == (3 * color_data->cfg_luma_thresh.size()));
  return rv;
}

void MachineSignalPipelineThread::setup_color_classifier(color_classifier_context_t_ *color_data)
{
  delete color_data->classifier;
  delete color_data->colormodel;
  delete color_data->scanline_grid;

  for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it = color_data->color_class.begin();
      it != color_data->color_class.end(); it++) {
    delete *it;
  }
  color_data->color_class.clear();

  // Update the color class used by the combined color model for the tuning filter
  std::vector<int>::iterator it_sat = color_data->cfg_sat_thresh.begin();
  std::vector<int>::iterator it_luma = color_data->cfg_luma_thresh.begin();
  std::vector<unsigned int>::iterator it_ref = color_data->cfg_ref_col.begin();
  std::vector<int>::iterator it_chroma = color_data->cfg_chroma_thresh.begin();
  while (it_chroma != color_data->cfg_chroma_thresh.end()) {
    std::vector<unsigned int> ref_col(it_ref, it_ref + 3);
    ColorModelSimilarity::color_class_t *color_class = new ColorModelSimilarity::color_class_t(
      color_data->color_expect,
      ref_col,
      *(it_chroma++),
      *(it_sat++),
      *(it_luma++)
    );
    it_ref += 3;
    color_data->color_class.push_back(color_class);
  }

  color_data->colormodel = new ColorModelSimilarity();
  color_data->colormodel->add_colors(color_data->color_class);
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



void MachineSignalPipelineThread::init()
{
  // Configure camera
  cfg_camera_ = config->get_string(CFG_PREFIX "/camera");
  setup_camera();

  shmbuf_ = new SharedMemoryImageBuffer("machine_signal", YUV422_PLANAR, cam_width_, cam_height_);
  shmbuf_cam_ = new SharedMemoryImageBuffer("machine_cam", YUV422_PLANAR, cam_width_, cam_height_);
  roi_drawer_ = new FilterROIDraw(&drawn_rois_, FilterROIDraw::DASHED_HINT);

  // Configure RED classifier
  cfy_ctxt_red_.cfg_ref_col = config->get_uints(CFG_PREFIX "/red/reference_color");
  cfy_ctxt_red_.cfg_chroma_thresh = config->get_ints(CFG_PREFIX "/red/chroma_thresh");
  cfy_ctxt_red_.cfg_sat_thresh = config->get_ints(CFG_PREFIX "/red/saturation_thresh");
  cfy_ctxt_red_.cfg_luma_thresh = config->get_ints(CFG_PREFIX "/red/luma_thresh");
  cfy_ctxt_red_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/red/min_points");
  cfy_ctxt_red_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/red/basic_roi_size");
  cfy_ctxt_red_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX "/red/neighborhood_min_match");
  cfy_ctxt_red_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/red/scangrid_x_offset");
  cfy_ctxt_red_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/red/scangrid_y_offset");

  setup_color_classifier(&cfy_ctxt_red_);

  // Configure RED classifier for delivery zone
  cfy_ctxt_red_delivery_.cfg_ref_col = config->get_uints(CFG_PREFIX "/red_delivery/reference_color");
  cfy_ctxt_red_delivery_.cfg_chroma_thresh = config->get_ints(CFG_PREFIX "/red_delivery/chroma_thresh");
  cfy_ctxt_red_delivery_.cfg_sat_thresh = config->get_ints(CFG_PREFIX "/red_delivery/saturation_thresh");
  cfy_ctxt_red_delivery_.cfg_luma_thresh = config->get_ints(CFG_PREFIX "/red_delivery/luma_thresh");
  cfy_ctxt_red_delivery_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/red_delivery/min_points");
  cfy_ctxt_red_delivery_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/red_delivery/basic_roi_size");
  cfy_ctxt_red_delivery_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX "/red_delivery/neighborhood_min_match");
  cfy_ctxt_red_delivery_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/red_delivery/scangrid_x_offset");
  cfy_ctxt_red_delivery_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/red_delivery/scangrid_y_offset");

  setup_color_classifier(&cfy_ctxt_red_delivery_);

  // Configure GREEN classifier
  cfy_ctxt_green_.cfg_ref_col = config->get_uints(CFG_PREFIX "/green/reference_color");
  cfy_ctxt_green_.cfg_chroma_thresh = config->get_ints(CFG_PREFIX "/green/chroma_thresh");
  cfy_ctxt_green_.cfg_sat_thresh = config->get_ints(CFG_PREFIX "/green/saturation_thresh");
  cfy_ctxt_green_.cfg_luma_thresh = config->get_ints(CFG_PREFIX "/green/luma_thresh");
  cfy_ctxt_green_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/green/min_points");
  cfy_ctxt_green_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/green/basic_roi_size");
  cfy_ctxt_green_.cfg_roi_neighborhood_min_match = config->get_int(CFG_PREFIX "/green/neighborhood_min_match");
  cfy_ctxt_green_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/green/scangrid_x_offset");
  cfy_ctxt_green_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/green/scangrid_y_offset");

  setup_color_classifier(&cfy_ctxt_green_);

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
  cfg_roi_green_horizon = config->get_uint(CFG_PREFIX "/roi_green_horizon");
  cfg_tuning_mode_ = config->get_bool(CFG_PREFIX "/tuning_mode");
  cfg_max_jitter_ = config->get_float(CFG_PREFIX "/max_jitter");
  cfg_draw_processed_rois_ = config->get_bool(CFG_PREFIX "/draw_processed_rois");
  cfg_enable_switch_ = config->get_bool(CFG_PREFIX "/start_enabled");
  cfg_fps_ = config->get_uint(CFG_PREFIX "/fps");
  cfg_debug_blink_ = config->get_bool(CFG_PREFIX "/debug_blink");
  cfg_debug_processing_ = config->get_bool(CFG_PREFIX "/debug_processing");

  std::string delivery_mode = config->get_string(CFG_PREFIX "/delivery_mode");
  if (delivery_mode == "on") cfg_delivery_mode_ = delivery_switch_t_::ON;
  else if (delivery_mode == "off") cfg_delivery_mode_ = delivery_switch_t_::OFF;
  else if (delivery_mode == "auto") cfg_delivery_mode_ = delivery_switch_t_::AUTO;
  else throw Exception("delivery_mode: invalid config value: %s", delivery_mode.c_str());

  // Setup combined ColorModel for tuning filter
  combined_colormodel_ = new ColorModelSimilarity();
  combined_colormodel_->add_colors(cfy_ctxt_red_.color_class);
  combined_colormodel_->add_colors(cfy_ctxt_green_.color_class);
  combined_colormodel_->add_colors(cfy_ctxt_red_delivery_.color_class);
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

  // Initialize frame rate detection & buffer lengths for blinking detection
  buflen_ = (unsigned int) ceil(cfg_fps_/4) + 1;
  desired_frametime_ = 1/(double)cfg_fps_;
  time_wait_ = new TimeWait(clock, (long int)(1000000 * desired_frametime_));
  buflen_ += 1 - (buflen_ % 2); // make sure buflen is uneven
  logger->log_info(name(), "Buffer length: %d", buflen_);
  last_second_ = new Time(clock);

  bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>("light_front_switch");
  bb_enable_switch_->set_enabled(cfg_enable_switch_);
  bb_delivery_switch_ = blackboard->open_for_writing<SwitchInterface>("machine_signal_delivery_mode");

  config->add_change_handler(this);
}


void MachineSignalPipelineThread::setup_camera()
{
  camera_ = vision_master->register_for_camera(cfg_camera_.c_str(), this);
  cam_width_ = camera_->pixel_width();
  cam_height_ = camera_->pixel_height();
}


void MachineSignalPipelineThread::finalize()
{
  delete camera_;
  delete roi_drawer_;
  delete last_second_;

  delete cfy_ctxt_red_.colormodel;
  delete cfy_ctxt_red_.classifier;
  delete cfy_ctxt_red_.scanline_grid;
  for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it = cfy_ctxt_red_.color_class.begin();
      it != cfy_ctxt_red_.color_class.end(); it++) {
    delete *it;
  }
  delete cfy_ctxt_red_delivery_.colormodel;
  delete cfy_ctxt_red_delivery_.classifier;
  delete cfy_ctxt_red_delivery_.scanline_grid;
  for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it = cfy_ctxt_red_delivery_.color_class.begin();
      it != cfy_ctxt_red_delivery_.color_class.end(); it++) {
    delete *it;
  }
  delete cfy_ctxt_green_.colormodel;
  delete cfy_ctxt_green_.classifier;
  delete cfy_ctxt_green_.scanline_grid;
  for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it = cfy_ctxt_green_.color_class.begin();
      it != cfy_ctxt_green_.color_class.end(); it++) {
    delete *it;
  }

  blackboard->close(bb_enable_switch_);
  blackboard->close(bb_delivery_switch_);

  vision_master->unregister_thread(this);

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

bool MachineSignalPipelineThread::lock_if_new_data()
{
  data_mutex_.lock();
  if (new_data_) {
    return true;
  } else {
    data_mutex_.unlock();
    return false;
  }
}

void MachineSignalPipelineThread::unlock()
{
  data_mutex_.unlock();
}

std::list<SignalState> &MachineSignalPipelineThread::get_known_signals()
{
  return known_signals_;
}

std::list<SignalState>::iterator MachineSignalPipelineThread::get_best_signal()
{
  return best_signal_;
}

void MachineSignalPipelineThread::loop()
{
  time_wait_->mark_start();

  Time now(clock);
  double frametime = now - last_second_;
  last_second_ = &(last_second_->stamp());
  if (frametime >= desired_frametime_ * 1.02) {
    logger->log_error(name(), "Running too slow (%f sec/frame). Data will be bad!", frametime);
  }

  while (!bb_enable_switch_->msgq_empty()) {
    if (SwitchInterface::DisableSwitchMessage *msg = bb_enable_switch_->msgq_first_safe(msg)) {
      cfg_enable_switch_ = false;
    }
    if (SwitchInterface::EnableSwitchMessage *msg = bb_enable_switch_->msgq_first_safe(msg)) {
      cfg_enable_switch_ = true;
    }
    bb_enable_switch_->msgq_pop();
  }
  while (!bb_delivery_switch_->msgq_empty()) {
    if (SwitchInterface::DisableSwitchMessage *msg = bb_delivery_switch_->msgq_first_safe(msg)) {
      cfg_delivery_mode_ = delivery_switch_t_::OFF;
    }
    if (SwitchInterface::EnableSwitchMessage *msg = bb_delivery_switch_->msgq_first_safe(msg)) {
      cfg_delivery_mode_ = delivery_switch_t_::ON;
    }
    bb_delivery_switch_->msgq_pop();
  }
  if (cfg_delivery_mode_ == delivery_switch_t_::OFF) {
    bb_delivery_switch_->set_enabled(false);
    bb_delivery_switch_->write();
  }
  if (cfg_delivery_mode_ == delivery_switch_t_::ON) {
    bb_delivery_switch_->set_enabled(true);
    bb_delivery_switch_->write();
  }
  bb_enable_switch_->set_enabled(cfg_enable_switch_);
  bb_enable_switch_->write();

  if (bb_enable_switch_->is_enabled()) {

    std::list<ROI> *rois_R, *rois_G;
    MutexLocker lock(&cfg_mutex_);

    // Reallocate classifiers if their config changed
    if (unlikely(cfg_changed_
      && color_data_consistent(&cfy_ctxt_red_)
    && color_data_consistent(&cfy_ctxt_red_delivery_)
    && color_data_consistent(&cfy_ctxt_green_) )) {

      combined_colormodel_->delete_colors();

      setup_color_classifier(&cfy_ctxt_red_);
      setup_color_classifier(&cfy_ctxt_green_);
      setup_color_classifier(&cfy_ctxt_red_delivery_);

      combined_colormodel_->add_colors(cfy_ctxt_red_.color_class);
      combined_colormodel_->add_colors(cfy_ctxt_red_delivery_.color_class);
      combined_colormodel_->add_colors(cfy_ctxt_green_.color_class);

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

    camera_->capture();

    if (unlikely(cfg_tuning_mode_)) {
      // Untreated copy of the cam image
      memcpy(shmbuf_cam_->buffer(), camera_->buffer(), shmbuf_cam_->data_size());
    }

    light_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
    black_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

    // First classify green to detect if we're looking at the delivery zone
    cfy_ctxt_green_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
    rois_G = cfy_ctxt_green_.classifier->classify();

    bool at_delivery;
    if (cfg_delivery_mode_ == delivery_switch_t_::AUTO) {
      at_delivery = rois_G->begin()->width > cam_width_/3;
      bb_delivery_switch_->set_enabled(at_delivery);
      bb_delivery_switch_->write();
    }
    else {
      at_delivery = bb_delivery_switch_->is_enabled();
    }

    // Then use the appropriate classifier for red
    if (at_delivery) {
      cfy_ctxt_red_delivery_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
      rois_R = cfy_ctxt_red_delivery_.classifier->classify();
    }
    else {
      cfy_ctxt_red_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
      rois_R = cfy_ctxt_red_.classifier->classify();
    }

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
    std::list<SignalState::signal_rois_t_> *signal_rois;
    if (at_delivery) {
      signal_rois = create_delivery_rois(rois_R);
    }
    else {
      signal_rois = create_signal_rois(rois_R, rois_G);
    }

    data_mutex_.lock();

    // Reset all known signals to not-seen
    for (std::list<SignalState>::iterator known_signal = known_signals_.begin();
        known_signal != known_signals_.end(); ++known_signal) {
      known_signal->inc_unseen();
    }

    // Go through all signals from this frame...
    { std::list<SignalState::signal_rois_t_>::iterator signal_it = signal_rois->begin();
    for (uint i=0; i < MAX_SIGNALS && signal_it != signal_rois->end(); ++i) {
      try {
        SignalState::frame_state_t_ frame_state({
          get_light_state(signal_it->red_roi),
              get_light_state(signal_it->yellow_roi),
              get_light_state(signal_it->green_roi),
        });

        // ... and match them to known signals based on the max_jitter tunable
        float dist_min = FLT_MAX;
        std::list<SignalState>::iterator best_match;
        for (std::list<SignalState>::iterator known_signal = known_signals_.begin();
            known_signal != known_signals_.end(); ++known_signal) {
          float dist = known_signal->distance(signal_it);
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
          SignalState *cur_state = new SignalState(buflen_, logger);
          cur_state->update(frame_state, signal_it);
          known_signals_.push_front(*cur_state);
          delete cur_state;
        }

        // Classifiers sometimes do strange things to the image width/height, so reset them here...
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

        delete signal_it->red_roi;
        signal_it->red_roi = NULL;
        delete signal_it->yellow_roi;
        signal_it->yellow_roi = NULL;
        delete signal_it->green_roi;
        signal_it->green_roi = NULL;
      }
      catch (OutOfBoundsException &e){
        logger->log_error(name(), "Signal at %d,%d: Invalid ROI: %s",
          signal_it->red_roi->start.x, signal_it->red_roi->start.y, e.what());
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
    if (at_delivery) {
      // Ordering is critical here
      best_signal_ = known_signals_.begin();
      known_signals_.sort(sort_signal_states_by_x_);
    }
    else {
      // Ordering is critical here
      known_signals_.sort(sort_signal_states_by_area_);
      best_signal_ = known_signals_.begin();
    }
    ///*
    if (unlikely(cfg_debug_blink_)) {
      logger->log_info(name(), best_signal_->get_debug_R());
      logger->log_info(name(), best_signal_->get_debug_Y());
      logger->log_info(name(), best_signal_->get_debug_G());
      logger->log_info(name(), "=================");//*/
    }
    new_data_ = true;
    data_mutex_.unlock();


    delete rois_R;
    delete rois_G;

    camera_->dispose_buffer();
  }
  time_wait_->wait();
}

/**
 * Determine if a given color ROI can be considered lit according to the luminance model.
 * @param light the ROI to be considered
 * @return true if "lit" according to our criteria, false otherwise.
 */
bool MachineSignalPipelineThread::get_light_state(firevision::ROI *light)
{
  light_scangrid_->set_roi(light);
  light_scangrid_->reset();
  std::list<ROI> *bright_rois = light_classifier_->classify();

  for (std::list<ROI>::iterator roi_it = bright_rois->begin(); roi_it != bright_rois->end(); ++roi_it) {
    float area_ratio = (float)(roi_it->width * roi_it->height) / (float)(light->width * light->height);
    if (roi_aspect_ok(*roi_it) && area_ratio > cfg_light_on_min_area_cover_) {
      if (unlikely(cfg_tuning_mode_))
        drawn_rois_.push_back(*roi_it);
      delete bright_rois;
      return true;
    }
  }
  delete bright_rois;
  return false;
}

bool MachineSignalPipelineThread::roi1_oversize(ROI &r1, ROI &r2) {
  return !rois_similar_width(r1, r2)
      && roi_aspect_ok(r2)
      && (r2.start.x + r2.width/10) - r1.start.x > 0
      && (r1.start.x + r1.width*1.1) - (r2.start.x + r2.width) > 0;
}


/**
 * Look for red and green ROIs that are likely to be part of a single signal. A red and a green ROI are considered
 * a match if the red one is above the green one and there's space for a yellow one to fit in between. Unmatched ROIs
 * and those that violate certain (configurable) sanity criteria are thrown out.
 * @param rois_R A list of red ROIs
 * @param rois_G A list of green ROIs
 * @return A list of ROIs in a struct that contains matching red, yellow and green ROIs
 */
std::list<SignalState::signal_rois_t_> *MachineSignalPipelineThread::create_signal_rois(
    std::list<ROI> *rois_R,
    std::list<ROI> *rois_G)
{
  std::list<SignalState::signal_rois_t_> *rv = new std::list<SignalState::signal_rois_t_>();

  if (cfg_debug_processing_) debug_proc_string_ = "";

  for (std::list<ROI>::iterator it_R = rois_R->begin(); it_R != rois_R->end(); ++it_R) {
    bool ok = false;

    if (!(roi_width_ok(*it_R))) continue;

    if (it_R->height > it_R->width) {
      it_R->height = it_R->width;
    }

    std::list<ROI>::iterator it_G = rois_G->begin();
    while(it_G != rois_G->end()) {
      ok = false;
      int vspace = it_G->start.y - (it_R->start.y + it_R->height);

      if (roi_width_ok(*it_G) && rois_x_aligned(*it_R, *it_G) &&
          it_G->start.y > cfg_roi_green_horizon) {

        if (cfg_debug_processing_) debug_proc_string_ += "g:W rg:A";

        ROI *roi_R = new ROI(*it_R);
        ROI *roi_G = new ROI(*it_G);

        if (roi1_oversize(*roi_R, *roi_G)
            && vspace > 0 && vspace < roi_G->height * 1.5) {
          roi_R->start.x = roi_G->start.x;
          roi_R->width = roi_G->width;
          if (roi_R->width > cam_width_) roi_R->width = cam_width_ - roi_R->start.x;
          int r_end = roi_G->start.y - roi_G->height;
          roi_R->height = r_end - roi_R->start.y;
          if (roi_R->start.y + roi_R->height > cam_height_) roi_R->height = cam_height_ - roi_R->start.y;

          if (cfg_debug_processing_) debug_proc_string_ += " r:O";
        }

        if (roi1_oversize(*roi_G, *it_R)
            && vspace > 0 && vspace < it_R->height * 1.5) {
          roi_G->start.x = it_R->start.x;
          roi_G->width = it_R->width;
          if (roi_G->width > cam_width_) roi_G->width = cam_width_ - roi_G->start.x;
          roi_G->height = it_R->height;
          if (roi_G->start.y + roi_G->height > cam_height_) roi_G->height = cam_height_ - roi_G->start.y;

          if (cfg_debug_processing_) debug_proc_string_ += " g:O";
        }

        if (rois_vspace_ok(*roi_R, *roi_G)) {
          if (cfg_debug_processing_) debug_proc_string_ += " V";

          if (!rois_similar_width(*roi_R, *roi_G)) {
            int wdiff = roi_G->width - it_R->width;
            int start_x = roi_G->start.x + wdiff/2;
            if (start_x < 0) start_x = 0;
            it_G->start.x = start_x;
            int width = roi_G->width - wdiff/2;
            if ((unsigned int)(start_x + width) > cam_width_) width = cam_width_ - start_x;
            roi_G->width = width;
            if (cfg_debug_processing_) debug_proc_string_ += " !W";
          }

          // Once we got through here it_G should have a pretty sensible green ROI.
          roi_G->height = roi_G->width;
          uint start_x = (roi_R->start.x + roi_G->start.x) / 2;
          uint height = (roi_R->height + roi_G->height) / 2;
          uint width = (roi_R->width + roi_G->width) / 2;
          uint r_end_y = roi_R->start.y + roi_G->height;
          uint start_y = r_end_y + (int)(roi_G->start.y - r_end_y - height)/2;
          ROI *roi_Y = new ROI(start_x, start_y,
            width, height,
            roi_R->image_width, roi_R->image_height);
          roi_Y->color = C_YELLOW;

          rv->push_back({roi_R, roi_Y, roi_G});
          ok = true;
          it_G = rois_G->erase(it_G);
        }
        else {
          delete roi_G;
          delete roi_R;
        }
      }
      if (!ok) {
        ++it_G;
      }
    }
    if (unlikely(!ok && cfg_debug_processing_)) logger->log_debug(name(), "field proc: %s", debug_proc_string_.c_str());
  }
  return rv;
}

std::list<SignalState::signal_rois_t_> *MachineSignalPipelineThread::create_delivery_rois(
  std::list<ROI> *rois_R)
{
  std::list<SignalState::signal_rois_t_> *rv = new std::list<SignalState::signal_rois_t_>();

  std::list<ROI>::iterator it_R = rois_R->begin();

  while(it_R != rois_R->end()) {
    if (it_R->start.y > cam_height_/2) {
      it_R = rois_R->erase(it_R);
    }
    else ++it_R;
  }

  rois_R->sort(sort_rois_by_area_);

  unsigned int i = 0;
  for (it_R = rois_R->begin(); it_R != rois_R->end() && i++ < TRACKED_SIGNALS; ++it_R) {
    bool found_some_black = false;
    try {

      ROI *roi_R = new ROI(*it_R);

      long int start_x, start_y, width, height;

      start_y = (long int)it_R->start.y - (long int)it_R->width/3;
      if (start_y < 0) start_y = 0;
      height = it_R->width/2;
      if (start_y + height > cam_height_) height = cam_height_ - start_y;
      ROI check_black_top(*it_R);
      check_black_top.set_start(it_R->start.x, start_y);
      check_black_top.set_height(height);

      if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
        drawn_rois_.push_back(check_black_top);

      black_scangrid_->set_roi(&check_black_top);
      std::list<ROI> *black_rois_top = black_classifier_->classify();

      if (!black_rois_top->empty()) {
        found_some_black = true;
        ROI black_top = *(black_rois_top->begin());
        if (unlikely(cfg_tuning_mode_)) {
          if (cfg_draw_processed_rois_) {
            drawn_rois_.push_back(black_top);
            delete black_rois_top;
          }
          else drawn_rois_.insert(drawn_rois_.end(), black_rois_top->begin(), black_rois_top->end());
        }

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
      width = roi_Y->width * 1.2;
      if (start_x + width > cam_width_) width = cam_width_ - start_x;
      start_y = roi_Y->start.y + roi_Y->height/2;
      height = roi_Y->height * 2.5;
      if (start_y + height > cam_height_) height = cam_height_ - start_y;
      check_black_bottom.set_start(start_x, start_y);
      check_black_bottom.set_width(width);
      check_black_bottom.set_height(height);
      check_black_bottom.color = C_BACKGROUND;

      if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_))
        drawn_rois_.push_back(check_black_bottom);

      black_scangrid_->set_roi(&check_black_bottom);
      std::list<ROI> *black_rois_bottom = black_classifier_->classify();

      if (!black_rois_bottom->empty()) {
        found_some_black = true;
        black_rois_bottom->sort(sort_rois_by_y_);
        ROI black_bottom = *(black_rois_bottom->begin());
        if (unlikely(cfg_tuning_mode_)) {
          if (cfg_draw_processed_rois_) {
            drawn_rois_.push_back(black_bottom);
            delete black_rois_bottom;
          }
          else drawn_rois_.insert(drawn_rois_.end(), black_rois_bottom->begin(), black_rois_bottom->end());
        }

        unsigned int height_adj = (black_bottom.start.y - roi_R->start.y) / 3;
        roi_R->height = height_adj;
        roi_Y->height = height_adj;
        roi_G->height = height_adj;
        roi_Y->start.y = roi_R->start.y + roi_R->height;
        roi_G->start.y = roi_Y->start.y + roi_Y->height;
      }

      /*
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

      if (((float)black_in_green_area / (float)green_area) < 0.4) { //*/
      if (found_some_black)
        rv->push_back({roi_R, roi_Y, roi_G});
      //}
    }
    catch (OutOfBoundsException &e) {
      logger->log_error(name(), "%s", e.what());
    }
  }

  cfy_ctxt_green_.scanline_grid->set_roi(it_R->full_image(cam_width_, cam_height_));
  cfy_ctxt_green_.scanline_grid->reset();

  return rv;
}

bool MachineSignalPipelineThread::rois_delivery_zone(ROI &red, ROI &green) {
  return (green.contains(red.start.x, red.start.y)
            && green.contains(red.start.x + red.get_width(),
                red.start.y + red.get_height()))
            || green.width > cam_width_/4;
}

bool MachineSignalPipelineThread::roi_width_ok(ROI &r)
{ return r.width < cam_width_/4; }

bool MachineSignalPipelineThread::rois_similar_width(ROI &r1, ROI &r2) {
  float width_ratio = (float)r1.width / (float)r2.width;
  return width_ratio >= 1/cfg_roi_max_width_ratio_ && width_ratio <= cfg_roi_max_width_ratio_;
}

bool MachineSignalPipelineThread::rois_x_aligned(ROI &r1,ROI &r2) {
  float avg_width = (r1.width + r2.width) / 2.0f;
  int mid1 = r1.start.x + r1.width/2;
  int mid2 = r2.start.x + r2.width/2;
  return abs(mid1 - mid2) / avg_width < cfg_roi_xalign_;
}

bool MachineSignalPipelineThread::roi_aspect_ok(ROI &r) {
  float aspect_ratio = (float)r.width / (float)r.height;
  return aspect_ratio > 1/cfg_roi_max_aspect_ratio_ && aspect_ratio < cfg_roi_max_aspect_ratio_;
}

bool MachineSignalPipelineThread::rois_vspace_ok(ROI &r1, ROI &r2) {
  float avg_height = (r1.height + r2.height) / 2.0f;
  int dist = r2.start.y - (r1.start.y + r1.height);
  return dist > 0.6 * avg_height && dist < 1.7 * avg_height;
}


void MachineSignalPipelineThread::config_value_erased(const char *path) {}
void MachineSignalPipelineThread::config_tag_changed(const char *new_tag) {}
void MachineSignalPipelineThread::config_comment_changed(const Configuration::ValueIterator *v) {}

void MachineSignalPipelineThread::config_value_changed(const Configuration::ValueIterator *v)
{
  if (v->valid()) {
    std::string path = v->path();
    std::string sufx = path.substr(strlen(CFG_PREFIX));
    std::string color_pfx = sufx.substr(0, sufx.substr(1).find("/")+1);
    std::string full_pfx = CFG_PREFIX + color_pfx;
    std::string opt = path.substr(full_pfx.length());

    MutexLocker lock(&cfg_mutex_);
    bool chg = false;

    if (color_pfx == "/red" || color_pfx == "/green" || color_pfx == "/red_delivery") {
      color_classifier_context_t_ *classifier = NULL;
      if (color_pfx == "/red")
        classifier = &cfy_ctxt_red_;
      else if (color_pfx == "/green")
        classifier = &cfy_ctxt_green_;
      else if (color_pfx == "/red_delivery")
        classifier = &cfy_ctxt_red_delivery_;

      std::string opt = path.substr(full_pfx.length());

      if (opt == "/reference_color")
        chg = test_set_cfg_value(&(classifier->cfg_ref_col), v->get_uints());
      else if (opt == "/saturation_thresh")
        chg = test_set_cfg_value(&(classifier->cfg_sat_thresh), v->get_ints());
      else if (opt == "/chroma_thresh")
        chg = test_set_cfg_value(&(classifier->cfg_chroma_thresh), v->get_ints());
      else if (opt == "/luma_thresh")
        chg = test_set_cfg_value(&(classifier->cfg_luma_thresh), v->get_ints());
      else if (opt == "/min_points")
        chg = test_set_cfg_value(&(classifier->cfg_roi_min_points), v->get_uint());
      else if (opt == "/basic_roi_size")
        chg = test_set_cfg_value(&(classifier->cfg_roi_basic_size), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        chg = test_set_cfg_value(&(classifier->cfg_roi_neighborhood_min_match), v->get_uint());
      else if (opt == "/scangrid_x_offset")
        chg = test_set_cfg_value(&(classifier->cfg_scangrid_x_offset), v->get_uint());
      else if (opt == "/scangrid_y_offset")
        chg = test_set_cfg_value(&(classifier->cfg_scangrid_y_offset), v->get_uint());
    }
    else if (color_pfx == "/bright_light") {
      if (opt == "/min_brightness")
        chg = test_set_cfg_value(&(cfg_light_on_threshold_), v->get_uint());
      else if (opt == "/min_points")
        chg = test_set_cfg_value(&(cfg_light_on_min_points_), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        chg = test_set_cfg_value(&(cfg_light_on_min_neighborhood_), v->get_uint());
      else if (opt == "/min_area_cover")
        cfg_light_on_min_area_cover_ = v->get_float();
    }
    else if (color_pfx == "/black") {
      if (opt == "/max_luminance")
        chg = test_set_cfg_value(&(cfg_black_threshold_), v->get_uint());
      else if (opt == "/min_points")
        chg = test_set_cfg_value(&(cfg_black_min_points_), v->get_uint());
      else if (opt == "/neighborhood_min_match")
        chg = test_set_cfg_value(&(cfg_black_min_neighborhood_), v->get_uint());
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
      else if (opt == "/roi_green_horizon")
        cfg_roi_green_horizon = v->get_uint();
      else if (opt == "/debug_blink")
        cfg_debug_blink_ = v->get_bool();
      else if (opt == "/debug_processing")
        cfg_debug_processing_ = v->get_bool();
    }
    cfg_changed_ = cfg_changed_ || chg;
  }
}

