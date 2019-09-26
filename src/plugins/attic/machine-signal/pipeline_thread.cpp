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

#include "custom_rois.h"

#include <aspect/logging.h>
#include <core/threading/mutex_locker.h>
#include <fvfilters/colorthreshold.h>
#include <fvutils/color/colorspaces.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

using namespace fawkes;
using namespace fawkes::tf;
using namespace std;
using namespace firevision;

MachineSignalPipelineThread::MachineSignalPipelineThread()
: Thread("MachineSignal", Thread::OPMODE_CONTINUOUS),
  VisionAspect(VisionAspect::CONTINUOUS),
  ConfigurationChangeHandler(CFG_PREFIX)
{
	new_data_ = false;

	cam_width_              = 0;
	cam_height_             = 0;
	cfg_light_on_threshold_ = 0;
	camera_                 = NULL;

	time_wait_         = NULL;
	cfg_fps_           = 0;
	desired_frametime_ = 0;

	cfy_ctxt_red_1_.colormodel      = NULL;
	cfy_ctxt_red_1_.classifier      = NULL;
	cfy_ctxt_red_1_.scanline_grid   = NULL;
	cfy_ctxt_red_1_.color_expect    = C_RED;
	cfy_ctxt_red_0_.colormodel      = NULL;
	cfy_ctxt_red_0_.classifier      = NULL;
	cfy_ctxt_red_0_.scanline_grid   = NULL;
	cfy_ctxt_red_0_.color_expect    = C_RED;
	cfy_ctxt_green_1_.colormodel    = NULL;
	cfy_ctxt_green_1_.classifier    = NULL;
	cfy_ctxt_green_1_.scanline_grid = NULL;
	cfy_ctxt_green_1_.color_expect  = C_GREEN;
	cfy_ctxt_green_0_.colormodel    = NULL;
	cfy_ctxt_green_0_.classifier    = NULL;
	cfy_ctxt_green_0_.scanline_grid = NULL;
	cfy_ctxt_green_0_.color_expect  = C_GREEN;

	cfg_changed_ = false;
	cam_changed_ = false;

	shmbuf_     = NULL;
	shmbuf_cam_ = NULL;

	light_classifier_              = NULL;
	light_colormodel_              = NULL;
	light_scangrid_                = NULL;
	cfg_light_on_min_neighborhood_ = 0;
	cfg_light_on_min_points_       = 0;

	black_classifier_           = NULL;
	black_colormodel_           = NULL;
	black_scangrid_             = NULL;
	cfg_black_min_neighborhood_ = 0;
	cfg_black_min_points_       = 0;
	cfg_black_y_thresh_         = 30;
	cfg_black_u_thresh_         = 30;
	cfg_black_v_thresh_         = 30;
	cfg_black_u_ref_            = 128;
	cfg_black_v_ref_            = 128;

	roi_drawer_               = NULL;
	color_filter_             = NULL;
	cfg_roi_max_aspect_ratio_ = 1.7;
	combined_colormodel_      = NULL;
	last_second_              = NULL;
	buflen_                   = 0;
	bb_enable_switch_         = NULL;
	cfg_enable_switch_        = false;

	pos2pixel_          = NULL;
	cfg_cam_angle_y_    = 0;
	cfg_cam_aperture_x_ = 0;
	cfg_cam_aperture_y_ = 0;

	bb_laser_clusters_[0] = NULL;
	bb_laser_clusters_[1] = NULL;
	bb_laser_clusters_[2] = NULL;

	cluster_rois_ = NULL;

	bb_signal_position_estimate_ = NULL;

	memset(bb_laser_lines_, NUM_LASER_LINES, sizeof(LaserLineInterface *));
}

MachineSignalPipelineThread::~MachineSignalPipelineThread()
{
}

bool
MachineSignalPipelineThread::color_data_consistent(color_classifier_context_t_ *color_data)
{
	bool rv = (color_data->cfg_ref_col.size() == (3 * color_data->cfg_chroma_thresh.size()))
	          && (color_data->cfg_ref_col.size() == (3 * color_data->cfg_sat_thresh.size()))
	          && (color_data->cfg_ref_col.size() == (3 * color_data->cfg_luma_thresh.size()));
	return rv;
}

void
MachineSignalPipelineThread::setup_color_classifier(color_classifier_context_t_ *color_data,
                                                    ROI *                        roi)
{
	delete color_data->classifier;
	delete color_data->colormodel;
	delete color_data->scanline_grid;

	for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it =
	       color_data->color_class.begin();
	     it != color_data->color_class.end();
	     it++) {
		delete *it;
	}
	color_data->color_class.clear();

	// Update the color class used by the combined color model for the tuning
	// filter
	std::vector<int>::iterator          it_sat    = color_data->cfg_sat_thresh.begin();
	std::vector<int>::iterator          it_luma   = color_data->cfg_luma_thresh.begin();
	std::vector<unsigned int>::iterator it_ref    = color_data->cfg_ref_col.begin();
	std::vector<int>::iterator          it_chroma = color_data->cfg_chroma_thresh.begin();
	while (it_chroma != color_data->cfg_chroma_thresh.end()) {
		std::vector<unsigned int>            ref_col(it_ref, it_ref + 3);
		ColorModelSimilarity::color_class_t *color_class = new ColorModelSimilarity::color_class_t(
		  color_data->color_expect, ref_col, *(it_chroma++), *(it_sat++), *(it_luma++));
		it_ref += 3;
		color_data->color_class.push_back(color_class);
	}

	color_data->colormodel = new ColorModelSimilarity();
	color_data->colormodel->add_colors(color_data->color_class);
	color_data->scanline_grid = new ScanlineGrid(cam_width_,
	                                             cam_height_,
	                                             color_data->cfg_scangrid_x_offset,
	                                             color_data->cfg_scangrid_y_offset,
	                                             roi);

	color_data->classifier = new SimpleColorClassifier(color_data->scanline_grid,
	                                                   color_data->colormodel,
	                                                   color_data->cfg_roi_min_points,
	                                                   color_data->cfg_roi_basic_size,
	                                                   false,
	                                                   color_data->cfg_roi_neighborhood_min_match,
	                                                   0,
	                                                   color_data->color_expect);
}

void
MachineSignalPipelineThread::init()
{
	// Configure camera
	cfg_camera_ = config->get_string(CFG_PREFIX "/camera");
	setup_camera();

	shmbuf_ = new SharedMemoryImageBuffer("machine-signal", YUV422_PLANAR, cam_width_, cam_height_);
	shmbuf_cam_ = new SharedMemoryImageBuffer("machine-cam", YUV422_PLANAR, cam_width_, cam_height_);
	roi_drawer_ = new FilterROIDraw(&drawn_rois_, FilterROIDraw::DASHED_HINT);

	// Configure RED ON classifier
	cfy_ctxt_red_1_.cfg_ref_col        = config->get_uints(CFG_PREFIX "/red_on/reference_color");
	cfy_ctxt_red_1_.cfg_chroma_thresh  = config->get_ints(CFG_PREFIX "/red_on/chroma_thresh");
	cfy_ctxt_red_1_.cfg_sat_thresh     = config->get_ints(CFG_PREFIX "/red_on/saturation_thresh");
	cfy_ctxt_red_1_.cfg_luma_thresh    = config->get_ints(CFG_PREFIX "/red_on/luma_thresh");
	cfy_ctxt_red_1_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/red_on/min_points");
	cfy_ctxt_red_1_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/red_on/basic_roi_size");
	cfy_ctxt_red_1_.cfg_roi_neighborhood_min_match =
	  config->get_int(CFG_PREFIX "/red_on/neighborhood_min_match");
	cfy_ctxt_red_1_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/red_on/scangrid_x_offset");
	cfy_ctxt_red_1_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/red_on/scangrid_y_offset");
	cfy_ctxt_red_1_.visualize             = config->get_bool(CFG_PREFIX "/red_on/visualize");

	setup_color_classifier(&cfy_ctxt_red_1_);

	// Configure RED OFF classifier
	cfy_ctxt_red_0_.cfg_ref_col        = config->get_uints(CFG_PREFIX "/red_off/reference_color");
	cfy_ctxt_red_0_.cfg_chroma_thresh  = config->get_ints(CFG_PREFIX "/red_off/chroma_thresh");
	cfy_ctxt_red_0_.cfg_sat_thresh     = config->get_ints(CFG_PREFIX "/red_off/saturation_thresh");
	cfy_ctxt_red_0_.cfg_luma_thresh    = config->get_ints(CFG_PREFIX "/red_off/luma_thresh");
	cfy_ctxt_red_0_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/red_off/min_points");
	cfy_ctxt_red_0_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/red_off/basic_roi_size");
	cfy_ctxt_red_0_.cfg_roi_neighborhood_min_match =
	  config->get_int(CFG_PREFIX "/red_off/neighborhood_min_match");
	cfy_ctxt_red_0_.cfg_scangrid_x_offset = config->get_int(CFG_PREFIX "/red_off/scangrid_x_offset");
	cfy_ctxt_red_0_.cfg_scangrid_y_offset = config->get_int(CFG_PREFIX "/red_off/scangrid_y_offset");
	cfy_ctxt_red_0_.visualize             = config->get_bool(CFG_PREFIX "/red_off/visualize");

	setup_color_classifier(&cfy_ctxt_red_0_);

	// Configure GREEN ON classifier
	cfy_ctxt_green_1_.cfg_ref_col        = config->get_uints(CFG_PREFIX "/green_on/reference_color");
	cfy_ctxt_green_1_.cfg_chroma_thresh  = config->get_ints(CFG_PREFIX "/green_on/chroma_thresh");
	cfy_ctxt_green_1_.cfg_sat_thresh     = config->get_ints(CFG_PREFIX "/green_on/saturation_thresh");
	cfy_ctxt_green_1_.cfg_luma_thresh    = config->get_ints(CFG_PREFIX "/green_on/luma_thresh");
	cfy_ctxt_green_1_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/green_on/min_points");
	cfy_ctxt_green_1_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/green_on/basic_roi_size");
	cfy_ctxt_green_1_.cfg_roi_neighborhood_min_match =
	  config->get_int(CFG_PREFIX "/green_on/neighborhood_min_match");
	cfy_ctxt_green_1_.cfg_scangrid_x_offset =
	  config->get_int(CFG_PREFIX "/green_on/scangrid_x_offset");
	cfy_ctxt_green_1_.cfg_scangrid_y_offset =
	  config->get_int(CFG_PREFIX "/green_on/scangrid_y_offset");
	cfy_ctxt_green_1_.visualize = config->get_bool(CFG_PREFIX "/green_on/visualize");

	setup_color_classifier(&cfy_ctxt_green_1_);

	// Configure GREEN OFF classifier
	cfy_ctxt_green_0_.cfg_ref_col       = config->get_uints(CFG_PREFIX "/green_off/reference_color");
	cfy_ctxt_green_0_.cfg_chroma_thresh = config->get_ints(CFG_PREFIX "/green_off/chroma_thresh");
	cfy_ctxt_green_0_.cfg_sat_thresh    = config->get_ints(CFG_PREFIX "/green_off/saturation_thresh");
	cfy_ctxt_green_0_.cfg_luma_thresh   = config->get_ints(CFG_PREFIX "/green_off/luma_thresh");
	cfy_ctxt_green_0_.cfg_roi_min_points = config->get_int(CFG_PREFIX "/green_off/min_points");
	cfy_ctxt_green_0_.cfg_roi_basic_size = config->get_int(CFG_PREFIX "/green_off/basic_roi_size");
	cfy_ctxt_green_0_.cfg_roi_neighborhood_min_match =
	  config->get_int(CFG_PREFIX "/green_off/neighborhood_min_match");
	cfy_ctxt_green_0_.cfg_scangrid_x_offset =
	  config->get_int(CFG_PREFIX "/green_off/scangrid_x_offset");
	cfy_ctxt_green_0_.cfg_scangrid_y_offset =
	  config->get_int(CFG_PREFIX "/green_off/scangrid_y_offset");
	cfy_ctxt_green_0_.visualize = config->get_bool(CFG_PREFIX "/green_off/visualize");

	setup_color_classifier(&cfy_ctxt_green_0_);

	// Configure brightness classifier
	cfg_light_on_threshold_  = config->get_uint(CFG_PREFIX "/bright_light/min_brightness");
	cfg_light_on_min_points_ = config->get_uint(CFG_PREFIX "/bright_light/min_points");
	cfg_light_on_min_neighborhood_ =
	  config->get_uint(CFG_PREFIX "/bright_light/neighborhood_min_match");
	cfg_light_on_min_area_cover_ = config->get_float(CFG_PREFIX "/bright_light/min_area_cover");

	// Configure black classifier
	cfg_black_y_thresh_         = config->get_uint(CFG_PREFIX "/black/y_threshold");
	cfg_black_u_thresh_         = config->get_uint(CFG_PREFIX "/black/u_threshold");
	cfg_black_v_thresh_         = config->get_uint(CFG_PREFIX "/black/v_threshold");
	cfg_black_u_ref_            = config->get_uint(CFG_PREFIX "/black/u_reference");
	cfg_black_v_ref_            = config->get_uint(CFG_PREFIX "/black/v_reference");
	cfg_black_min_neighborhood_ = config->get_uint(CFG_PREFIX "/black/neighborhood_min_match");
	cfg_black_min_points_       = config->get_uint(CFG_PREFIX "/black/min_points");

	// General config
	cfg_roi_max_aspect_ratio_ = config->get_float(CFG_PREFIX "/roi_max_aspect_ratio");
	cfg_roi_max_width_ratio_  = config->get_float(CFG_PREFIX "/roi_max_r2g_width_ratio");
	cfg_roi_max_height_       = config->get_uint(CFG_PREFIX "/roi_max_height");
	cfg_roi_max_width_        = config->get_uint(CFG_PREFIX "/roi_max_width");
	cfg_roi_xalign_           = config->get_float(CFG_PREFIX "/roi_xalign_by_width");
	cfg_roi_green_horizon     = config->get_uint(CFG_PREFIX "/roi_green_horizon");
	cfg_tuning_mode_          = config->get_bool(CFG_PREFIX "/tuning_mode");
	cfg_max_jitter_           = config->get_float(CFG_PREFIX "/max_jitter");
	cfg_draw_processed_rois_  = config->get_bool(CFG_PREFIX "/draw_processed_rois");
	cfg_enable_switch_        = config->get_bool(CFG_PREFIX "/start_enabled");
	cfg_fps_                  = config->get_uint(CFG_PREFIX "/fps");
	cfg_debug_blink_          = config->get_bool(CFG_PREFIX "/debug_blink");
	cfg_debug_processing_     = config->get_bool(CFG_PREFIX "/debug_processing");
	cfg_debug_tf_             = config->get_bool(CFG_PREFIX "/debug_tf");

	cfg_lasercluster_frame_         = config->get_string(CFG_PREFIX "/laser_frame");
	cfg_lasercluster_signal_radius_ = 0.5 * config->get_float(CFG_PREFIX "/signal_width");
	cfg_lasercluster_signal_top_    = config->get_float(CFG_PREFIX "/signal_top");
	cfg_lasercluster_signal_bottom_ = config->get_float(CFG_PREFIX "/signal_bottom");
	cfg_cam_aperture_x_             = config->get_float(CFG_PREFIX "/cam_aperture_x");
	cfg_cam_aperture_y_             = config->get_float(CFG_PREFIX "/cam_aperture_y");
	cfg_cam_angle_y_                = config->get_float(CFG_PREFIX "/cam_angle");
	cfg_cam_frame_                  = config->get_string(CFG_PREFIX "/cam_frame");

	// Laser-lines config
	cfg_laser_lines_enabled_ = config->get_bool(CFG_PREFIX "/laser-lines/enable");
	cfg_laser_lines_min_vis_hist_ =
	  config->get_uint(CFG_PREFIX "/laser-lines/min_visibility_history");
	cfg_laser_lines_moving_avg_ = config->get_bool(CFG_PREFIX "/laser-lines/moving_avg");

	// Laser Cluster config
	cfg_lasercluster_enabled_ = config->get_bool(CFG_PREFIX "/lasercluster/enable");
	cfg_lasercluster_min_vis_hist_ =
	  config->get_uint(CFG_PREFIX "/lasercluster/min_visibility_history");

	pos2pixel_ = new PositionToPixel(tf_listener,
	                                 cfg_cam_frame_,
	                                 cfg_cam_aperture_x_,
	                                 cfg_cam_aperture_y_,
	                                 cam_width_,
	                                 cam_height_,
	                                 cfg_cam_angle_y_);

	// Setup combined ColorModel for tuning filter
	combined_colormodel_ = new ColorModelSimilarity();
	if (cfy_ctxt_red_1_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_red_1_.color_class);
	if (cfy_ctxt_red_0_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_red_0_.color_class);
	if (cfy_ctxt_green_1_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_green_1_.color_class);
	if (cfy_ctxt_green_0_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_green_0_.color_class);
	color_filter_ = new FilterColorThreshold(combined_colormodel_);

	// Setup luminance classifier for light on/off detection
	light_scangrid_   = new ScanlineGrid(cam_width_, cam_height_, 1, 1);
	light_colormodel_ = new ColorModelLuminance(cfg_light_on_threshold_);
	light_classifier_ = new SimpleColorClassifier(light_scangrid_,
	                                              light_colormodel_,
	                                              cfg_light_on_min_points_,
	                                              8,
	                                              false,
	                                              cfg_light_on_min_neighborhood_,
	                                              0,
	                                              C_WHITE);
	light_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

	// Setup black classifier for top/bottom recognition
	black_scangrid_   = new ScanlineGrid(cam_width_, cam_height_, 1, 1);
	black_colormodel_ = new ColorModelBlack(cfg_black_y_thresh_,
	                                        cfg_black_u_thresh_,
	                                        cfg_black_v_thresh_,
	                                        cfg_black_u_ref_,
	                                        cfg_black_v_ref_);
	black_classifier_ = new SimpleColorClassifier(black_scangrid_,
	                                              black_colormodel_,
	                                              cfg_black_min_points_,
	                                              6,
	                                              false,
	                                              cfg_black_min_neighborhood_,
	                                              0,
	                                              C_BLACK);
	black_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

	// Initialize frame rate detection & buffer lengths for blinking detection
	buflen_            = (unsigned int)ceil(cfg_fps_ / 4) + 1;
	desired_frametime_ = 1 / (double)cfg_fps_;
	time_wait_         = new TimeWait(clock, (long int)(1000000 * desired_frametime_));
	buflen_ += 1 - (buflen_ % 2); // make sure buflen is uneven
	logger->log_info(name(), "Buffer length: %d", buflen_);
	last_second_ = new Time(clock);

	bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>("/machine-signal");
	bb_enable_switch_->set_enabled(cfg_enable_switch_);
	bb_enable_switch_->write();

	bb_laser_clusters_[0] =
	  blackboard->open_for_reading<Position3DInterface>("/laser-cluster/ampel/1");
	bb_laser_clusters_[1] =
	  blackboard->open_for_reading<Position3DInterface>("/laser-cluster/ampel/2");
	bb_laser_clusters_[2] =
	  blackboard->open_for_reading<Position3DInterface>("/laser-cluster/ampel/3");

	bb_signal_position_estimate_ =
	  blackboard->open_for_writing<SignalHintInterface>("/machine-signal/position-hint");

	bb_open_laser_lines();

	config->add_change_handler(this);
}

void
MachineSignalPipelineThread::bb_open_laser_lines()
{
	string avg_suffix = "";
	if (cfg_laser_lines_moving_avg_)
		avg_suffix = "/moving_avg";
	for (uint i = 0; i < NUM_LASER_LINES; ++i) {
		bb_laser_lines_[i] = blackboard->open_for_reading_f<LaserLineInterface>("/laser-lines/%d%s",
		                                                                        i + 1,
		                                                                        avg_suffix.c_str());
	}
}

void
MachineSignalPipelineThread::bb_close_laser_lines()
{
	string avg_suffix = "";
	if (cfg_laser_lines_moving_avg_)
		avg_suffix = "/moving_avg";
	for (LaserLineInterface *bb_laser_line : bb_laser_lines_) {
		blackboard->close(bb_laser_line);
	}
}

void
MachineSignalPipelineThread::setup_camera()
{
	camera_     = vision_master->register_for_camera(cfg_camera_.c_str(), this);
	cam_width_  = camera_->pixel_width();
	cam_height_ = camera_->pixel_height();
}

void
MachineSignalPipelineThread::finalize()
{
	delete camera_;
	delete roi_drawer_;
	delete last_second_;

	delete cfy_ctxt_red_1_.colormodel;
	delete cfy_ctxt_red_1_.classifier;
	delete cfy_ctxt_red_1_.scanline_grid;
	for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it =
	       cfy_ctxt_red_1_.color_class.begin();
	     it != cfy_ctxt_red_1_.color_class.end();
	     it++) {
		delete *it;
	}
	delete cfy_ctxt_green_1_.colormodel;
	delete cfy_ctxt_green_1_.classifier;
	delete cfy_ctxt_green_1_.scanline_grid;
	for (std::vector<ColorModelSimilarity::color_class_t *>::iterator it =
	       cfy_ctxt_green_1_.color_class.begin();
	     it != cfy_ctxt_green_1_.color_class.end();
	     it++) {
		delete *it;
	}

	blackboard->close(bb_enable_switch_);
	for (Position3DInterface *iface : bb_laser_clusters_) {
		blackboard->close(iface);
	}
	blackboard->close(bb_signal_position_estimate_);
	bb_close_laser_lines();

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

bool
MachineSignalPipelineThread::lock_if_new_data()
{
	data_mutex_.lock();
	if (new_data_) {
		return true;
	} else {
		data_mutex_.unlock();
		return false;
	}
}

void
MachineSignalPipelineThread::unlock()
{
	data_mutex_.unlock();
}

std::list<SignalState> &
MachineSignalPipelineThread::get_known_signals()
{
	return known_signals_;
}

std::list<SignalState>::iterator
MachineSignalPipelineThread::get_best_signal()
{
	return best_signal_;
}

bool
MachineSignalPipelineThread::bb_switch_is_enabled(SwitchInterface *sw)
{
	bool rv = sw->is_enabled();
	while (!sw->msgq_empty()) {
		if (sw->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
			rv = false;
		} else if (sw->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
			rv = true;
		}

		sw->msgq_pop();
	}
	if (rv != sw->is_enabled()) {
		logger->log_info(name(), "*** enabled: %s", rv ? "yes" : "no");
		sw->set_enabled(rv);
		sw->write();
	}
	return rv;
}

#define LIMIT_VALUE(min, value, max) \
	std::min(std::max((long int)min, (long int)value), (long int)max)

firevision::WorldROI
MachineSignalPipelineThread::pos3d_to_roi(const Stamped<Point> &cluster_laser)
{
	// Transform laser cluster into camera frame
	Stamped<Point> cluster_cam;
	tf_listener->transform_point(cfg_cam_frame_, cluster_laser, cluster_cam);

	// Then compute the upper left and bottom right corner we expect to see
	// (Account for cam parallax when adding the signal radius)
	cart_coord_3d_t top_left, bottom_right;
	float           x      = cluster_cam.getX();
	float           y      = cluster_cam.getY();
	float           phi    = atan2f(y, x);
	float           edge_x = sinf(phi) * cfg_lasercluster_signal_radius_;
	float           edge_y = cosf(phi) * cfg_lasercluster_signal_radius_;

	top_left.x = cluster_cam.x() - edge_x;
	top_left.y = cluster_cam.y() + edge_y;
	top_left.z = cluster_cam.z() + cfg_lasercluster_signal_top_;

	bottom_right.x = cluster_cam.x() + edge_x;
	bottom_right.y = cluster_cam.y() - edge_y;
	bottom_right.z = cluster_cam.z() + cfg_lasercluster_signal_bottom_;

	if (unlikely(cfg_debug_tf_)) {
		logger->log_debug(name(),
		                  "Signal top_left: %f, %f, %f; bottom_right: %f, %f, %f.",
		                  top_left.x,
		                  top_left.y,
		                  top_left.z,
		                  bottom_right.x,
		                  bottom_right.y,
		                  bottom_right.z);
	}

	// And finally compute a ROI that (hopefully) contains the real signal
	point_t top_left_px_tmp =
	  pos2pixel_->get_pixel_position_unchecked(top_left, cfg_cam_frame_, clock);
	point_t bot_right_px_tmp =
	  pos2pixel_->get_pixel_position_unchecked(bottom_right, cfg_cam_frame_, clock);
	upoint_t top_left_px, bot_right_px;

	top_left_px.x  = min((long int)max(0, top_left_px_tmp.x), (long int)cam_width_);
	top_left_px.y  = min((long int)max(0, top_left_px_tmp.y), (long int)cam_height_);
	bot_right_px.x = min((long int)max(0, bot_right_px_tmp.x), (long int)cam_width_);
	bot_right_px.y = min((long int)max(0, bot_right_px_tmp.y), (long int)cam_height_);

	WorldROI cluster_roi;
	cluster_roi.set_start(top_left_px);
	cluster_roi.set_width(bot_right_px.x - top_left_px.x);
	cluster_roi.set_height(bot_right_px.y - top_left_px.y);
	cluster_roi.set_image_width(cam_width_);
	cluster_roi.set_image_height(cam_height_);
	cluster_roi.color     = C_MAGENTA;
	cluster_roi.world_pos = shared_ptr<Stamped<Point>>(
	  new Stamped<Point>(cluster_laser, cluster_laser.stamp, cluster_laser.frame_id));
	/* if (tf_listener->can_transform("/map", cluster_laser.frame_id,
  clock->now())) { cluster_roi.world_pos = new Stamped<Point>();
    tf_listener->transform_point("/map", cluster_laser,
  *(cluster_roi.world_pos));
  } */
	return cluster_roi;
}

set<firevision::WorldROI, SignalState::compare_rois_by_area> *
MachineSignalPipelineThread::bb_get_laser_rois()
{
	set<WorldROI, SignalState::compare_rois_by_area> *rv =
	  new set<WorldROI, SignalState::compare_rois_by_area>();
	if (unlikely(cfg_lasercluster_enabled_)) {
		for (Position3DInterface *bb_pos : bb_laser_clusters_) {
			bb_pos->read();
			if (bb_pos && bb_pos->has_writer()
			    && bb_pos->visibility_history() >= (long int)cfg_lasercluster_min_vis_hist_) {
				try {
					WorldROI laser_in_cam = pos3d_to_roi(Stamped<Point>(
					  Point(bb_pos->translation(0), bb_pos->translation(1), bb_pos->translation(2)),
					  Time(0, 0),
					  bb_pos->frame()));
					if (laser_in_cam.get_width() > 20 && laser_in_cam.get_height() > 40) {
						rv->insert(laser_in_cam);
					}
				} catch (OutOfBoundsException &e) {
					// This is a pretty normal case, getting a 3D position that is outside
					// the cam viewport
					// logger->log_debug(name(), e);
				} catch (Exception &e) {
					logger->log_error(name(), e);
				}
			}
		}
	}

	Point *signal_hint_msg = nullptr;
	while (!bb_signal_position_estimate_->msgq_empty()) {
		// Process new signal hint messages
		SignalHintInterface::SignalPositionMessage *msg = nullptr;
		if ((msg = bb_signal_position_estimate_->msgq_first_safe(msg))) {
			signal_hint_msg =
			  new btVector3(msg->translation(0), msg->translation(1), msg->translation(2));
		} else {
			logger->log_error(name(), "EEEK! Invalid message in SignalHintInterface! This is a Bug!");
		}
		bb_signal_position_estimate_->msgq_pop();
	}

	if (likely(cfg_laser_lines_enabled_)) {
		if (signal_hint_msg) {
			// Only update BB with sent MSGs if we're actually using them
			bb_signal_position_estimate_->set_translation(0, signal_hint_msg->getX());
			bb_signal_position_estimate_->set_translation(1, signal_hint_msg->getY());
			bb_signal_position_estimate_->set_translation(2, signal_hint_msg->getZ());
			bb_signal_position_estimate_->write();
			logger->log_info(name(),
			                 "Signal hint: %f, %f, %f",
			                 signal_hint_msg->getX(),
			                 signal_hint_msg->getY(),
			                 signal_hint_msg->getZ());
		}

		bb_signal_position_estimate_->read();
		Point current_signal_hint(bb_signal_position_estimate_->translation(0),
		                          bb_signal_position_estimate_->translation(1),
		                          bb_signal_position_estimate_->translation(2));

		LaserLineInterface *best_line = bb_laser_lines_[0];
		best_line->read();

		if (string(best_line->frame_id()) != "") {
			// Find the leftmost endpoint, as seen from base_link
			float *ep1 = best_line->end_point_1();
			float *ep2 = best_line->end_point_2();
			float *left_end;
			if (atan2f(ep2[1], ep2[0]) > atan2f(ep1[1], ep1[0]))
				left_end = ep2;
			else
				left_end = ep1;

			Stamped<Point> ep_laser = Stamped<Point>(Point(left_end[0], left_end[1], left_end[2]),
			                                         Time(0, 0),
			                                         best_line->frame_id());

			if (tf_listener->can_transform("/base_link", ep_laser.frame_id, ep_laser.stamp)) {
				Stamped<Point> ep_bl;
				tf_listener->transform_point("/base_link", ep_laser, ep_bl);
				// Table height is measured from the floor, i.e. in /base_link z=0.
				ep_bl.setZ(0);

				Stamped<Point> signal_hint_now(ep_bl + current_signal_hint, Time(0, 0), ep_bl.frame_id);
				if (best_line->visibility_history() >= (long int)cfg_laser_lines_min_vis_hist_
				    || (best_line->visibility_history() > 0
				        && signal_hint_now.distance(signal_hint_) < 0.01)) {
					// Either visibility history is good, or it is bad but the line has
					// moved by less than a centimeter.
					signal_hint_ = signal_hint_now;
					try {
						WorldROI signal_in_cam = pos3d_to_roi(signal_hint_now);
						if (signal_in_cam.get_width() > 20 && signal_in_cam.get_height() > 40) {
							rv->insert(signal_in_cam);
						}
					} catch (OutOfBoundsException &e) {
					} catch (Exception &e) {
						logger->log_error(name(), e.what());
					}
				}
			} else {
				logger->log_error(name(),
				                  "Missing transform from %s to /base_link!",
				                  best_line->frame_id());
			}
		}
	}
	delete signal_hint_msg;
	return rv;
}

inline void
MachineSignalPipelineThread::reinit_color_config()
{
	combined_colormodel_->delete_colors();

	setup_color_classifier(&cfy_ctxt_red_1_);
	setup_color_classifier(&cfy_ctxt_red_0_);
	setup_color_classifier(&cfy_ctxt_green_1_);
	setup_color_classifier(&cfy_ctxt_green_0_);

	if (cfy_ctxt_red_1_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_red_1_.color_class);
	if (cfy_ctxt_red_0_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_red_0_.color_class);
	if (cfy_ctxt_green_1_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_green_1_.color_class);
	if (cfy_ctxt_green_0_.visualize)
		combined_colormodel_->add_colors(cfy_ctxt_green_0_.color_class);

	delete light_classifier_;
	light_classifier_ = new SimpleColorClassifier(light_scangrid_,
	                                              new ColorModelLuminance(cfg_light_on_threshold_),
	                                              cfg_light_on_min_points_,
	                                              8,
	                                              false,
	                                              cfg_light_on_min_neighborhood_,
	                                              0,
	                                              C_WHITE);

	delete black_classifier_;
	delete black_colormodel_;
	black_colormodel_ = new ColorModelBlack(cfg_black_y_thresh_,
	                                        cfg_black_u_thresh_,
	                                        cfg_black_v_thresh_,
	                                        cfg_black_u_ref_,
	                                        cfg_black_v_ref_);
	black_classifier_ = new SimpleColorClassifier(black_scangrid_,
	                                              black_colormodel_,
	                                              cfg_black_min_points_,
	                                              6,
	                                              false,
	                                              cfg_black_min_neighborhood_,
	                                              0,
	                                              C_BLACK);

	delete pos2pixel_;
	pos2pixel_ = new PositionToPixel(tf_listener,
	                                 cfg_cam_frame_,
	                                 cfg_cam_aperture_x_,
	                                 cfg_cam_aperture_y_,
	                                 cam_width_,
	                                 cam_height_,
	                                 cfg_cam_angle_y_);
}

static inline float
symmetry(const ROI &r)
{
	return r.width <= r.height ? float(r.width) / float(r.height) : float(r.height) / float(r.width);
}

static inline float
similarity(const ROI &r1, const ROI &r2)
{
	ROI roi_union(std::min(r1.start.x, r2.start.x),
	              std::min(r1.start.y, r2.start.y),
	              std::max(r1.width, r2.width),
	              std::max(r1.height, r2.height),
	              r1.image_width,
	              r1.image_height);
	ROI roi_intersection = r1.intersect(r2);

	return float(roi_intersection.width * roi_intersection.height)
	       / float(roi_union.width * roi_union.height);
}

inline float
MachineSignalPipelineThread::compactness(const SignalState::signal_rois_t_ &s, const ROI &laser_roi)
{
	// unsigned int gap1 = std::abs(s.yellow_roi->start.y - (s.red_roi->start.y +
	// s.red_roi->height)); unsigned int gap2 = std::abs(s.green_roi->start.y -
	// (s.yellow_roi->start.y + s.yellow_roi->height));
	float h =
	  std::abs(static_cast<long>((s.green_roi->start.y + s.green_roi->height) - s.red_roi->start.y));
	// float gappitude = 1 - (float(gap1 + gap2) / h);
	float shortitude = 1 - (h / float(laser_roi.height));
	return shortitude;
	// return 1 - (h / float(laser_roi.height));
}

float
MachineSignalPipelineThread::signal_beauty(const SignalState::signal_rois_t_ &s,
                                           const ROI &                        laser_roi)
{
	ROI opt_R(*(s.red_roi));
	opt_R.start.x = laser_roi.start.x;
	opt_R.width   = laser_roi.width;
	opt_R.height  = laser_roi.width;

	ROI opt_G(*(s.green_roi));
	opt_G.start.x = laser_roi.start.x;
	opt_G.width   = laser_roi.width;
	opt_G.height  = laser_roi.width;

	float cmp = compactness(s, laser_roi);

	if (cfg_debug_processing_)
		debug_proc_string_ += " truth: " + to_string(s.truth) + ", compactness: " + to_string(cmp);

	return similarity(*(s.red_roi), opt_R) * similarity(*(s.green_roi), opt_G) * s.truth * cmp;
}

void
MachineSignalPipelineThread::loop()
{
	time_wait_->mark_start();

	Time   now(clock);
	double frametime = now - last_second_;
	last_second_     = &(last_second_->stamp());
	if (frametime >= desired_frametime_ * 1.1) {
		logger->log_warn(name(),
		                 "Running too slow (%f sec/frame). Blink detection will be unreliable!",
		                 frametime);
	}

	cfg_enable_switch_ = bb_switch_is_enabled(bb_enable_switch_);

	if (cfg_enable_switch_) {
		std::list<ROI> *rois_R_1, *rois_R_0, *rois_G_1, *rois_G_0;
		MutexLocker     lock(&cfg_mutex_);

		// Reallocate classifiers if their config changed
		if (unlikely(cfg_changed_ && color_data_consistent(&cfy_ctxt_red_1_)
		             && color_data_consistent(&cfy_ctxt_red_0_)
		             && color_data_consistent(&cfy_ctxt_green_1_)
		             && color_data_consistent(&cfy_ctxt_green_0_))) {
			reinit_color_config();
			bb_close_laser_lines();
			try {
				bb_open_laser_lines();
			} catch (Exception &e) {
				logger->log_error(name(), e.what());
				logger->log_error(name(),
				                  "Error opening smoothed LaserLineInterface. "
				                  "Retrying with moving_avg = false");
				cfg_laser_lines_moving_avg_ = false;
				bb_open_laser_lines();
			}
			cfg_changed_ = false;
		}

		camera_->capture();

		light_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
		black_classifier_->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

		cfy_ctxt_red_1_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
		cfy_ctxt_red_0_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
		cfy_ctxt_green_1_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);
		cfy_ctxt_green_0_.classifier->set_src_buffer(camera_->buffer(), cam_width_, cam_height_);

		drawn_rois_.clear();

		cluster_rois_ = bb_get_laser_rois();

		if (cluster_rois_ && cluster_rois_->size() == 1) {
			ROI r(*(cluster_rois_->begin()));
			cfy_ctxt_red_1_.scanline_grid->set_roi(&r);
			cfy_ctxt_red_0_.scanline_grid->set_roi(&r);
			cfy_ctxt_green_1_.scanline_grid->set_roi(&r);
			cfy_ctxt_green_0_.scanline_grid->set_roi(&r);
		} else {
			ROI top_half(0, 0, cam_width_, cam_height_ * 0.7, cam_width_, cam_height_);
			cfy_ctxt_red_1_.scanline_grid->set_roi(&top_half);
			cfy_ctxt_red_0_.scanline_grid->set_roi(&top_half);
			ROI bot_half(0, cam_height_ * 0.25, cam_width_, cam_height_ * 0.75, cam_width_, cam_height_);
			cfy_ctxt_green_1_.scanline_grid->set_roi(&bot_half);
			cfy_ctxt_green_0_.scanline_grid->set_roi(&bot_half);
		}

		rois_R_1 = cfy_ctxt_red_1_.classifier->classify();
		rois_R_0 = cfy_ctxt_red_0_.classifier->classify();
		rois_G_1 = cfy_ctxt_green_1_.classifier->classify();
		rois_G_0 = cfy_ctxt_green_0_.classifier->classify();

		if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_)) {
			if (cfy_ctxt_red_1_.visualize)
				drawn_rois_.insert(drawn_rois_.end(), rois_R_1->begin(), rois_R_1->end());
			if (cfy_ctxt_red_0_.visualize)
				drawn_rois_.insert(drawn_rois_.end(), rois_R_0->begin(), rois_R_0->end());
			if (cfy_ctxt_green_1_.visualize)
				drawn_rois_.insert(drawn_rois_.end(), rois_G_1->begin(), rois_G_1->end());
			if (cfy_ctxt_green_0_.visualize)
				drawn_rois_.insert(drawn_rois_.end(), rois_G_0->begin(), rois_G_0->end());
		}

		// Create and group ROIs that make up the red, yellow and green lights of a
		// signal
		if (cfg_debug_processing_)
			debug_proc_string_ = "";
		list<SignalState::signal_rois_t_> *signal_rois = new list<SignalState::signal_rois_t_>();

		for (const ROI &laser_roi : *cluster_rois_) {
			list<SignalState::signal_rois_t_> laser_signals;
			SignalState::signal_rois_t_ *     signal;

			signal = create_laser_signals(laser_roi, rois_R_0, rois_G_0);
			if (signal)
				laser_signals.push_back(*signal);
			delete signal;

			signal = create_laser_signals(laser_roi, rois_R_0, rois_G_1);
			if (signal)
				laser_signals.push_back(*signal);
			delete signal;

			signal = create_laser_signals(laser_roi, rois_R_1, rois_G_0);
			if (signal)
				laser_signals.push_back(*signal);
			delete signal;

			signal = create_laser_signals(laser_roi, rois_R_1, rois_G_1);
			if (signal)
				laser_signals.push_back(*signal);
			delete signal;

			laser_signals.sort([&laser_roi, this](const SignalState::signal_rois_t_ &a,
			                                      const SignalState::signal_rois_t_ &b) -> bool {
				return this->signal_beauty(a, laser_roi) >= this->signal_beauty(b, laser_roi);
			});
			if (!laser_signals.empty())
				signal_rois->push_back(laser_signals.front());
		}

		if (signal_rois->size() == 0) {
			// No signals found with laser, try vanilla algorithm
			delete signal_rois;
			list<ROI> *rois_R = new list<ROI>();
			rois_R->insert(rois_R->end(), rois_R_1->begin(), rois_R_1->end());
			rois_R->insert(rois_R->end(), rois_R_0->begin(), rois_R_0->end());
			list<ROI> *rois_G = new list<ROI>();
			rois_G->insert(rois_G->end(), rois_G_1->begin(), rois_G_1->end());
			rois_G->insert(rois_G->end(), rois_G_0->begin(), rois_G_0->end());
			signal_rois = create_field_signals(rois_R, rois_G);
		}

		// Lock mutex since now we start updating the data that is synced
		// to the blackboard by the MachineSignalSensorThread.
		data_mutex_.lock();

		for (SignalState &known_signal : known_signals_) {
			// Decrease visibility history if signal hasn't been seen for
			// a while or it's not inside a laser ROI anymore
			known_signal.inc_unseen(*cluster_rois_);
		}

		// Go through all signals from this frame...
		{
			std::list<SignalState::signal_rois_t_>::iterator signal_it = signal_rois->begin();
			for (uint i = 0; i < MAX_SIGNALS && signal_it != signal_rois->end(); ++i) {
				try {
					// ... and match them to known signals based on the max_jitter tunable
					float                            dist_min   = FLT_MAX;
					std::list<SignalState>::iterator best_match = known_signals_.begin();
					for (std::list<SignalState>::iterator known_signal = known_signals_.begin();
					     known_signal != known_signals_.end();
					     ++known_signal) {
						float dist = known_signal->distance(signal_it);
						if (dist < dist_min) {
							best_match = known_signal;
							dist_min   = dist;
						}
					}

					if (dist_min < cfg_max_jitter_) {
						best_match->update_geometry(signal_it);
					} else { // No historic match was found for the current signal
						SignalState::historic_signal_rois_t_ new_signal;
						logger->log_debug(name(), "new signal: dist=%f, history_len=%d", dist_min, 0);
						new_signal.red_roi =
						  shared_ptr<HistoricSmoothROI>(new HistoricSmoothROI(*(signal_it->red_roi), 0));
						new_signal.yellow_roi =
						  shared_ptr<HistoricSmoothROI>(new HistoricSmoothROI(*(signal_it->yellow_roi), 0));
						new_signal.green_roi =
						  shared_ptr<HistoricSmoothROI>(new HistoricSmoothROI(*(signal_it->green_roi), 0));
						SignalState *cur_state = new SignalState(buflen_, logger, new_signal);

						known_signals_.push_front(*cur_state);
						best_match = known_signals_.begin();
						delete cur_state;
					}

					SignalState::frame_state_t_ frame_state({
					  get_light_state(best_match->signal_rois_history_.red_roi.get()),
					  get_light_state(best_match->signal_rois_history_.yellow_roi.get()),
					  get_light_state(best_match->signal_rois_history_.green_roi.get()),
					});

					best_match->update_state(frame_state);

					// Classifiers sometimes do strange things to the image width/height,
					// so reset them here...
					best_match->signal_rois_history_.red_roi->set_image_width(cam_width_);
					best_match->signal_rois_history_.red_roi->set_image_height(cam_height_);
					best_match->signal_rois_history_.yellow_roi->set_image_width(cam_width_);
					best_match->signal_rois_history_.yellow_roi->set_image_height(cam_height_);
					best_match->signal_rois_history_.green_roi->set_image_width(cam_width_);
					best_match->signal_rois_history_.green_roi->set_image_height(cam_height_);

					if (unlikely(cfg_tuning_mode_ && cfg_draw_processed_rois_)) {
						drawn_rois_.push_back(*(best_match->signal_rois_history_.red_roi));
						drawn_rois_.push_back(*(best_match->signal_rois_history_.yellow_roi));
						drawn_rois_.push_back(*(best_match->signal_rois_history_.green_roi));
					}
				} catch (OutOfBoundsException &e) {
					// logger->log_debug(name(), "Signal at %d,%d: Invalid ROI: %s",
					//  signal_it->red_roi->start.x, signal_it->red_roi->start.y,
					//  e.what());
				}
				signal_it++;
			}
		}

		delete signal_rois;
		signal_rois = NULL;

		if (unlikely(cfg_tuning_mode_)) { // Draw a representation of what we see
			                                // into SHM buffer(s)
			shmbuf_cam_->lock_for_write();
			shmbuf_->lock_for_write();

			// Untreated copy of the cam image
			memcpy(shmbuf_cam_->buffer(), camera_->buffer(), shmbuf_cam_->data_size());

			// Visualize color similarities in tuning buffer
			color_filter_->set_src_buffer(camera_->buffer(), ROI::full_image(cam_width_, cam_height_));
			color_filter_->set_dst_buffer(shmbuf_->buffer(),
			                              ROI::full_image(shmbuf_->width(), shmbuf_->height()));
			color_filter_->apply();

			if (cluster_rois_) {
				drawn_rois_.insert(drawn_rois_.end(), cluster_rois_->begin(), cluster_rois_->end());
			}

			// Visualize the signals and bright spots we found
			roi_drawer_->set_rois(&drawn_rois_);
			roi_drawer_->set_src_buffer(shmbuf_->buffer(), ROI::full_image(cam_width_, cam_height_), 0);
			roi_drawer_->set_dst_buffer(shmbuf_->buffer(), NULL);
			roi_drawer_->apply();

			shmbuf_cam_->unlock();
			shmbuf_->unlock();
		}

		// Throw out the signals with the worst visibility histories
		known_signals_.sort(SignalState::compare_signal_states_by_visibility());
		while (known_signals_.size() > MAX_SIGNALS)
			known_signals_.pop_back();

		// Then sort geometrically
		known_signals_.sort(SignalState::compare_signal_states_by_area());
		best_signal_ = known_signals_.begin();

		if (unlikely(best_signal_ != known_signals_.end() && cfg_debug_blink_)) {
			logger->log_info(name(), best_signal_->get_debug_R());
			logger->log_info(name(), best_signal_->get_debug_Y());
			logger->log_info(name(), best_signal_->get_debug_G());
			logger->log_info(name(), "================="); //*/
		}
		new_data_ = true;
		data_mutex_.unlock();

		delete rois_R_1;
		delete rois_R_0;
		delete rois_G_1;
		delete rois_G_0;

		camera_->dispose_buffer();
	} /* if (cfg_enable_switch_) */
	else {
		known_signals_.clear();
	}
	time_wait_->wait();
}

/**
 * Determine if a given color ROI can be considered lit according to the
 * luminance model.
 * @param light the ROI to be considered
 * @return true if "lit" according to our criteria, false otherwise.
 */
bool
MachineSignalPipelineThread::get_light_state(firevision::ROI *light)
{
	light_scangrid_->set_roi(light);
	light_scangrid_->reset();
	std::list<ROI> *bright_rois = light_classifier_->classify();

	for (std::list<ROI>::iterator roi_it = bright_rois->begin(); roi_it != bright_rois->end();
	     ++roi_it) {
		float area_ratio =
		  (float)(roi_it->width * roi_it->height) / (float)(light->width * light->height);
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

bool
MachineSignalPipelineThread::roi1_x_overlaps_below(ROI &r1, ROI &r2)
{
	return (r1.start.y > r2.start.y)
	       && ((r1.start.x <= r2.start.x && r1.start.x + r1.width > r2.start.x)
	           || (r1.start.x > r2.start.x && r1.start.x < r2.start.x + r2.width
	               && r1.start.x + r1.width > r2.start.x + r2.width));
}

bool
MachineSignalPipelineThread::roi1_x_intersects(ROI &r1, ROI &r2)
{
	return !(r1.start.x + r1.width <= r2.start.x || r2.start.x + r2.width <= r1.start.x);
}

bool
MachineSignalPipelineThread::roi1_oversize(ROI &r1, ROI &r2)
{
	return !rois_similar_width(r1, r2) && roi_aspect_ok(r2)
	       && (r2.start.x + r2.width / 10) - r1.start.x > 0
	       && (r1.start.x + r1.width * 1.1) - (r2.start.x + r2.width) > 0;
}

/**
 * @param roi_R a red ROI
 * @param roi_G a green ROI
 * @return a yellow ROI that fits between roi_R and roi_G iff they match all
 * configured constraints
 */
ROI *
MachineSignalPipelineThread::red_green_match(ROI *r, ROI *g)
{
	if (!r || !g)
		return nullptr;

	shared_ptr<ROI> roi_R = make_shared<ROI>(*r);
	shared_ptr<ROI> roi_G = make_shared<ROI>(*g);

	int vspace = roi_G->start.y - (roi_R->start.y + roi_R->height);
	try {
		if (roi_G->height > cfg_roi_max_height_ && roi1_x_overlaps_below(*roi_G, *roi_R)) {
			ROI *recheck_G    = new ROI(*roi_G);
			recheck_G->height = cfg_roi_max_height_;
			cfy_ctxt_green_1_.scanline_grid->set_roi(recheck_G);
			list<ROI> *rechecked_G = cfy_ctxt_green_1_.classifier->classify();
			if (!rechecked_G->empty()) {
				ROI &new_G = rechecked_G->front();
				if (roi_width_ok(new_G) && rois_x_aligned(*roi_R, new_G) && !roi1_oversize(new_G, *roi_R)
				    && rois_vspace_ok(*roi_R, new_G) && rois_similar_width(*roi_R, new_G)) {
					*roi_G = new_G;
				}
			}
			cfy_ctxt_green_1_.scanline_grid->set_roi(recheck_G->full_image(cam_width_, cam_height_));
			delete recheck_G;
			delete rechecked_G;
		}

		if (roi_R->width > cfg_roi_max_width_ && roi_R->height > cfg_roi_max_height_
		    && roi_G->start.y > roi_R->start.y && roi1_x_intersects(*roi_G, *roi_R)) {
			ROI *recheck_R = new ROI(*roi_R);
			recheck_R->start.y += roi_R->height - cfg_roi_max_height_;
			recheck_R->height -= roi_R->height - cfg_roi_max_height_;
			cfy_ctxt_red_1_.scanline_grid->set_roi(recheck_R);
			list<ROI> *rechecked_R = cfy_ctxt_red_1_.classifier->classify();
			if (!rechecked_R->empty()) {
				ROI &new_R = rechecked_R->front();
				if (roi_width_ok(new_R) && rois_x_aligned(new_R, *roi_G) && !roi1_oversize(new_R, *roi_G)
				    && rois_vspace_ok(new_R, *roi_G) && rois_similar_width(new_R, *roi_G)) {
					*roi_R = new_R;
				}
			}
			cfy_ctxt_red_1_.scanline_grid->set_roi(recheck_R->full_image(cam_width_, cam_height_));
			delete recheck_R;
			delete rechecked_R;
		}

		if (roi_width_ok(*roi_G) && rois_x_aligned(*roi_R, *roi_G)
		    && roi_G->start.y > cfg_roi_green_horizon) {
			if (cfg_debug_processing_)
				debug_proc_string_ += " | g:W rg:A";

			if (roi1_oversize(*roi_R, *roi_G) && vspace > 0 && vspace < roi_G->height * 1.5) {
				roi_R->start.x = roi_G->start.x;
				roi_R->width   = roi_G->width;
				if (roi_R->width > cam_width_)
					roi_R->width = cam_width_ - roi_R->start.x;
				int r_end     = roi_G->start.y - roi_G->height;
				roi_R->height = r_end - roi_R->start.y;
				if (roi_R->start.y + roi_R->height > cam_height_)
					roi_R->height = cam_height_ - roi_R->start.y;

				if (cfg_debug_processing_)
					debug_proc_string_ += " r:O";
			}

			if (roi1_oversize(*roi_G, *roi_R) && vspace > 0 && vspace < roi_R->height * 1.5) {
				roi_G->start.x = roi_R->start.x;
				roi_G->width   = roi_R->width;
				if (roi_G->width > cam_width_)
					roi_G->width = cam_width_ - roi_G->start.x;
				roi_G->height = roi_R->height;
				if (roi_G->start.y + roi_G->height > cam_height_)
					roi_G->height = cam_height_ - roi_G->start.y;

				if (cfg_debug_processing_)
					debug_proc_string_ += " g:O";
			}

			if (rois_vspace_ok(*roi_R, *roi_G)) {
				if (cfg_debug_processing_)
					debug_proc_string_ += " V";

				if (!rois_similar_width(*roi_R, *roi_G)) {
					int wdiff   = roi_G->width - roi_R->width;
					int start_x = roi_G->start.x + wdiff / 2;
					if (start_x < 0)
						start_x = 0;
					roi_G->start.x = start_x;
					int width      = roi_G->width - wdiff / 2;
					if ((unsigned int)(start_x + width) > cam_width_)
						width = cam_width_ - start_x;
					roi_G->width = width;
					if (cfg_debug_processing_)
						debug_proc_string_ += " !W";
				}

				// Once we got through here it_G should have a pretty sensible green
				// ROI.
				roi_G->height = roi_G->width;
				uint start_x  = (roi_R->start.x + roi_G->start.x) / 2;
				uint height   = (roi_R->height + roi_G->height) / 2;
				uint width    = (roi_R->width + roi_G->width) / 2;
				uint r_end_y  = roi_R->start.y + roi_G->height;
				uint start_y  = r_end_y + (int)(roi_G->start.y - r_end_y - height) / 2;
				if (start_y < 0)
					start_y = 0;
				ROI *roi_Y =
				  new ROI(start_x, start_y, width, height, roi_R->image_width, roi_R->image_height);
				roi_Y->color = C_YELLOW;

				*r = *roi_R;
				*g = *roi_G;
				return roi_Y;
			}
		}
	} catch (Exception &e) {
		logger->log_error(name(), e.what());
	}
	return nullptr;
}

/**
 * Look for red and green ROIs that are likely to be part of a single signal. A
 * red and a green ROI are considered a match if the red one is above the green
 * one and there's space for a yellow one to fit in between. Unmatched ROIs and
 * those that violate certain (configurable) sanity criteria are thrown out.
 * @param rois_R A list of red ROIs
 * @param rois_G A list of green ROIs
 * @return A list of ROIs in a struct that contains matching red, yellow and
 * green ROIs
 */
std::list<SignalState::signal_rois_t_> *
MachineSignalPipelineThread::create_field_signals(std::list<ROI> *rois_R, std::list<ROI> *rois_G)
{
	std::list<SignalState::signal_rois_t_> *rv = new std::list<SignalState::signal_rois_t_>();

	/*if (unlikely(cfg_tuning_mode_ && !cfg_draw_processed_rois_)) {
    drawn_rois_.insert(drawn_rois_.end(), rois_R->begin(), rois_R->end());
    drawn_rois_.insert(drawn_rois_.end(), rois_G->begin(), rois_G->end());
  }*/

	std::list<ROI>::iterator it_R = rois_R->begin();
	uint                     i    = 0;
	while (it_R != rois_R->end()) {
		if (!(roi_width_ok(*it_R)) || (it_R->start.y > cam_height_ / 2)) {
			it_R = rois_R->erase(it_R);
			continue;
		}

		if (it_R->height > it_R->width) {
			it_R->height = it_R->width;
		}

		ROI *                    roi_Y = nullptr;
		std::list<ROI>::iterator it_G  = rois_G->begin();
		while (it_G != rois_G->end()) {
			if ((roi_Y = red_green_match(&*it_R, &*it_G))) {
				auto roi_G = shared_ptr<ROI>(new ROI(*it_G));
				auto roi_R = shared_ptr<ROI>(new ROI(*it_R));
				rv->push_back({roi_R, shared_ptr<ROI>(roi_Y), roi_G, nullptr, 1.0});
				it_G = rois_G->erase(it_G);
			} else {
				++it_G;
			}
		}

		if (roi_Y)
			it_R = rois_R->erase(it_R);
		else
			++it_R;

		if (unlikely(!roi_Y && cfg_debug_processing_))
			logger->log_debug(name(), "field proc red %d: %s", ++i, debug_proc_string_.c_str());
	}
	return rv;
}

ROI *
MachineSignalPipelineThread::merge_rois_in_roi(const ROI &outer_roi, list<ROI> *rois)
{
	ROI *               rv     = nullptr;
	list<ROI>::iterator it_roi = rois->begin();
	while (it_roi != rois->end()) {
		ROI          intersection = it_roi->intersect(outer_roi);
		unsigned int area_R       = it_roi->width * it_roi->height;
		unsigned int area_intrsct = intersection.width * intersection.height;
		if (area_R && (float(area_intrsct) / float(area_R) >= 0.3)) {
			if (rv != nullptr) {
				rv->extend(intersection.start.x, intersection.start.y);
				rv->extend(intersection.start.x + intersection.width,
				           intersection.start.y + intersection.height);
			} else {
				rv = new ROI(intersection);
			}

			// Erase ROIs that have been processed here
			it_roi = rois->erase(it_roi);
		} else {
			++it_roi;
		}
	}
	if (rv)
		rois->push_front(rv);
	return rv;
}

SignalState::signal_rois_t_ *
MachineSignalPipelineThread::create_laser_signals(const ROI &     laser_roi,
                                                  std::list<ROI> *rois_R,
                                                  std::list<ROI> *rois_G)
{
	SignalState::signal_rois_t_ *rv = nullptr;

	{ // Used to be a loop, kept block to simplify diffs
		try {
			shared_ptr<ROI> roi_R(merge_rois_in_roi(laser_roi, rois_R));
			shared_ptr<ROI> roi_G(merge_rois_in_roi(laser_roi, rois_G));

			// Look for the black cap on top
			ROI roi_black_top(laser_roi);
			roi_black_top.height = laser_roi.height / 4;
			roi_black_top.color  = C_BACKGROUND;
			if (roi_R) {
				roi_black_top.start.x = roi_R->start.x;
				roi_black_top.width   = roi_R->width;
			} else if (roi_G) {
				roi_black_top.start.x = roi_G->start.x;
				roi_black_top.width   = roi_G->width;
			}
			black_scangrid_->set_roi(&roi_black_top);
			list<ROI> *black_stuff_top = black_classifier_->classify();

			// Look for the black socket
			ROI roi_black_bottom(laser_roi);
			roi_black_bottom.color   = C_BACKGROUND;
			roi_black_bottom.start.y = laser_roi.start.y + laser_roi.height * 0.75;
			roi_black_bottom.height  = laser_roi.height / 4;
			if (roi_G) {
				roi_black_bottom.start.x = roi_G->start.x;
				roi_black_bottom.width   = roi_G->width;
			} else if (roi_R) {
				roi_black_bottom.start.x = roi_R->start.x;
				roi_black_bottom.width   = roi_R->width;
			}
			roi_black_bottom.set_image_height(cam_width_);
			roi_black_bottom.set_image_height(cam_height_);
			black_scangrid_->set_roi(&roi_black_bottom);
			list<ROI> *black_stuff_bottom = black_classifier_->classify();

			if (unlikely(cfg_tuning_mode_)) {
				drawn_rois_.insert(drawn_rois_.end(), black_stuff_top->begin(), black_stuff_top->end());
				drawn_rois_.insert(drawn_rois_.end(),
				                   black_stuff_bottom->begin(),
				                   black_stuff_bottom->end());
			}

			if (roi_R) {
				if (!black_stuff_top->empty()) {
					// Extend red ROI up to the black cap
					ROI &        black       = black_stuff_top->front();
					unsigned int black_end_y = black.start.y + black.height;
					int          hdiff       = roi_R->start.y - black_end_y;
					roi_R->start.y           = black_end_y;
					if (-hdiff > long(roi_R->height))
						hdiff = -roi_R->height;
					if (roi_R->height + hdiff > cam_height_)
						hdiff = cam_height_ - roi_R->height;
					roi_R->height += hdiff;
				}

				if (!roi_aspect_ok(*roi_R)) {
					roi_R->height = roi_R->width;
				}
			}
			///*
			if (roi_G) {
				// Improve green ROI with black socket
				if (!black_stuff_bottom->empty()) {
					ROI &black = black_stuff_bottom->front();
					int  hdiff = black.start.y - (roi_G->start.y + roi_G->height);
					if (hdiff > 0) {
						roi_G->height += hdiff;
					}
				}
			} //*/

			ROI *roi_Y = red_green_match(roi_R.get(), roi_G.get());

			if (roi_Y) {
				rv = new SignalState::signal_rois_t_(
				  {shared_ptr<ROI>(roi_R), shared_ptr<ROI>(roi_Y), shared_ptr<ROI>(roi_G), nullptr, 1.0});
			} else {
				SignalState::signal_rois_t_ signal;

				if (roi_R && roi_G) {
					float badness_R = abs(1 - (float(roi_R->height) / float(laser_roi.width)));
					float badness_G = abs(1 - (float(roi_G->height) / float(laser_roi.width)));
					if (cfg_debug_processing_) {
						drawn_rois_.push_back(*roi_R);
						drawn_rois_.push_back(*roi_G);
					}
					if (roi_G->height > laser_roi.width && badness_G > badness_R) {
						// Height of roi_R is closer to the width of the laser ROI than
						// roi_G
						uint hdiff = roi_G->height - roi_R->height;
						if (hdiff < roi_G->height) {
							roi_G->start.y += hdiff / 2;
							roi_G->height -= hdiff;
							if (cfg_debug_processing_)
								logger->log_info(name(), "laser adapt red: %d", hdiff);
						}
					} else if (roi_R->height > laser_roi.width && badness_R > badness_G) {
						uint hdiff = roi_R->height - roi_G->height;
						if (hdiff < roi_R->height) {
							roi_R->start.y += hdiff / 2;
							roi_R->height -= hdiff;
							if (cfg_debug_processing_)
								logger->log_info(name(), "laser adapt green: %d", hdiff);
						}
					}

					ROI *roi_Y = red_green_match(roi_R.get(), roi_G.get());
					if (roi_Y) {
						rv = new SignalState::signal_rois_t_({shared_ptr<ROI>(roi_R),
						                                      shared_ptr<ROI>(roi_Y),
						                                      shared_ptr<ROI>(roi_G),
						                                      nullptr,
						                                      1.0});
					} else {
						if (cfg_debug_processing_)
							logger->log_info(name(), "threw out r+g");
					}
				} else if (roi_R && !roi_G) {
					if (black_stuff_bottom->size() > 0) {
						roi_R->height = (black_stuff_bottom->front().start.y - roi_R->start.y) / 3;
					} else if (roi_R->start.y + roi_R->height * 3 > cam_height_) {
						roi_R->height = (cam_height_ - roi_R->start.y) / 3;
					}

					signal.red_roi = roi_R;

					// Put equally sized yellow & green ROIs below the red one
					signal.yellow_roi          = shared_ptr<ROI>(new ROI());
					signal.yellow_roi->color   = C_YELLOW;
					signal.yellow_roi->start.x = roi_R->start.x;
					signal.yellow_roi->start.y = roi_R->start.y + roi_R->height;
					signal.yellow_roi->width   = roi_R->width;
					signal.yellow_roi->height  = roi_R->height;

					signal.green_roi          = shared_ptr<ROI>(new ROI());
					signal.green_roi->color   = C_GREEN;
					signal.green_roi->start.x = signal.yellow_roi->start.x;
					signal.green_roi->start.y = signal.yellow_roi->start.y + signal.yellow_roi->height;
					signal.green_roi->width   = signal.yellow_roi->width;
					signal.green_roi->height  = signal.yellow_roi->height;

					signal.truth = 0.001;

					rv = new SignalState::signal_rois_t_(signal);
				} else if (!roi_R && roi_G) {
					if (black_stuff_top->size() > 0) {
						ROI &black    = black_stuff_top->front();
						roi_G->height = (roi_G->start.y + roi_G->height - (black.start.y + black.height)) / 3;
					} else if (long(roi_G->start.y) - long(roi_G->height) * 2 < 0) {
						unsigned int hnew = (roi_G->start.y + roi_G->height) / 3;
						roi_G->start.y += roi_G->height - hnew;
						roi_G->height = hnew;
					}

					signal.green_roi = roi_G;

					signal.yellow_roi          = shared_ptr<ROI>(new ROI());
					signal.yellow_roi->color   = C_YELLOW;
					signal.yellow_roi->start.x = roi_G->start.x;
					int start_y                = roi_G->start.y - roi_G->height;
					if (start_y < 0)
						start_y = 0;
					signal.yellow_roi->start.y = start_y;
					signal.yellow_roi->width   = roi_G->width;
					signal.yellow_roi->height  = roi_G->height;

					signal.red_roi          = shared_ptr<ROI>(new ROI());
					signal.red_roi->color   = C_RED;
					signal.red_roi->start.x = signal.yellow_roi->start.x;
					start_y                 = signal.yellow_roi->start.y - signal.yellow_roi->height;
					if (start_y < 0)
						start_y = 0;
					signal.red_roi->start.y = start_y;
					signal.red_roi->width   = signal.yellow_roi->width;
					signal.red_roi->height  = signal.yellow_roi->height;

					signal.truth = 0.001;

					rv = new SignalState::signal_rois_t_(signal);
				}
			}
			delete black_stuff_top;
			delete black_stuff_bottom;
		} catch (OutOfBoundsException &e) {
			logger->log_error(name(), e);
		}
	}
	if (unlikely(cfg_debug_processing_))
		logger->log_info(name(), "laser proc: %s", debug_proc_string_.c_str());
	return rv;
}

bool
MachineSignalPipelineThread::roi_width_ok(ROI &r)
{
	return r.width < cam_width_ / 3;
}

bool
MachineSignalPipelineThread::rois_similar_width(ROI &r1, ROI &r2)
{
	float width_ratio = (float)r1.width / (float)r2.width;
	return width_ratio >= 1 / cfg_roi_max_width_ratio_ && width_ratio <= cfg_roi_max_width_ratio_;
}

bool
MachineSignalPipelineThread::rois_x_aligned(ROI &r1, ROI &r2)
{
	float avg_width = (r1.width + r2.width) / 2.0f;
	int   mid1      = r1.start.x + r1.width / 2;
	int   mid2      = r2.start.x + r2.width / 2;
	return abs(mid1 - mid2) / avg_width < cfg_roi_xalign_;
}

bool
MachineSignalPipelineThread::roi_aspect_ok(ROI &r)
{
	float aspect_ratio = (float)r.width / (float)r.height;
	return aspect_ratio > 1 / cfg_roi_max_aspect_ratio_ && aspect_ratio < cfg_roi_max_aspect_ratio_;
}

bool
MachineSignalPipelineThread::rois_vspace_ok(ROI &r1, ROI &r2)
{
	float avg_height = (r1.height + r2.height) / 2.0f;
	int   dist       = r2.start.y - (r1.start.y + r1.height);
	return dist > 0.6 * avg_height && dist < 1.7 * avg_height;
}

void
MachineSignalPipelineThread::config_value_erased(const char *path)
{
}
void
MachineSignalPipelineThread::config_tag_changed(const char *new_tag)
{
}
void
MachineSignalPipelineThread::config_comment_changed(const Configuration::ValueIterator *v)
{
}

void
MachineSignalPipelineThread::config_value_changed(const Configuration::ValueIterator *v)
{
	if (v->valid()) {
		std::string path       = v->path();
		std::string sufx       = path.substr(strlen(CFG_PREFIX));
		std::string sub_prefix = sufx.substr(0, sufx.substr(1).find("/") + 1);
		std::string full_pfx   = CFG_PREFIX + sub_prefix;
		std::string opt        = path.substr(full_pfx.length());

		MutexLocker lock(&cfg_mutex_);
		bool        chg = false;

		if (sub_prefix == "/red_on" || sub_prefix == "/green_on" || sub_prefix == "/red_off"
		    || sub_prefix == "/green_off") {
			color_classifier_context_t_ *classifier = NULL;
			if (sub_prefix == "/red_on")
				classifier = &cfy_ctxt_red_1_;
			else if (sub_prefix == "/green_on")
				classifier = &cfy_ctxt_green_1_;
			else if (sub_prefix == "/red_off")
				classifier = &cfy_ctxt_red_0_;
			else if (sub_prefix == "/green_off")
				classifier = &cfy_ctxt_green_0_;

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
			else if (opt == "/visualize")
				chg = test_set_cfg_value(&(classifier->visualize), v->get_bool());
		} else if (sub_prefix == "/bright_light") {
			if (opt == "/min_brightness")
				chg = test_set_cfg_value(&(cfg_light_on_threshold_), v->get_uint());
			else if (opt == "/min_points")
				chg = test_set_cfg_value(&(cfg_light_on_min_points_), v->get_uint());
			else if (opt == "/neighborhood_min_match")
				chg = test_set_cfg_value(&(cfg_light_on_min_neighborhood_), v->get_uint());
			else if (opt == "/min_area_cover")
				cfg_light_on_min_area_cover_ = v->get_float();
		} else if (sub_prefix == "/black") {
			if (opt == "/y_threshold")
				chg = test_set_cfg_value(&(cfg_black_y_thresh_), v->get_uint());
			else if (opt == "/u_threshold")
				chg = test_set_cfg_value(&(cfg_black_u_thresh_), v->get_uint());
			else if (opt == "/v_threshold")
				chg = test_set_cfg_value(&(cfg_black_v_thresh_), v->get_uint());
			else if (opt == "/u_reference")
				chg = test_set_cfg_value(&(cfg_black_u_ref_), v->get_uint());
			else if (opt == "/v_reference")
				chg = test_set_cfg_value(&(cfg_black_v_ref_), v->get_uint());
			else if (opt == "/min_points")
				chg = test_set_cfg_value(&(cfg_black_min_points_), v->get_uint());
			else if (opt == "/neighborhood_min_match")
				chg = test_set_cfg_value(&(cfg_black_min_neighborhood_), v->get_uint());
		} else if (sub_prefix == "/lasercluster") {
			if (opt == "/min_visibility_history")
				cfg_lasercluster_min_vis_hist_ = v->get_uint();
			else if (opt == "/enable")
				cfg_lasercluster_enabled_ = v->get_bool();
		} else if (sub_prefix == "/laser-lines") {
			if (opt == "/min_visibility_history")
				cfg_laser_lines_min_vis_hist_ = v->get_uint();
			else if (opt == "/enable")
				cfg_laser_lines_enabled_ = v->get_bool();
			else if (opt == "/moving_avg")
				cfg_laser_lines_moving_avg_ = v->get_bool();
		} else if (sub_prefix == "") {
			if (opt == "/roi_max_aspect_ratio")
				cfg_roi_max_aspect_ratio_ = v->get_float();
			else if (opt == "/roi_max_width")
				cfg_roi_max_width_ = v->get_uint();
			else if (opt == "/roi_max_height")
				cfg_roi_max_height_ = v->get_uint();
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
			else if (opt == "/laser_frame")
				chg = test_set_cfg_value(&(cfg_lasercluster_frame_), v->get_string());
			else if (opt == "/cam_frame")
				chg = test_set_cfg_value(&(cfg_cam_frame_), v->get_string());
			else if (opt == "/cam_aperture_x")
				chg = test_set_cfg_value(&(cfg_cam_aperture_x_), v->get_float());
			else if (opt == "/cam_aperture_y")
				chg = test_set_cfg_value(&(cfg_cam_aperture_y_), v->get_float());
			else if (opt == "/cam_angle")
				chg = test_set_cfg_value(&(cfg_cam_angle_y_), v->get_float());
			else if (opt == "/signal_width")
				cfg_lasercluster_signal_radius_ = 0.5 * v->get_float();
			else if (opt == "/signal_top")
				cfg_lasercluster_signal_top_ = v->get_float();
			else if (opt == "/signal_bottom")
				cfg_lasercluster_signal_bottom_ = v->get_float();
			else if (opt == "/debug_tf")
				cfg_debug_tf_ = v->get_bool();
		}
		cfg_changed_ = cfg_changed_ || chg;
	}
}
