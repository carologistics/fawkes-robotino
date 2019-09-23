
/***************************************************************************
 *  pipeline_thread.h - Robotino AmpelVar Pipeline Thread
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

#include <fvcams/camera.h>
#include <fvmodels/global_position/omni_global.h>
#include <fvmodels/mirror/bulb.h>
#include <fvmodels/mirror/mirrormodel.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/color/conversions.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/ipc/shm_image.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <utils/math/angle.h> //diff angle
#include <utils/math/coord.h> //cart2polar2d
#include <utils/system/hostinfo.h>

#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <string>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** @class RobotinoAmpelVarPipelineThread "pipeline_thread.h"
 * Ampel indicator thread.
 *
 * @author Tim Niemueller (From ancient times and now again)
 * @author Daniel Beck (Mid-Size base)
 */

/** Constructor. */
RobotinoAmpelVarPipelineThread::RobotinoAmpelVarPipelineThread()
: Thread("RobotinoAmpelVarThread", Thread::OPMODE_WAITFORWAKEUP), VisionAspect(VisionAspect::CYCLIC)
{
	scanline_        = NULL;
	mirror_          = NULL;
	shm_buffer_      = NULL;
	ampel_red_if_    = NULL;
	ampel_orange_if_ = NULL;
	ampel_green_if_  = NULL;

	laser_pos_if_ = NULL;

	cspace_to_ = YUV422_PLANAR;

	drawer_ = 0;
}

/** Destructor. */
RobotinoAmpelVarPipelineThread::~RobotinoAmpelVarPipelineThread()
{
}

/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
RobotinoAmpelVarPipelineThread::init()
{
	cfg_prefix_        = "/plugins/ampelvar/";
	cfg_camera_        = config->get_string((cfg_prefix_ + "camera").c_str());
	cfg_camera_height_ = config->get_float((cfg_prefix_ + "camera_height").c_str());
	cfg_debug_buffer_  = config->get_bool((cfg_prefix_ + "debug_buffer").c_str());
	cfg_frame_         = config->get_string((cfg_prefix_ + "frame").c_str());
	cfg_mirror_file_ =
	  std::string(CONFDIR) + "/" + config->get_string((cfg_prefix_ + "mirror_file").c_str());

	cfg_red_height_    = config->get_float((cfg_prefix_ + "red_height").c_str());
	cfg_orange_height_ = config->get_float((cfg_prefix_ + "orange_height").c_str());
	cfg_green_height_  = config->get_float((cfg_prefix_ + "green_height").c_str());
	cfg_is_static_     = config->get_bool((cfg_prefix_ + "static").c_str());

	// camera
	cam_ = vision_master->register_for_camera(cfg_camera_.c_str(), this);

	// interfaces
	try {
		ampel_red_if_ = blackboard->open_for_writing<SwitchInterface>("ampel_red");
		ampel_red_if_->write();
		ampel_orange_if_ = blackboard->open_for_writing<SwitchInterface>("ampel_orange");
		ampel_orange_if_->write();
		ampel_green_if_ = blackboard->open_for_writing<SwitchInterface>("ampel_green");
		ampel_green_if_->write();

		laser_pos_if_ = blackboard->open_for_reading<Position3DInterface>("Machine_0");

	} catch (Exception &e) {
		delete cam_;
		cam_ = NULL;
		e.append("Opening ampel interfaces for writing failed");
		throw;
	}

	try {
		ampel_switch_if_ = blackboard->open_for_writing<SwitchInterface>("ampelswitch");
	} catch (Exception &e) {
		e.append("Opening ampelswitch interface for writing failed");
		throw;
	}

	// image properties
	img_width_   = cam_->pixel_width();
	img_height_  = cam_->pixel_height();
	cspace_from_ = cam_->colorspace(); // cspace from == cspace to, damit kein
	                                   // convert. sonst exception

	if (!cfg_debug_buffer_) {
		try {
			if (cspace_from_ != cspace_to_) {
				throw Exception("cspace_from_ does not match cspace_to_");
			}
		} catch (Exception &e) {
			throw;
		}
	}

	// other stuff that might throw an exception
	try {
		// mirror calibration
		mirror_ = new Bulb(cfg_mirror_file_.c_str(), "omni-mirror", true /* destroy on delete */);

		// SHM image buffer
		shm_buffer_ =
		  new SharedMemoryImageBuffer("ampelvar-processed", cspace_to_, img_width_, img_height_);
		if (!shm_buffer_->is_valid()) {
			throw Exception("Shared memory segment not valid");
		}

		shm_buffer_->set_frame_id(cfg_frame_.c_str());
		buffer_ = shm_buffer_->buffer();

	} catch (Exception &e) {
		delete mirror_;
		mirror_ = NULL;
		delete shm_buffer_;
		shm_buffer_ = NULL;
		blackboard->close(ampel_red_if_);
		ampel_red_if_ = NULL;
		blackboard->close(ampel_orange_if_);
		ampel_orange_if_ = NULL;
		blackboard->close(ampel_green_if_);
		ampel_green_if_ = NULL;

		blackboard->close(laser_pos_if_);
		laser_pos_if_ = NULL;

		delete cam_;
		cam_ = NULL;
		throw;
	}

	// scanline_ model
	scanline_ = new ScanlineGrid(img_width_, img_height_, 1, 1);

	// drawer
	if (cfg_debug_buffer_) {
		drawer_ = new Drawer();
		drawer_->set_buffer(buffer_, img_width_, img_height_);
		drawer_->set_color(76, 84, 255);
	}

	for (unsigned int i = 0; i <= 63; i++) {
		bucket[i] = 0;
	}

	center = mirror_->getCenter();
	/*center.x=158;
  center.y=121;*/

	luminance_threshold = 60;

	logger->log_debug(name(), "init=> center: x=%i, y=%i", center.x, center.y);

	/* static values for non-dynamic-approach */
	upper_left_red_x_man    = 53; // ima3, ima2: each 53
	upper_left_orange_x_man = 53;
	upper_left_green_x_man  = 53;

	upper_left_red_y_man    = 35; // ima3: 38, ima2: 35
	upper_left_orange_y_man = 56; // ima3: 57, ima2: 56
	upper_left_green_y_man  = 84; // ima3: 72, ima2: 84

	red_width_man    = 24;
	orange_width_man = 24;
	green_width_man  = 24;

	red_height_man    = 12; // ima3: each 10, ima2: each 12
	orange_height_man = 12;
	green_height_man  = 12;
}

/** Thread finalization. */
void
RobotinoAmpelVarPipelineThread::finalize()
{
	delete drawer_;
	delete scanline_;
	delete shm_buffer_;
	delete mirror_;

	try {
		ampel_red_if_->write();
		blackboard->close(ampel_red_if_);
		ampel_orange_if_->write();
		blackboard->close(ampel_orange_if_);
		ampel_green_if_->write();
		blackboard->close(ampel_green_if_);

		ampel_switch_if_->write();
		blackboard->close(ampel_switch_if_);

		blackboard->close(laser_pos_if_);

		delete ROI_colors[0];
		delete ROI_colors[1];
		delete ROI_colors[2];

	} catch (Exception &e) {
		e.append("Closing ampel interfaces failed");
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
RobotinoAmpelVarPipelineThread::loop()
{
	// check if there is a msg in the msg-queue
	while (!ampel_switch_if_->msgq_empty()) {
		if (SwitchInterface::DisableSwitchMessage *msg = ampel_switch_if_->msgq_first_safe(msg)) {
			logger->log_debug(name(), "Switch disable message received");
			ampel_switch_if_->set_enabled(false);
		} else if (SwitchInterface::EnableSwitchMessage *msg = ampel_switch_if_->msgq_first_safe(msg)) {
			logger->log_debug(name(), "Switch enable message received");
			ampel_switch_if_->set_enabled(true);
		}
		ampel_switch_if_->msgq_pop();
		ampel_switch_if_->write();
	}

	if (ampel_switch_if_->is_enabled() == false) {
		return;
	} //*/

	starttime = clock->now();
	laser_pos_if_->read();

	cam_->capture();

	buffer_ = cam_->buffer(); // dispose buffer dann am ende!

	if (!cfg_is_static_) {
		ampel_x = laser_pos_if_->translation(0);
		ampel_y = laser_pos_if_->translation(1);

		cart2polar2d(ampel_x, ampel_y, &theta, &distance_ampel);

		if (cfg_is_static_) {
			distance_ampel = 0.265f;
			theta          = 0.f;
		}

		// Berechnung des projezierten Abstandes der Ampelfarben auf der Nullebene
		distance_red =
		  cfg_camera_height_ * 100 * distance_ampel / (cfg_camera_height_ - cfg_red_height_);
		distance_orange =
		  cfg_camera_height_ * 100 * distance_ampel / (cfg_camera_height_ - cfg_orange_height_);
		distance_green =
		  cfg_camera_height_ * 100 * distance_ampel / (cfg_camera_height_ - cfg_green_height_);

		// logger->log_debug(name(),"red %f, orange %f, green
		// %f",distance_red,distance_orange,distance_green);

		// px position aus bulb! nearest neighbour
		// aus Laser: distance d und winkel theta

		d        = distance_red;
		gefunden = false;
		map      = mirror_->get_lut();

		for (h = 1; h < img_height_ / 2; ++h) {
			for (w = 1; w < img_width_ - 1; ++w) {
				pol_l   = map[h * img_width_ + w];
				pol_r   = map[h * img_width_ + w + 1];
				pol_l.r = pol_l.r * 100;

				if (pol_l.phi < -M_PI) {
					pol_l.phi = pol_l.phi + 2 * M_PI;
				}

				if (pol_l.phi < -M_PI) {
					pol_r.phi = pol_r.phi + 2 * M_PI;
				}

				if (pol_l.phi <= theta && pol_l.r <= d && pol_r.phi >= theta) {
					gefunden = true;
				}
				if (pol_l.phi >= theta && pol_l.r <= d && pol_r.phi <= theta) {
					gefunden = true;
				}
				if (gefunden)
					break;
			}
			if (gefunden)
				break;
		}

		px_position_red_x = w;
		px_position_red_y = h;

		d        = distance_orange;
		gefunden = false;
		map      = mirror_->get_lut();

		for (h = 1; h < img_height_ / 2; ++h) {
			for (w = 1; w < img_width_ - 1; ++w) {
				pol_l   = map[h * img_width_ + w];
				pol_r   = map[h * img_width_ + w + 1];
				pol_l.r = pol_l.r * 100;

				if (pol_l.phi < -M_PI) {
					pol_l.phi = pol_l.phi + 2 * M_PI;
				}

				if (pol_l.phi < -M_PI) {
					pol_r.phi = pol_r.phi + 2 * M_PI;
				}

				if (pol_l.phi <= theta && pol_l.r <= d && pol_r.phi >= theta) {
					gefunden = true;
				}
				if (pol_l.phi >= theta && pol_l.r <= d && pol_r.phi <= theta) {
					gefunden = true;
				}
				if (gefunden)
					break;
			}
			if (gefunden)
				break;
		}
		px_position_orange_x = w;
		px_position_orange_y = h;

		d        = distance_green;
		gefunden = false;
		map      = mirror_->get_lut();

		for (h = 1; h < img_height_ / 2; ++h) {
			for (w = 1; w < img_width_ - 1; ++w) {
				pol_l   = map[h * img_width_ + w];
				pol_r   = map[h * img_width_ + w + 1];
				pol_l.r = pol_l.r * 100;

				if (pol_l.phi < -M_PI) {
					pol_l.phi = pol_l.phi + 2 * M_PI;
				}

				if (pol_l.phi < -M_PI) {
					pol_r.phi = pol_r.phi + 2 * M_PI;
				}

				if (pol_l.phi <= theta && pol_l.r <= d && pol_r.phi >= theta) {
					gefunden = true;
				}
				if (pol_l.phi >= theta && pol_l.r <= d && pol_r.phi <= theta) {
					gefunden = true;
				}
				if (gefunden)
					break;
			}
			if (gefunden)
				break;
		}

		px_position_green_x = w;
		px_position_green_y = h;
	}

	luminance_threshold = 2;

	// set ROI parameter for each color
	if (!cfg_is_static_) {
		try {
			ROI_colors[0] = new ROI(px_position_red_x, px_position_red_y, 6, 2, img_width_, img_height_);
			ROI_colors[1] =
			  new ROI(px_position_orange_x, px_position_orange_y, 6, 2, img_width_, img_height_);
			ROI_colors[2] =
			  new ROI(px_position_green_x, px_position_green_y, 6, 2, img_width_, img_height_);
		} catch (Exception &e) {
			e.append("ROIs out of bound");
			throw;
		}
	}

	// static ROIs
	if (cfg_is_static_) {
		ROI_colors[0] = new ROI(upper_left_red_x_man,
		                        upper_left_red_y_man,
		                        red_width_man,
		                        red_height_man,
		                        img_width_,
		                        img_height_);
		ROI_colors[1] = new ROI(upper_left_orange_x_man,
		                        upper_left_orange_y_man,
		                        orange_width_man,
		                        orange_height_man,
		                        img_width_,
		                        img_height_);
		ROI_colors[2] = new ROI(upper_left_green_x_man,
		                        upper_left_green_y_man,
		                        green_width_man,
		                        green_height_man,
		                        img_width_,
		                        img_height_);
	}

	// detect red
	scanline_->reset();
	scanline_->set_roi(ROI_colors[0]);
	while (!scanline_->finished()) {
		current_x = (*scanline_)->x;
		current_y = (*scanline_)->y;
		luminance = buffer_[current_y * img_width_ + current_x];
		// logger->log_info(name(),"luminance red=%i",luminance);
		bucket[(unsigned int)(luminance / 4)]++;
		++(*scanline_);
	}

	bucket[0] = bucket[54] + bucket[55] + bucket[56] + bucket[57] + bucket[58] + bucket[59]
	            + bucket[60] + bucket[61] + bucket[62] + bucket[63];

	// logger->log_debug(name(),"bucket[0] red: %i",bucket[0]);
	if (bucket[0] >= luminance_threshold) {
		is_red = true;
	} else
		is_red = false;

	for (unsigned int i = 0; i <= 63; i++) {
		bucket[i] = 0;
	}

	// detect orange
	scanline_->reset();
	scanline_->set_roi(ROI_colors[1]);

	while (!scanline_->finished()) {
		current_x = (*scanline_)->x;
		current_y = (*scanline_)->y;
		luminance = buffer_[current_y * img_width_ + current_x];
		// logger->log_info(name(),"luminance orange=%i",luminance);
		bucket[(unsigned int)(luminance / 4)]++;
		++(*scanline_);
	}

	bucket[0] = bucket[54] + bucket[55] + bucket[56] + bucket[57] + bucket[58] + bucket[59]
	            + bucket[60] + bucket[61] + bucket[62] + bucket[63];

	// logger->log_debug(name(),"bucket[0] orange: %i",bucket[0]);
	if (bucket[0] >= luminance_threshold) {
		// drawer_->draw_circle(center.x,center.y-px_position_orange,3);
		is_orange = true;
	} else
		is_orange = false;

	for (unsigned int i = 0; i <= 63; i++) {
		bucket[i] = 0;
	}

	// detect green
	scanline_->reset();
	scanline_->set_roi(ROI_colors[2]);
	while (!scanline_->finished()) {
		current_x = (*scanline_)->x;
		current_y = (*scanline_)->y;
		luminance = buffer_[current_y * img_width_ + current_x];
		// logger->log_info(name(),"luminance green=%i",luminance);
		bucket[(unsigned int)(luminance / 4)]++;
		++(*scanline_);
	}

	bucket[0] = bucket[54] + bucket[55] + bucket[56] + bucket[57] + bucket[58] + bucket[59]
	            + bucket[60] + bucket[61] + bucket[62] + bucket[63];

	// logger->log_debug(name(),"bucket[0] green: %i",bucket[0]);

	if (bucket[0] >= luminance_threshold) {
		is_green = true;
	} else
		is_green = false;

	if (cfg_debug_buffer_) {
		// wenn der debug buffer geschrieben werden soll
		convert(
		  cspace_from_, cspace_to_, cam_->buffer(), shm_buffer_->buffer(), img_width_, img_height_);
		drawer_->draw_rectangle(upper_left_red_x_man,
		                        upper_left_red_y_man,
		                        red_width_man,
		                        red_height_man);
		drawer_->draw_rectangle(upper_left_orange_x_man,
		                        upper_left_orange_y_man,
		                        orange_width_man,
		                        orange_height_man);
		drawer_->draw_rectangle(upper_left_green_x_man,
		                        upper_left_green_y_man,
		                        green_width_man,
		                        green_height_man);
	}

	for (unsigned int i = 0; i <= 63; i++) {
		bucket[i] = 0;
	}

	/*
    scanline_model->reset();
          while (! scanline_model->finished()) {

            x = (*scanline_model)->x;
            y = (*scanline_model)->y; */

	// interface schreiben
	ampel_red_if_->set_enabled(is_red);
	ampel_orange_if_->set_enabled(is_orange);
	ampel_green_if_->set_enabled(is_green);

	ampel_red_if_->write();
	ampel_orange_if_->write();
	ampel_green_if_->write();

	delete ROI_colors[0];
	delete ROI_colors[1];
	delete ROI_colors[2];

	// logger->log_debug(name(),"result red: %i, orange: %i, green:
	// %i",ampel_red_if_->is_enabled(),ampel_orange_if_->is_enabled(),ampel_green_if_->is_enabled());
	// logger->log_debug(name(),"Zeit: %f",clock->elapsed(&starttime));

	cam_->dispose_buffer();
}

/*  float zwischen1 =
  -0.0034*distance_red*distance_red+0.9855*distance_red+12.218; float zwischen2
  =-0.0034*distance_orange*distance_orange+0.9855*distance_orange+12.218;

  logger->log_info(name(),"float red px distance: %f",zwischen1);
  logger->log_info(name(),"float orange px distance: %f",zwischen2);

  px_position_red=0.000009*distance_red*distance_red*distance_red-0.005*distance_red*distance_red+1.0462*distance_red+11.712;
  px_position_orange=0.000009*distance_orange*distance_orange*distance_orange-0.005*distance_orange*distance_orange+1.0462*distance_orange+11.712;
  px_position_green=0.000009*distance_green*distance_green*distance_green-0.005*distance_green*distance_green+1.0462*distance_green+11.712;*/
