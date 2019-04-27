/***************************************************************************
 *  pipeline_thread.h - Robotino OmniVision Pipeline Thread
 *
 *  Created: Thu May 24 17:17:46 2012
 *  Copyright  2005-2012  Tim Niemueller [www.niemueller.de]
 *             2012		  Sebastian Reuter
 *             2007       Daniel Beck
 *             2005       Martin Heracles
 *             2012-2013  Johannes Rothe
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
#include <fvclassifiers/simple.h>
#include <fvfilters/roidraw.h>
#include <fvmodels/color/lookuptable.h>
#include <fvmodels/global_position/omni_global.h>
#include <fvmodels/mirror/bulb.h>
#include <fvmodels/mirror/mirrormodel.h>
#include <fvmodels/relative_position/omni_relative.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/color/conversions.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/ipc/shm_image.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <utils/hungarian_method/hungarian.h>
#include <utils/misc/string_conversions.h>
#include <utils/system/hostinfo.h>

#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <string>
#include <unistd.h>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** @class OmniVisionPucksPipelineThread "pipeline_thread.h"
 * Puck detector thread.
 *
 * @author Tim Niemueller (From ancient times and now again)
 * @author Daniel Beck (Mid-Size base)
 * @author Johannes Rothe
 */

/** Amount of pucks */
#define PUCK_AMOUNT 20

/** Constructor. */
OmniVisionPucksPipelineThread::OmniVisionPucksPipelineThread()
: Thread("RobotinoOmniVisionThread", Thread::OPMODE_CONTINUOUS),
  VisionAspect(VisionAspect::CONTINUOUS)
{
	scanline_   = NULL;
	cm_         = NULL;
	mirror_     = NULL;
	rel_pos_    = NULL;
	classifier_ = NULL;
	shm_buffer_ = NULL;
	_new_data   = false;

	_data_mutex  = new Mutex();
	if_puck_map_ = new PuckIfMap();

	cspace_to_ = YUV422_PLANAR;
	drawer_    = 0;
}

/** Destructor. */
OmniVisionPucksPipelineThread::~OmniVisionPucksPipelineThread()
{
}

/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
OmniVisionPucksPipelineThread::init()
{
	cfg_prefix_      = "/hardware/robotino/omnivision/";
	cfg_camera_      = config->get_string((cfg_prefix_ + "camera").c_str());
	cfg_frame_       = config->get_string((cfg_prefix_ + "frame").c_str());
	cfg_cam_height_  = config->get_float((cfg_prefix_ + "camera_height").c_str());
	cfg_puck_radius_ = config->get_float((cfg_prefix_ + "puck_radius").c_str());
	cfg_mirror_file_ =
	  std::string(CONFDIR) + "/" + config->get_string((cfg_prefix_ + "mirror_file").c_str());
	cfg_colormap_file_ =
	  std::string(CONFDIR) + "/" + config->get_string((cfg_prefix_ + "colormap_file").c_str());
	cfg_neighbors      = config->get_float((cfg_prefix_ + "neighbors").c_str());
	cfg_basic_roi_size = config->get_float((cfg_prefix_ + "basic_roi_size").c_str());

	// camera
	cam_ = vision_master->register_for_camera(cfg_camera_.c_str(), this);

	// init puck interfaces

	try {
		int i;
		for (i = 1; i <= PUCK_AMOUNT; i++) {
			Position3DInterface *puckif;
			char *               omni_name;
			if (asprintf(&omni_name, "OmniPuck%d", i) != -1) {
				puckif = blackboard->open_for_writing<Position3DInterface>(omni_name);
				puckif->set_frame(cfg_frame_.c_str());
				puckif->set_visibility_history(-1);
				puckif->write();
				free(omni_name);
				puck_ifs_.push_back(puckif);
			}
		}
	} catch (Exception &e) {
		e.append("Opening puck interfaces for writing failed");
		throw;
	}

	try {
		switchInterface = blackboard->open_for_writing<SwitchInterface>("omnivisionSwitch");
	} catch (Exception &e) {
		e.append("Opening switch interface for writing failed");
		throw;
	}

	// image properties
	img_width_   = cam_->pixel_width();
	img_height_  = cam_->pixel_height();
	cspace_from_ = cam_->colorspace();

	// other stuff that might throw an exception
	try {
		// mirror calibration
		mirror_ = new Bulb(cfg_mirror_file_.c_str(), "omni-mirror", true /* destroy on delete */);

		// colormodel
		cm_ = new ColorModelLookupTable(cfg_colormap_file_.c_str(),
		                                "omni-colormap",
		                                true /* destroy on delete */);

		// SHM image buffer
		shm_buffer_ =
		  new SharedMemoryImageBuffer("omni-processed", cspace_to_, img_width_, img_height_);
		if (!shm_buffer_->is_valid()) {
			throw Exception("Shared memory segment not valid");
		}
		shm_buffer_->set_frame_id(cfg_frame_.c_str());
		buffer_ = shm_buffer_->buffer();

	} catch (Exception &e) {
		delete mirror_;
		mirror_ = NULL;
		delete cm_;
		cm_ = NULL;
		delete shm_buffer_;
		shm_buffer_ = NULL;
		delete cam_;
		cam_ = NULL;
		throw;
	}

	// scanline_ model
	scanline_ = new ScanlineGrid(img_width_, img_height_, 2, 2);

	// position model
	rel_pos_ = new OmniRelative(mirror_);

	// classifier
	classifier_ =
	  new SimpleColorClassifier(scanline_, cm_, 0, cfg_basic_roi_size, false, cfg_neighbors, 4);
	/** Constructor.
   * @param scanline_model scanline model
   * @param color_model color model
   * @param min_num_points minimum number of points in ROI to be considered
   * @param box_extent basic extent of a new ROI
   * @param upward set to true if you have an upward scanline model, this means
   * that points are traversed from the bottom to the top. In this case the ROIs
   * are initially extended towards the top instead of the bottom.
   * @param neighbourhood_min_match minimum number of object pixels to grow
   * neighbourhood
   * @param grow_by grow region by that many pixels
   * @param color color to look for
   */
	// drawer
	drawer_ = new Drawer();
	drawer_->set_buffer(buffer_, img_width_, img_height_);
	drawer_->set_color(0, 127, 127);
}

/** Thread finalization. */
void
OmniVisionPucksPipelineThread::finalize()
{
	delete drawer_;
	delete classifier_;
	delete rel_pos_;
	delete scanline_;
	delete shm_buffer_;
	delete cm_;
	delete mirror_;

	logger->log_debug(name(), "Unregistering from vision master");
	vision_master->unregister_thread(this);
	delete cam_;

	try {
		std::list<fawkes::Position3DInterface *>::iterator puck;
		puck = puck_ifs_.begin();
		for (puck = puck_ifs_.begin(); puck != puck_ifs_.end(); puck++) {
			(*puck)->set_visibility_history(0);
			blackboard->close(*puck);
		}

		blackboard->close(switchInterface);

	} catch (Exception &e) {
		logger->log_error(name(), "Closing interface failed!");
		logger->log_error(name(), e);
		throw;
	}
}

/** Process image to detect objects.
 * Retrieves a new image from the camera and uses the classifier
 * determine objects according to the colormap. Publish the
 * position of the closest such object as puck position.
 */
void
OmniVisionPucksPipelineThread::loop()
{
	old_pucks_ = current_pucks_;
	// check if there is a msg in the msg-queue
	while (!switchInterface->msgq_empty()) {
		if (SwitchInterface::DisableSwitchMessage *msg = switchInterface->msgq_first_safe(msg)) {
			logger->log_info(name(), "Switch disable message received");
			switchInterface->set_enabled(false);
		} else if (SwitchInterface::EnableSwitchMessage *msg = switchInterface->msgq_first_safe(msg)) {
			logger->log_info(name(), "Switch enable message received");
			switchInterface->set_enabled(true);
		}
		switchInterface->msgq_pop();
		switchInterface->write();
	}

	if (!switchInterface->is_enabled()) {
		std::list<fawkes::Position3DInterface *>::iterator puck;
		puck = puck_ifs_.begin();
		for (puck = puck_ifs_.begin(); puck != puck_ifs_.end(); puck++) {
			(*puck)->set_visibility_history(0);
			(*puck)->write();
		}
		usleep(500000);
		return;
	}

	cam_->capture();
	convert(cspace_from_, cspace_to_, cam_->buffer(), buffer_, img_width_, img_height_);
	puck_visible_ = false;

	// run classifier
	classifier_->set_src_buffer(cam_->buffer(), img_width_, img_height_);

	rois_ = classifier_->classify();
	cam_->dispose_buffer();

	FilterROIDraw *f = new FilterROIDraw();
	f->set_src_buffer(buffer_, ROI::full_image(img_width_, img_height_), 0);
	// post-process ROIs
	std::list<firevision::ROI>::iterator r;
	if (rois_->empty()) {
		logger->log_debug(name(), "keine ROIS gefunden");
		shm_buffer_->set_circle_found(false);
	} else {
		// if we have at least one ROI
		puck_visible_ = true;
		// get only the pucks within 1m radius to the robot
		min_dist_ = 1.f;
		// sort the ROIS for distance
		rois_->sort(sortFunctor(rel_pos_, classifier_));

		int roicounter = 0;
		// erase all ROIS with index greater than PUCK_AMOUNT
		for (r = rois_->begin(); r != rois_->end();) {
			classifier_->get_mass_point_of_color(&(*r), &mass_point_);
			rel_pos_->set_center(mass_point_.x, mass_point_.y);
			rel_pos_->calc_unfiltered();
			f->set_dst_buffer(buffer_, &(*r));
			f->set_style(FilterROIDraw::DASHED_HINT);
			f->apply();
			if (roicounter > PUCK_AMOUNT || rel_pos_->get_distance() > min_dist_) {
				r = rois_->erase(r);
				continue;
			}
			r++;
			roicounter++;
		}
	}
	// pucks has now same length or is longer than rois_
	std::list<Position3DInterface *>::iterator puckif_it;
	puckif_it = puck_ifs_.begin();
	_data_mutex->lock();
	current_pucks_.clear();
	relPositions_.clear();
	for (r = rois_->begin(); r != rois_->end(); r++) {
		if (puck_visible_) {
			classifier_->get_mass_point_of_color(&(*r), &mass_point_);
			rel_pos_->set_center(mass_point_.x, mass_point_.y);
			drawer_->draw_circle(mass_point_.x, mass_point_.y, 6);
			if (rel_pos_->is_pos_valid()) {
				rel_pos_->calc();

				Point3d puck_relative;
				puck_relative.setX(rel_pos_->get_x());
				puck_relative.setY(rel_pos_->get_y());
				Point3d puck_absolute = apply_tf_to_global(puck_relative);
				current_pucks_.push_back(puck_absolute);
				relPositions_[puck_absolute] = puck_relative;
				// logger->log_debug(name(),
				//		"transformation applied: (%f|%f)->(%f|%f)",
				//		puck_relative.getX(), puck_relative.getY(),
				//		current_pucks_.back().getX(),
				//		current_pucks_.back().getY());
			}
		}
	}
	associate_pucks_with_ifs();
	_new_data = true;
	_data_mutex->unlock();
}

void
OmniVisionPucksPipelineThread::associate_pucks_with_ifs()
{
	hungarian_problem_t cost_matrix;
	cost_matrix.num_rows = old_pucks_.size();
	cost_matrix.num_cols = current_pucks_.size();
	cost_matrix.cost     = (int **)calloc(cost_matrix.num_rows, sizeof(int *));
	for (int i = 0; i < cost_matrix.num_rows; ++i) {
		cost_matrix.cost[i] = (int *)calloc(cost_matrix.num_cols, sizeof(int *));
	}
	for (unsigned int index_old = 0; index_old < old_pucks_.size(); ++index_old) {
		for (unsigned int index_current = 0; index_current < current_pucks_.size(); ++index_current) {
			// hungarian method takes int, so we transform our floats to suitable ints
			// e.g.: 0.015 -> 15
			int dist_in_int = (int)old_pucks_[index_old].distance(current_pucks_[index_current]) * 1000;
			// take the square distance
			cost_matrix.cost[index_old][index_current] = dist_in_int * dist_in_int;
		}
	}
	HungarianMethod hSolver;
	hSolver.init(cost_matrix.cost,
	             cost_matrix.num_rows,
	             cost_matrix.num_cols,
	             HUNGARIAN_MODE_MINIMIZE_COST);
	hSolver.solve();

	int  size_assignment;
	int *assignment = hSolver.get_assignment(size_assignment);

	list<Position3DInterface *> unused_ifs(puck_ifs_);
	list<Point3d>               new_pucks;
	PuckIfMap *                 if_puck_map_current = new PuckIfMap();

	for (int i = 0; i < size_assignment; ++i) {
		if ((unsigned int)i >= old_pucks_.size()) {
			// new puck detected, no match to old
			new_pucks.push_back(current_pucks_[assignment[i]]);
		} else if ((unsigned int)assignment[i] >= current_pucks_.size()) {
			// puck has been lost
			continue;
		} else {
			Point3d &old_puck = old_pucks_[i];
			Point3d &new_puck = current_pucks_[assignment[i]];
			// logger->log_debug(name(), "Old: (%f|%f) matches New: (%f|%f)",
			//		old_puck.getX(), old_puck.getY(), new_puck.getX(),
			//		new_puck.getY());
			map<Point3d, Position3DInterface *>::iterator old_if = if_puck_map_->find(old_puck);
			if (old_if != if_puck_map_->end()) {
				// logger->log_debug(name(), "Gets old if %s",
				//		old_if->second->id());
				unused_ifs.remove(old_if->second);
				(*if_puck_map_current)[new_puck] = old_if->second;
				old_if->second->set_visibility_history(old_if->second->visibility_history() + 1);
			} else {
				new_pucks.push_back(new_puck);
			}
		}
	}
	// assign new pucks
	for (Point3d &new_puck : new_pucks) {
		if (unused_ifs.size() == 0) {
			break;
		}
		Position3DInterface *pos_if = unused_ifs.front();
		unused_ifs.pop_front();
		pos_if->set_visibility_history(1);
		(*if_puck_map_current)[new_puck] = pos_if;
		// logger->log_debug(name(), "Puck at (%f|%f) gets if %s", new_puck.getX(),
		//		new_puck.getY(), pos_if->id());
	}

	// count down visibility of unseen pucks
	for (Position3DInterface *unused_if : unused_ifs) {
		int vis = min(-1, unused_if->visibility_history() - 1);
		unused_if->set_visibility_history(vis);
		unused_if->write();
	}

	delete if_puck_map_;
	if_puck_map_ = if_puck_map_current;
	for (auto &p : *if_puck_map_) {
		Position3DInterface *pos_if = p.second;
		Point3d              relpos = relPositions_[p.first];
		pos_if->set_translation(0, relpos.getX());
		pos_if->set_translation(1, relpos.getY());
	}
}

OmniVisionPucksPipelineThread::Point3d
OmniVisionPucksPipelineThread::apply_tf_to_global(Point3d src)
{
	const char *source_frame = "/base_link";
	const char *target_frame = "/map";

	src.stamp = Time(0, 0);
	Point3d targetPoint;
	targetPoint.frame_id = target_frame;

	bool link_frame_exists  = tf_listener->frame_exists(target_frame);
	bool laser_frame_exists = tf_listener->frame_exists(source_frame);

	if (!link_frame_exists || !laser_frame_exists) {
		logger->log_warn(name(),
		                 "Frame missing: %s %s   %s %s",
		                 source_frame,
		                 link_frame_exists ? "exists" : "missing",
		                 target_frame,
		                 laser_frame_exists ? "exists" : "missing");
	} else {
		src.frame_id = source_frame;
		try {
			tf_listener->transform_point(target_frame, src, targetPoint);
		} catch (tf::ExtrapolationException &e) {
			logger->log_debug(name(), "Extrapolation error: %s", e.what());
			return src;
		} catch (tf::ConnectivityException &e) {
			logger->log_debug(name(), "Connectivity exception: %s", e.what());
			return src;
		} catch (Exception &e) {
			logger->log_debug(name(), "Fawkes exception: %s", e.what());
			return src;
		} catch (std::exception &e) {
			logger->log_debug(name(), "Generic exception: %s", e.what());
			return src;
		}

		return targetPoint;
	}
	return src;
}

OmniVisionPucksPipelineThread::sortFunctor::sortFunctor(firevision::RelativePositionModel *rp,
                                                        firevision::SimpleColorClassifier *c)
: relpos(rp), classifier(c)
{
}
bool
OmniVisionPucksPipelineThread::sortFunctor::operator()(firevision::ROI i, firevision::ROI j)
{
	float            leftdist, rightdist;
	fawkes::upoint_t mass_point_comp;
	classifier->get_mass_point_of_color(&i, &mass_point_comp);
	relpos->set_center(mass_point_comp.x, mass_point_comp.y);
	relpos->calc_unfiltered();
	leftdist = relpos->get_distance();
	classifier->get_mass_point_of_color(&j, &mass_point_comp);
	relpos->set_center(mass_point_comp.x, mass_point_comp.y);
	relpos->calc_unfiltered();
	rightdist = relpos->get_distance();
	if (leftdist <= rightdist) {
		return true;
	} else {
		return false;
	}
}
/** Lock data if fresh.*/
bool
OmniVisionPucksPipelineThread::lock_if_new_data()
{
	_data_mutex->lock();
	if (_new_data) {
		return true;
	} else {
		_data_mutex->unlock();
		return false;
	}
}
/** Unlock data, */
void
OmniVisionPucksPipelineThread::unlock()
{
	_new_data = false;
	_data_mutex->unlock();
}
