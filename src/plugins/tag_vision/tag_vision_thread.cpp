/***************************************************************************
 *  tag_vision_thread.cpp - Thread to print the robot's position to the log
 *
 *  Created: Thu Sep 27 14:31:11 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "tag_vision_thread.h"

#include <interfaces/Position3DInterface.h>
#include <opencv2/opencv.hpp>
#include <tf/types.h>

#include <math.h>
#include <string>

#define CFG_PREFIX "/plugins/tag_vision/"
#define IMAGE_CAHNNELS 3

using namespace fawkes;
using namespace alvar;
using namespace std;

const std::string TagVisionThread::tag_frame_basename = "tag_";

/** @class TagVisionThread "tag_vision_thread.h"
 * Thread to identify AR Tags and provid e their position
 * @author Nicolas Limpert & Randolph Maaßen
 */

/** Constructor. */
TagVisionThread::TagVisionThread()
: Thread("TagVisionThread", Thread::OPMODE_WAITFORWAKEUP),
  VisionAspect(VisionAspect::CYCLIC),
  ConfigurationChangeHandler(CFG_PREFIX),
  fawkes::TransformAspect(fawkes::TransformAspect::BOTH_DEFER_PUBLISHER)
{
	fv_cam_       = nullptr;
	shm_buffer_   = nullptr;
	image_buffer_ = nullptr;
	markers_      = nullptr;
}

void
TagVisionThread::init()
{
	config->add_change_handler(this);
	// load config
	// config prefix in string for concatinating
	std::string prefix = CFG_PREFIX;
	// log, that we open load the config
	logger->log_info(name(), "loading config");
	// load alvar camera calibration
	if (!alvar_cam_.SetCalib(config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),
	                         0,
	                         0,
	                         FILE_FORMAT_DEFAULT)) {
		this->logger->log_warn(this->name(), "Faild to load calibration file");
	}
	// load marker size and apply it
	marker_size_ = config->get_uint((prefix + "marker_size").c_str());
	alvar_detector_.SetMarkerSize(marker_size_);

	// Image Buffer ID
	shm_id_ = config->get_string((prefix + "shm_image_id").c_str());

	// init firevision camera
	// CAM swapping not working (??)
	if (fv_cam_ != nullptr) {
		// free the camera
		fv_cam_->stop();
		fv_cam_->flush();
		fv_cam_->dispose_buffer();
		fv_cam_->close();
		delete fv_cam_;
		fv_cam_ = nullptr;
	}
	if (fv_cam_ == nullptr) {
		std::string connection = this->config->get_string((prefix + "camera").c_str());
		fv_cam_                = vision_master->register_for_camera(connection.c_str(), this);
		fv_cam_->start();
		fv_cam_->open();
		this->img_width_  = fv_cam_->pixel_width();
		this->img_height_ = fv_cam_->pixel_height();
	}

	// set camera resolution
	alvar_cam_.SetRes(this->img_width_, this->img_height_);

	// SHM image buffer
	if (shm_buffer_ != nullptr) {
		delete shm_buffer_;
		shm_buffer_   = nullptr;
		image_buffer_ = nullptr;
	}

	shm_buffer_ = new firevision::SharedMemoryImageBuffer(shm_id_.c_str(),
	                                                      firevision::YUV422_PLANAR,
	                                                      this->img_width_,
	                                                      this->img_height_);
	if (!shm_buffer_->is_valid()) {
		delete shm_buffer_;
		delete fv_cam_;
		shm_buffer_ = nullptr;
		fv_cam_     = nullptr;
		throw fawkes::Exception("Shared memory segment not valid");
	}
	std::string frame = this->config->get_string((prefix + "frame").c_str());
	shm_buffer_->set_frame_id(frame.c_str());

	image_buffer_ = shm_buffer_->buffer();
	ipl_image_ =
      cv::Mat(cv::Size(this->img_width_, this->img_height_), CV_8UC1, IMAGE_CAHNNELS);

	// set up marker
	max_marker_    = 16;
	this->markers_ = new std::vector<alvar::MarkerData>();

	this->tag_interfaces_ = new TagPositionList(this->blackboard,
	                                            tf_listener,
	                                            this->max_marker_,
	                                            frame,
	                                            this->name(),
	                                            this->logger,
	                                            this->clock,
	                                            this);
	// get laser-line interfaces
	laser_line_ifs_ = new std::vector<fawkes::LaserLineInterface *>();
	for (int i = 1; i <= 8; i++) {
		// std::string if_name = "/laser-lines/" + i;
		std::string if_name = "/laser-lines/" + std::to_string(i);

		fawkes::LaserLineInterface *ll_if =
		  blackboard->open_for_reading<fawkes::LaserLineInterface>(if_name.c_str());
		laser_line_ifs_->push_back(ll_if);
	}
}

void
TagVisionThread::finalize()
{
	vision_master->unregister_thread(this);
	config->rem_change_handler(this);
	// free the markers
	this->markers_->clear();
	delete this->markers_;
	delete fv_cam_;
	fv_cam_ = nullptr;
	delete shm_buffer_;
	shm_buffer_   = nullptr;
	image_buffer_ = nullptr;
    ipl_image_.release();
	delete this->tag_interfaces_;

	while (!laser_line_ifs_->empty()) {
		blackboard->close(laser_line_ifs_->back());
		laser_line_ifs_->pop_back();
	}
	delete laser_line_ifs_;
}

/**
 * Get the appropriate TransformPublisher for the given tag index
 * @param idx The tag index
 * @return The TransformPublisher to be used for the tag index
 */
tf::TransformPublisher *
TagVisionThread::get_tf_publisher(size_t idx)
{
	if (tf_publishers.find(tag_frame_basename + std::to_string(idx)) == tf_publishers.end())
		tf_add_publisher("%s%ld", tag_frame_basename.c_str(), idx);

	return tf_publishers[tag_frame_basename + std::to_string(idx)];
}

void
TagVisionThread::loop()
{
	if (!cfg_mutex_.try_lock()) {
		// logger->log_info(name(),"Skipping loop");
		return;
	}
	if (fv_cam_ == nullptr || !fv_cam_->ready()) {
		logger->log_info(name(), "Camera not ready");
		init();
		return;
	}
	// logger->log_info(name(),"entering loop");
	// get img form fv
	fv_cam_->capture();
	firevision::convert(fv_cam_->colorspace(),
	                    firevision::YUV422_PLANAR,
	                    fv_cam_->buffer(),
	                    image_buffer_,
	                    this->img_width_,
	                    this->img_height_);
	fv_cam_->dispose_buffer();
	// convert img
    firevision::CvMatAdapter::convert_image_bgr(image_buffer_, ipl_image_);
	// get marker from img
	get_marker();

	this->tag_interfaces_->update_blackboard(this->markers_, laser_line_ifs_);

	cfg_mutex_.unlock();
}

void
TagVisionThread::get_marker()
{
	// detect makres on image
	alvar_detector_.Detect(ipl_image_, &alvar_cam_);
	// reset currently saved markers
	this->markers_->clear();
	// fill output array
	for (alvar::MarkerData &tmp_marker : *(this->alvar_detector_.markers)) {
		Pose tmp_pose = tmp_marker.pose;
		// skip the marker, if the pose is directly on the camera (error)
		if (tmp_pose.translation[0] < 1 && tmp_pose.translation[1] < 1 && tmp_pose.translation[2] < 1) {
			continue;
		}
		this->markers_->push_back(tmp_marker);
		// add up to markers
		tmp_marker.Visualize(ipl_image_, &alvar_cam_);
	}
    firevision::CvMatAdapter::convert_image_yuv422_planar(ipl_image_, image_buffer_);
}

// config handling
void TagVisionThread::config_value_erased(const char *path){};
void TagVisionThread::config_tag_changed(const char *new_tag){};
void TagVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v){};
void
TagVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
	if (cfg_mutex_.try_lock()) {
		try {
			std::string prefix = CFG_PREFIX;
			// log, that we open load the config
			logger->log_info(name(), "loading config");
			// load alvar camera calibration
			alvar_cam_.SetCalib(config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),
			                    0,
			                    0,
			                    FILE_FORMAT_DEFAULT);
			// load marker size and apply it
			marker_size_ = config->get_uint((prefix + "marker_size").c_str());
			alvar_detector_.SetMarkerSize(marker_size_);
		} catch (fawkes::Exception &e) {
			logger->log_error(name(), e);
		}
	}
	// gets called for every changed entry... so init is called once per change.
	cfg_mutex_.unlock();
}
