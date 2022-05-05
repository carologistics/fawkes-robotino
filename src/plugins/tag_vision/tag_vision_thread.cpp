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
#include <tf/types.h>

#include <math.h>
#include <opencv2/opencv.hpp>
#include <string>

#define CFG_PREFIX "/plugins/tag_vision/"
#define IMAGE_CAHNNELS 3

using namespace fawkes;
#ifdef HAVE_ALVAR
using namespace alvar;
#endif
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
	// Marker type
	std::string marker_type_str = config->get_string((prefix + "marker_type").c_str());
	// load marker size and apply it
	marker_size_ = config->get_uint((prefix + "marker_size").c_str());
	if (marker_type_str.find("ARUCO") != std::string::npos
	    || marker_type_str.find("APRILTAG") != std::string::npos) {
		std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> aruco_tag_type_lookup = {
		  {"ARUCO_4X4_50", cv::aruco::DICT_4X4_50},
		  {"ARUCO_4X4_100", cv::aruco::DICT_4X4_100},
		  {"ARUCO_4X4_250", cv::aruco::DICT_4X4_250},
		  {"ARUCO_4X4_1000", cv::aruco::DICT_4X4_1000},
		  {"ARUCO_5X5_50", cv::aruco::DICT_5X5_50},
		  {"ARUCO_5X5_100", cv::aruco::DICT_5X5_100},
		  {"ARUCO_5X5_250", cv::aruco::DICT_5X5_250},
		  {"ARUCO_5X5_1000", cv::aruco::DICT_5X5_1000},
		  {"ARUCO_6X6_50", cv::aruco::DICT_6X6_50},
		  {"ARUCO_6X6_100", cv::aruco::DICT_6X6_100},
		  {"ARUCO_6X6_250", cv::aruco::DICT_6X6_250},
		  {"ARUCO_6X6_1000", cv::aruco::DICT_6X6_1000},
		  {"ARUCO_7X7_50", cv::aruco::DICT_7X7_50},
		  {"ARUCO_7X7_100", cv::aruco::DICT_7X7_100},
		  {"ARUCO_7X7_250", cv::aruco::DICT_7X7_250},
		  {"ARUCO_7X7_1000", cv::aruco::DICT_7X7_1000},
		  {"ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
		  {"APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
		  {"APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
		  {"APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
		  {"APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}};
		marker_type_ = MarkerType::ARUCO;
		auto it      = aruco_tag_type_lookup.find(marker_type_str);
		if (it != aruco_tag_type_lookup.end()) {
			aruco_tag_type_ = it->second;
		} else {
			throw Exception("Invalid Aruco marker type selected!");
		}
	} else if (marker_type_str == "ALVAR")
		marker_type_ = MarkerType::ALVAR;
	else
		throw fawkes::Exception("Invalid marker type selected!");
	markers_ = std::make_shared<std::vector<TagVisionMarker>>();

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
	ipl_image_    = cv::Mat(cv::Size(this->img_width_, this->img_height_), CV_8UC3, 3);

	// set up marker
	max_marker_ = 16;

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
	switch (marker_type_) {
	case ALVAR:
#ifdef HAVE_ALVAR
		// load alvar camera calibration
		if (!alvar_cam_.SetCalib(
		      config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),
		      0,
		      0,
		      FILE_FORMAT_DEFAULT)) {
			this->logger->log_warn(this->name(), "Failed to load calibration file");
		}
		alvar_detector_.SetMarkerSize(marker_size_);

		// set camera resolution
		alvar_cam_.SetRes(this->img_width_, this->img_height_);
#else
		throw fawkes::Exception("Cannot detect alvar tags, ALVAR not found.");
#endif
		break;
	case ARUCO:
		cameraMatrix_ = (cv::Mat_<double>(3, 3) << 547.4, 0, 317.50, 0, 544.05, 170.31, 0, 0, 1);
		distCoeffs_   = (cv::Mat_<double>(1, 4) << 0, 0, 0, 0);
		break;
	default: break;
	}
}

void
TagVisionThread::finalize()
{
	vision_master->unregister_thread(this);
	config->rem_change_handler(this);
	// free the markers
	this->markers_->clear();
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
	this->markers_->clear();
	switch (marker_type_) {
	case MarkerType::ALVAR: {
		// detect makres on image
#ifdef HAVE_ALVAR
		alvar_detector_.Detect(ipl_image_, &alvar_cam_);
		// reset currently saved markers
		// fill output array
		for (alvar::MarkerData &tmp_alvar_marker : *(this->alvar_detector_.markers)) {
			alvar::Pose tmp_pose   = tmp_alvar_marker.pose;
			cv::Mat     quaternion = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
			tmp_pose.GetQuaternion(quaternion);
			TagVisionMarker tmp_marker{{{tmp_pose.translation[ALVAR_TRANS::A_T_X],
			                             tmp_pose.translation[ALVAR_TRANS::A_T_Y],
			                             tmp_pose.translation[ALVAR_TRANS::A_T_Z]},
			                            {quaternion.at<double>(0, 0),
			                             quaternion.at<double>(1, 0),
			                             quaternion.at<double>(2, 0),
			                             quaternion.at<double>(3, 0)}},
			                           tmp_alvar_marker.GetId()};
			// skip the marker, if the pose is directly on the camera (error)
			if (tmp_pose.translation[0] < 1 && tmp_pose.translation[1] < 1
			    && tmp_pose.translation[2] < 1) {
				continue;
			}
			this->markers_->push_back(tmp_marker);
			// add up to markers
			tmp_alvar_marker.Visualize(ipl_image_, &alvar_cam_);
		}
#endif
		break;
	}
	case MarkerType::ARUCO: {
		std::vector<int>                       markerIds;
		std::vector<std::vector<cv::Point2f>>  markerCorners, rejectedCandidates;
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(aruco_tag_type_);
		cv::aruco::detectMarkers(
		  ipl_image_, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		// if at least one marker detected
		if (markerIds.size() > 0) {
			cv::aruco::drawDetectedMarkers(ipl_image_, markerCorners, markerIds);
		}
		std::vector<cv::Vec3d> rvecs, tvecs;
		for (std::vector<int>::size_type i = 0; i < markerIds.size(); i++) {
			cv::aruco::estimatePoseSingleMarkers(
			  markerCorners, marker_size_ / 1000., cameraMatrix_, distCoeffs_, rvecs, tvecs);
			cv::Mat rot_matrix;
			cv::Rodrigues(rvecs[i], rot_matrix);
			auto   tvec_scaled = 1000. * tvecs[i];
			double m00, m01, m02, m10, m11, m12, m20, m21, m22, qw, qx, qy, qz;
			m00 = rot_matrix.at<double>(0, 0);
			m01 = rot_matrix.at<double>(0, 1);
			m02 = rot_matrix.at<double>(0, 2);
			m10 = rot_matrix.at<double>(1, 0);
			m11 = rot_matrix.at<double>(1, 1);
			m12 = rot_matrix.at<double>(1, 2);
			m20 = rot_matrix.at<double>(2, 0);
			m21 = rot_matrix.at<double>(2, 1);
			m22 = rot_matrix.at<double>(2, 2);

			double tr =
			  rot_matrix.at<double>(0, 0) + rot_matrix.at<double>(1, 1) + rot_matrix.at<double>(2, 2);

			if (tr > 0) {
				double S = sqrt(tr + 1.0) * 2; // S=4*qw
				qw       = 0.25 * S;
				qx       = (m21 - m12) / S;
				qy       = (m02 - m20) / S;
				qz       = (m10 - m01) / S;
			} else if ((m00 > m11) & (m00 > m22)) {
				double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
				qw       = (m21 - m12) / S;
				qx       = 0.25 * S;
				qy       = (m01 + m10) / S;
				qz       = (m02 + m20) / S;
			} else if (m11 > m22) {
				double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
				qw       = (m02 - m20) / S;
				qx       = (m01 + m10) / S;
				qy       = 0.25 * S;
				qz       = (m12 + m21) / S;
			} else {
				double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
				qw       = (m10 - m01) / S;
				qx       = (m02 + m20) / S;
				qy       = (m12 + m21) / S;
				qz       = 0.25 * S;
			}

			TagVisionMarker tmp_marker{{tvec_scaled, {qw, qx, qy, qz}}, markerIds[i]};
			markers_->push_back(tmp_marker);
			//		cv::Mat outputImage;
			//		ipl_image_.copyTo(outputImage);
			//		for (unsigned int i = 0; i < rvecs.size(); ++i) {
			//			auto rvec = rvecs[i];
			//			auto tvec = tvecs[i];
			//			cv::drawFrameAxes(outputImage, cameraMatrix_, distCoeffs_, rvec, tvec, 0.1);
			//		}
			//		cv::imshow("out", outputImage);
			//		cv::waitKey(1);
		}
		break;
	}
	default:
		logger->log_error(name(), "Marker detection skipped, specified marker type not supported");
		break;
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
#ifdef HAVE_AR_TRACK_ALVAR
			// load alvar camera calibration
			alvar_cam_.SetCalib(config->get_string((prefix + "alvar_camera_calib_file").c_str()).c_str(),
			                    0,
			                    0,
			                    FILE_FORMAT_DEFAULT);
			// load marker size and apply it
			marker_size_ = config->get_uint((prefix + "marker_size").c_str());
			alvar_detector_.SetMarkerSize(marker_size_);
#endif
		} catch (fawkes::Exception &e) {
			logger->log_error(name(), e);
		}
	}
	// gets called for every changed entry... so init is called once per change.
	cfg_mutex_.unlock();
}
