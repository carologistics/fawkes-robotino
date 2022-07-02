/***************************************************************************
 *  tag_position_list.cpp - List for storing tag positions and their interfaces
 *
 *  Generated: Mon Mar 23 12:01:15 2015
 *  Copyright  2012  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "tag_position_list.h"

#include "tag_vision_thread.h"

#include <tf/types.h>
#include <utils/math/angle.h>

/** @class TagPositionList "tag_position_list.h"
 * This class handles the Tag Positions and the Blackboard communication for the
 * TagVision
 *
 * @author Randolph Maaßen
 */

using namespace fawkes;

/**
 * Creates an std::vector for handling TagVisionInterfaces for the TagVision.
 * The Interfaces are stored here and updated on the update_blackboard() method
 *
 * @param blackboard The blackboard used to publish on and to create / handle
 * the interfaces
 * @param tf_listener The transform listener
 * @param max_markers Maximum number of markers to detect at the same time
 * @param cam_frame The frame of reference for the tag positions
 * @param thread_name Thread name for log information
 * @param logger The loger used for logging
 * @param clock The fawkes clock, used to stamp the transforms
 * @param main_thread Pointer to the main plugin thread, used to create TF
 *        publishers
 */
TagPositionList::TagPositionList(fawkes::BlackBoard *     blackboard,
                                 fawkes::tf::Transformer *tf_listener,
                                 size_t                   max_markers,
                                 std::string              cam_frame,
                                 std::string              thread_name,
                                 fawkes::Logger *         logger,
                                 fawkes::Clock *          clock,
                                 TagVisionThread *        main_thread)
{
	// store parameters
	this->blackboard_  = blackboard;
	this->max_markers_ = max_markers;
	this->thread_name_ = thread_name;
	this->logger_      = logger;
	this->clock_       = clock;
	this->main_thread_ = main_thread;
	this->cam_frame_   = cam_frame;
	this->tf_listener_ = tf_listener;

	// create blackboard interfaces
	for (unsigned int i = 0; i < this->max_markers_; i++) {
		// create a name for the new interface
		std::string interface_name     = std::string("/tag-vision/") + std::to_string(i);
		std::string interface_name_map = std::string("/tag-vision/") + std::to_string(i) + "/to_map";
		// create the interface and store
		try {
			// get an interface from the blackboard
			auto interface =
			  this->blackboard_->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
			// set the frame of the interface
			interface->set_frame(cam_frame.c_str());
			// generate a helper class and push it into this vector

			// get an interface from the blackboard
			auto map_interface = this->blackboard_->open_for_writing<fawkes::Position3DInterface>(
			  (interface_name_map).c_str());
			// set the frame of the interface
			map_interface->set_frame("map");
			// generate a helper class and push it into this vector

			this->push_back(new TagPositionInterfaceHelper(interface,
			                                               map_interface,
			                                               i,
			                                               this->clock_,
			                                               main_thread_->get_tf_publisher(i, "tag_"),
			                                               main_thread_->get_tf_publisher(i, "map_tag_"),
			                                               tf_listener_,
			                                               cam_frame));
		} catch (std::exception &e) {
			this->logger_->log_error(thread_name.c_str(), "Could not open the blackboard: %s", e.what());
			throw(e);
		}
	}

	// initialize tag info interface
	try {
		this->index_interface_ =
		  blackboard->open_for_writing<fawkes::TagVisionInterface>("/tag-vision/info");
	} catch (std::exception &e) {
		this->logger_->log_error(thread_name.c_str(), "Could not open the blackboard: %s", e.what());
		throw(e);
	}
}

/**
 * The destructor closes all interfaces and frees allocated memory
 */
TagPositionList::~TagPositionList()
{
	// close tag vision interface
	this->blackboard_->close(this->index_interface_);

	// delete this interfaces
	for (auto &interface : *this) {
		this->blackboard_->close(interface->pos_iface());
		delete interface;
	}
}

TagPose
TagPositionList::get_laser_line_pose(fawkes::LaserLineInterface *laser_line_if)
{
	TagPose pose;

	float x = laser_line_if->end_point_1(0)
	          + (laser_line_if->end_point_2(0) - laser_line_if->end_point_1(0)) / 2;
	float y = laser_line_if->end_point_1(1)
	          + (laser_line_if->end_point_2(1) - laser_line_if->end_point_1(1)) / 2;
	float ori = fawkes::normalize_mirror_rad(laser_line_if->bearing());

	fawkes::tf::Quaternion f_q = fawkes::tf::create_quaternion_from_yaw(double(ori));
	tf::Point              f_p(tf::Scalar(x), tf::Scalar(y), 0.);

	fawkes::tf::Pose f_p_in(f_q, f_p);

	fawkes::tf::Stamped<fawkes::tf::Pose> f_sp_in(f_p_in,
	                                              laser_line_if->timestamp(),
	                                              laser_line_if->frame_id());

	fawkes::tf::Stamped<fawkes::tf::Pose> f_sp_out;

	try {
		//    logger_->log_info("tag_vision", "Transform from %s to %s",
		//    laser_line_if->frame_id(), frame_.c_str());
		tf_listener_->transform_pose(cam_frame_, f_sp_in, f_sp_out);
	} catch (fawkes::Exception &) {
		f_sp_in.stamp = fawkes::Time(0, 0);
		tf_listener_->transform_pose(cam_frame_, f_sp_in, f_sp_out);
		//    logger_->log_warn("tag_vision", "Can't transform laser-line; error
		//    %s\nuse newest", e.what());
	}

	pose.tvec = {f_sp_out.getOrigin().getX() * 1000,
	             f_sp_out.getOrigin().getY() * 1000,
	             f_sp_out.getOrigin().getZ() * 1000};

	fawkes::tf::Quaternion q_out         = f_sp_out.getRotation();
	fawkes::tf::Quaternion q_rot_for_cam = fawkes::tf::create_quaternion_from_rpy(M_PI_2, 0, -M_PI_2);
	q_out                                = q_out * q_rot_for_cam;

	pose.quaternion = {q_out.getW(), q_out.getX(), q_out.getY(), q_out.getZ()};

	return pose;
}

TagPose
TagPositionList::get_nearest_laser_line_pose(
  TagPose                                    tag_pose,
  std::vector<fawkes::LaserLineInterface *> *laser_line_ifs)
{
	double dist_closest = 10000000000000000.;

	TagPose ll_pose_closest; // = get_laser_line_pose( laser_line_ifs->at(0) );
	ll_pose_closest.tvec = {10000000, 10000000., 10000000.};
	// for each laser_line
	for (unsigned int i = 0; i < laser_line_ifs->size(); ++i) {
		laser_line_ifs->at(i)->read();
		if (laser_line_ifs->at(i)->visibility_history() > 0) {
			// get centeroid
			// transform to what we need ;)
			TagPose ll_pose_current;
			try {
				ll_pose_current = get_laser_line_pose(laser_line_ifs->at(i));
			} catch (fawkes::Exception &) {
				continue;
			}
			// check if closest (translation)
			double dist_current = sqrt((ll_pose_current.tvec[0] - tag_pose.tvec[0])
			                             * (ll_pose_current.tvec[0] - tag_pose.tvec[0])
			                           + (ll_pose_current.tvec[1] - tag_pose.tvec[1])
			                               * (ll_pose_current.tvec[1] - tag_pose.tvec[1])
			                           + (ll_pose_current.tvec[2] - tag_pose.tvec[2])
			                               * (ll_pose_current.tvec[2] - tag_pose.tvec[2]));
			// save closest
			if (dist_current < dist_closest) {
				ll_pose_closest = ll_pose_current;
				dist_closest    = dist_current;
			}
		}
	}

	// check if the choosen laser-line is ok, to use (don't use anything far away
	if (dist_closest <= 150) {
		return ll_pose_closest;
	} else {
		//    logger_->log_info("tag_vision", "can't find sutable laser-line, use
		//    tag; dist is: %lf", dist_closest);
		return tag_pose;
	}
}

/**
 * Find a blackboard interface manager that we can update with the given
 * ALVAR marker.
 * @param marker
 * @return An unused or matching interface manager.
 */
TagPositionInterfaceHelper *
TagPositionList::find_suitable_interface(const TagVisionMarker &marker) const
{
	int                         min_vis_hist = std::numeric_limits<int>::max();
	TagPositionInterfaceHelper *rv           = nullptr;

	for (TagPositionInterfaceHelper *interface : *this) {
		if (interface->marker_id() == marker.marker_id
		    || interface->marker_id() == EMPTY_INTERFACE_MARKER_ID)
			return interface;

		if (interface->visibility_history() < min_vis_hist) {
			min_vis_hist = interface->visibility_history();
			rv           = interface;
		}
	}

	if (min_vis_hist > INTERFACE_UNSEEN_BOUND)
		return nullptr;

	return rv;
}

/**
 * Assignes every marker found to an interface. The interface will stay the same
 * for a marker as long as the marker is considered seen (visibility history >
 * 0). It also updates the Marker IDs on the TagVision interface.
 *
 * @param marker_list The detected markers.
 * @param laser_line_ifs Laser lines for orientation sanity check
 */
void
TagPositionList::update_blackboard(std::vector<TagVisionMarker> &             marker_list,
                                   std::vector<fawkes::LaserLineInterface *> *laser_line_ifs)
{
	int i = 0;
	for (auto &marker : marker_list) {
		// skip the marker, if the pose is directly on the camera (error)
		TagPose tmp_pose = marker.pose;
		if (tmp_pose.tvec[0] < 1 && tmp_pose.tvec[1] < 1 && tmp_pose.tvec[2] < 1) {
			continue;
		}
		try {
			TagPose ll_pose = get_nearest_laser_line_pose(tmp_pose, laser_line_ifs);
			tmp_pose        = ll_pose;
		} catch (std::exception &e) {
			logger_->log_error(thread_name_.c_str(), "Failed to match tag to laser line: %s", e.what());
		}
		++i;

		TagPositionInterfaceHelper *marker_interface = find_suitable_interface(marker);
		if (marker_interface) {
			if (marker_interface->marker_id() != marker.marker_id) {
				marker_interface->set_marker_id(marker.marker_id);
				marker_interface->set_visibility_history(0);
			}
			marker_interface->set_pose(tmp_pose, marker.type);
		} else
			logger_->log_warn(thread_name_.c_str(),
			                  "Cannot publish tag #%u: Index interface full!",
			                  marker.marker_id);
	}
	// update blackboard with interfaces
	int32_t visible_markers = 0;
	for (TagPositionInterfaceHelper *pos_iface : *this) {
		this->index_interface_->set_tag_id(pos_iface->index(), pos_iface->marker_id());
		if (pos_iface->marker_id() != EMPTY_INTERFACE_MARKER_ID) {
			visible_markers++;
		}
		pos_iface->write();
	}
	this->index_interface_->set_tags_visible(visible_markers);
	this->index_interface_->write();
}
