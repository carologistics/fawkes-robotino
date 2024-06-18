/***************************************************************************
 *  tag_position_interface.h - Interface handler for tag position
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

#ifndef TAG_POSITION_INTREFACE_H
#define TAG_POSITION_INTREFACE_H

#include <interfaces/Position3DInterface.h>
#include <tf/transform_publisher.h>
#include <tf/transformer.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <string>

#ifdef HAVE_AR_TRACK_ALVAR
#	include <ar_track_alvar/Pose.h>
#else
#	ifdef HAVE_ALVAR
#		include <alvar/Pose.h>
#	endif
#endif

#define EMPTY_INTERFACE_MARKER_ID 0
#define INTERFACE_UNSEEN_BOUND -5

/// Type of the Marker
enum MarkerType { ARUCO, ALVAR };

/// Pose of a tag
struct TagPose
{
	/// Translation
	cv::Vec3d tvec;
	/// Rotation
	cv::Vec4d quaternion;
};
/// Stores all requred information of a marker
struct TagVisionMarker
{
	/// Pose of the mmarker
	TagPose pose;
	/// ID of the marker
	unsigned int marker_id;
	/// Marker Type
	MarkerType type;
};

enum ROT { X = 0, Y = 1, Z = 2, W = 3 };

enum ALVAR_ROT {
	A_W = 0,
	A_X = 1,
	A_Y = 2,
	A_Z = 3,
};

enum CV_ROT {
	CV_W = 0,
	CV_X = 1,
	CV_Y = 2,
	CV_Z = 3,
};

enum TRANS {
	T_X = 0,
	T_Y = 1,
	T_Z = 2,
};

class TagPositionInterfaceHelper
{
public:
	/// Constructor
	TagPositionInterfaceHelper(u_int32_t                       index_,
	                           fawkes::Clock                  *clock,
	                           fawkes::tf::TransformPublisher *tf_publisher,
	                           fawkes::tf::TransformPublisher *map_tf_publisher,
	                           fawkes::tf::Transformer        *tf_listener,
	                           std::string                     frame);
	/// Destructor
	~TagPositionInterfaceHelper();

	/// Update the position of the interface
	void set_pose(TagPose new_pose, MarkerType marker_type);

	/// Write the interface on the blackboard
	void write();

	unsigned long marker_id() const;
	void          set_marker_id(unsigned long new_id);
	int32_t       visibility_history() const;
	void          set_visibility_history(int32_t);

	std::string get_zone();
	int         get_discrete_ori();

	/**
   * @brief Returns the position of this interface in the TagPositionList, or
   * any other enumeration
   *
   * @return The position of this interface of the list
   */
	size_t
	index()
	{
		return this->index_;
	}
	/**
   * @brief Returns the stored map pose
   *
   * @return The pose in map frame
   */
	TagPose
	map_pose()
	{
		return this->map_pose_;
	}

private:
	/// The visibility history for the interface to handle
	int32_t visibility_history_;

	/// Marker, whether the interface was updated since last write
	bool was_seen_;

	/// The id of the marker the interface represents
	unsigned long marker_id_;

	/// The position of the interface in the vector
	size_t index_;

	/// The frame of reference for this tag
	std::string cam_frame_;

	/// The child frame / name of the transform
	std::string tag_frame_;

	/// The clock to get the time stamps for StampedTransform nad Transform
	/// publishing
	fawkes::Clock *clock_;

	/// The transform publisher
	fawkes::tf::TransformPublisher *tf_publisher_;
	fawkes::tf::TransformPublisher *map_tf_publisher_;
	fawkes::tf::Transformer        *tf_listener_;

	MarkerType marker_type_;
	TagPose    map_pose_;
	TagPose    cam_pose_;
};

#endif // TAG_POSITION_INTREFACE_H
