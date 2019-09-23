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
#include <tf/types.h>
#include <utils/math/angle.h>

#include <string>

#ifdef HAVE_AR_TRACK_ALVAR
#	include <ar_track_alvar/Pose.h>
#else
#	include <alvar/Pose.h>
#endif

#define EMPTY_INTERFACE_MARKER_ID 0
#define INTERFACE_UNSEEN_BOUND -5

class TagPositionInterfaceHelper
{
	enum ROT { X = 0, Y = 1, Z = 2, W = 3 };

	enum ALVAR_ROT {
		A_W = 0,
		A_X = 1,
		A_Y = 2,
		A_Z = 3,
	};

	enum TRANS {
		T_X = 0,
		T_Y = 1,
		T_Z = 2,
	};

	enum ALVAR_TRANS {
		A_T_X = 2,
		A_T_Y = 0,
		A_T_Z = 1,
	};

public:
	/// Constructor
	TagPositionInterfaceHelper(fawkes::Position3DInterface *   position_interface,
	                           u_int32_t                       index_,
	                           fawkes::Clock *                 clock,
	                           fawkes::tf::TransformPublisher *tf_publisher,
	                           std::string                     frame);
	/// Destructor
	~TagPositionInterfaceHelper();

	/// Update the position of the interface
	void set_pose(alvar::Pose new_pose);

	/// Write the interface on the blackboard
	void write();

	unsigned long marker_id() const;
	void          set_marker_id(unsigned long new_id);
	int32_t       visibility_history() const;
	void          set_visibility_history(int32_t);

	/**
   * @brief Returns the Position3D interface of this plugin
   *
   * @return The interface
   */
	fawkes::Position3DInterface *
	pos_iface()
	{
		return this->pos_iface_;
	}

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

private:
	/// The interface to handle
	fawkes::Position3DInterface *pos_iface_;

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
};

#endif // TAG_POSITION_INTREFACE_H
