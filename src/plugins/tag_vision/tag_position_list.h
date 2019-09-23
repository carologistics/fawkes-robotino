/***************************************************************************
 *  tag_position_list.h - List for storing tag positions and their interfaces
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

#ifndef TAG_POSITION_LIST_H
#define TAG_POSITION_LIST_H

#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#include <interfaces/TagVisionInterface.h>

#include <string>
#include <vector>
#ifdef HAVE_AR_TRACK_ALVAR
#	include <ar_track_alvar/Marker.h>
#else
#	include <alvar/Marker.h>
#endif
#include "tag_position_interface_helper.h"

#include <aspect/tf.h>
#include <interfaces/LaserLineInterface.h>
#include <logging/logger.h>

class TagVisionThread;

class TagPositionList : public std::vector<TagPositionInterfaceHelper *>
{
public:
	TagPositionList(fawkes::BlackBoard *     blackboard,
	                fawkes::tf::Transformer *tf_listener,
	                size_t                   max_markers,
	                std::string              cam_frame,
	                std::string              thread_name,
	                fawkes::Logger *         logger,
	                fawkes::Clock *          clock,
	                TagVisionThread *        main_thread);
	/// Destructor
	~TagPositionList();

	void update_blackboard(std::vector<alvar::MarkerData> *           marker_list,
	                       std::vector<fawkes::LaserLineInterface *> *laser_line_ifs);

	TagPositionInterfaceHelper *find_suitable_interface(const alvar::MarkerData &) const;

private:
	/// How many markers can be detected at the same time
	size_t max_markers_;
	/// The blackboard to publish on
	fawkes::BlackBoard *blackboard_;
	/// Tag vision inforamtion interface
	fawkes::TagVisionInterface *index_interface_;
	/// Name of the calling thread
	std::string thread_name_;
	/// Logger for logging
	fawkes::Logger *logger_;
	/// The clock for the StampedTransforms
	fawkes::Clock *clock_;
	/// Publisher for the transforms
	TagVisionThread *main_thread_;

	std::string              cam_frame_;
	fawkes::tf::Transformer *tf_listener_;

	alvar::Pose get_laser_line_pose(fawkes::LaserLineInterface *laser_line_if);
	alvar::Pose
	get_nearest_laser_line_pose(alvar::Pose                                tag_pose,
	                            std::vector<fawkes::LaserLineInterface *> *laser_line_ifs);
};

#endif // TAG_POSITION_LIST_H
