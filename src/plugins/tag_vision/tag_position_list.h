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

#include <vector>
#include <string>
#include <interfaces/TagVisionInterface.h>
#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#ifdef HAVE_AR_TRACK_ALVAR
#  include <ar_track_alvar/Marker.h>
#else
#  include <alvar/Marker.h>
#endif
#include <logging/logger.h>

#include "tag_position_interface_helper.h"


class TagPositionList : public std::vector<TagPositionInterfaceHelper*>
{
public:
  /// Constructor
  TagPositionList(fawkes::BlackBoard *blackboard, u_int32_t max_markers, std::string frame, std::string thread_name, fawkes::Logger *logger_, fawkes::Clock *clock, fawkes::tf::TransformPublisher *tf_publisher);
  /// Destructor
  ~TagPositionList();
  /// Update the blackboard with the stored data
  void update_blackboard(std::vector<alvar::MarkerData> *marker_list);

private:
  /// How many markers can be detected at the same time
  u_int32_t max_markers_;
  /// The blackboard to publish on
  fawkes::BlackBoard *blackboard_;
  /// Tag vision inforamtion interface
  fawkes::TagVisionInterface *tag_vision_interface_;
  /// Name of the calling thread
  std::string thread_name_;
  /// Logger for logging
  fawkes::Logger *logger_;
  /// The clock for the StampedTransforms
  fawkes::Clock *clock_;
  /// Publisher for the transforms
  fawkes::tf::TransformPublisher *tf_publisher_;
};

#endif // TAG_POSITION_LIST_H
