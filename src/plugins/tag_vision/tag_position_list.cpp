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

/** @class TagPositionList "tag_position_list.h"
 * This class handles the Tag Positions and the Blackboard communication for the TagVision
 *
 * @author Randolph Maaßen
 */

/**
 * Creates an std::vector for handling TagVisionInterfaces for the TagVision. The Interfaces are stored here and updated on the update_blackboard() method
 *
 * @param blackboard The blackboard used to publish on and to create / handle the interfaces
 * @param max_markers Maximum number of markers to detect at the same time
 * @param frame The frame of reference for the tag positions
 * @param thread_name Thread name for log information
 * @param logger The loger used for logging
 * @param clock The fawkes clock, used to stamp the transforms
 * @param tf_publisher The fawes transform publisher, used to publish the transforms of the tags
 */
TagPositionList::TagPositionList(fawkes::BlackBoard *blackboard, u_int32_t max_markers, std::string frame, std::string thread_name, fawkes::Logger *logger, fawkes::Clock *clock, fawkes::tf::TransformPublisher *tf_publisher)
{
  // store parameters
  this->blackboard_ = blackboard;
  this->max_markers_ = max_markers;
  this->thread_name_ = thread_name;
  this->logger_ = logger;
  this->clock_ = clock;
  this->tf_publisher_ = tf_publisher;

  // create blackboard interfaces
  for(size_t i=0; i < this->max_markers_; i++)
  {
    // create a name for the new interface
    std::string interface_name = std::string("/tag-vision/") + std::to_string(i);
    // create the interface and store
    try
    {
      // get an interface from the blackboard
      auto interface = this->blackboard_->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
      // set the frame of the interface
      interface->set_frame(frame.c_str());
      // generate a helper class and push it into this vector
      this->push_back(new TagPositionInterfaceHelper(interface, i, this->clock_, this->tf_publisher_, frame));
    }
    catch (std::exception &e)
    {
      this->logger_->log_error(thread_name.c_str(),(std::string("Could not open the blackboard: ") + std::string(e.what())).c_str());
      throw(e);
    }
  }

  // initialize tag info interface
  try
  {
    this->tag_vision_interface_ = blackboard->open_for_writing<fawkes::TagVisionInterface>("/tag-vision/info");
  }
  catch (std::exception &e)
  {
    this->logger_->log_error(thread_name.c_str(),(std::string("Could not open the blackboard: ") + std::string(e.what())).c_str());
    throw(e);
  }
}

/**
 * The destructor closes all interfaces and frees allocated memory
 */
TagPositionList::~TagPositionList()
{
  // close tag vision interface
  this->blackboard_->close(this->tag_vision_interface_);

  // delete this interfaces
  for(auto &&interface: *this)
  {
    this->blackboard_->close(interface->interface());
    delete interface;
  }
}

/**
 * Assignes every marker found to an interface. The interface will stay the same for
 * a marker as long as the marker is considered seen (visibility history > 0). It
 * also updates the Marker IDs on the TagVision interface.
 *
 * @param marker_list The detected markers.
 */
void TagPositionList::update_blackboard(std::vector<alvar::MarkerData> *marker_list)
{
  for(alvar::MarkerData& marker: *marker_list)
  {
    // skip the marker, if the pose is directly on the camera (error)
    alvar::Pose tmp_pose = marker.pose;
    if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 && tmp_pose.translation[2]<1){
        continue;
    }
    // get the id of the marker
    u_int32_t marker_id=marker.GetId();

    // find an interface with this marker assigned or an empty interface
    TagPositionInterfaceHelper *marker_interface = NULL;
    TagPositionInterfaceHelper *empty_interface = NULL;
    for(TagPositionInterfaceHelper *interface: *this)
    {
      // assign marker_interface
      if(interface->marker_id() == marker_id){
        marker_interface = interface;
      }
      // assign empty interface
      if(empty_interface == NULL && interface->marker_id() == EMPTY_INTERFACE_MARKER_ID)
      {
        empty_interface = interface;
      }
    }
    // if no marker interface is found, assign the empty interface and assign the marker id
    if(marker_interface == NULL)
    {
      // no empty interface found, cannot find any suitable interface, 
      if(empty_interface == NULL)
      {
        continue;
      }
      marker_interface = empty_interface;
      marker_interface->set_marker_id(marker_id);
    }
    // continue with the marker interface
    marker_interface->set_pose(tmp_pose);
  }
  // update blackboard with interfaces
  u_int32_t visible_markers = 0;
  for(TagPositionInterfaceHelper *interface: *this)
  {
    this->tag_vision_interface_->set_tag_id(interface->vector_position(), interface->marker_id());
    if(interface->marker_id() != EMPTY_INTERFACE_MARKER_ID)
    {
      visible_markers++;
    }
    interface->write();
  }
  this->tag_vision_interface_->set_tags_visible(visible_markers);
  this->tag_vision_interface_->write();
}
