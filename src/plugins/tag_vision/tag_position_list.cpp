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

#include <utils/math/angle.h>
#include <tf/types.h>

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
TagPositionList::TagPositionList(fawkes::BlackBoard *blackboard, fawkes::tf::Transformer *tf_listener, u_int32_t max_markers, std::string frame, std::string thread_name, fawkes::Logger *logger, fawkes::Clock *clock, fawkes::tf::TransformPublisher *tf_publisher)
{
  // store parameters
  this->blackboard_ = blackboard;
  this->max_markers_ = max_markers;
  this->thread_name_ = thread_name;
  this->logger_ = logger;
  this->clock_ = clock;
  this->tf_publisher_ = tf_publisher;
  frame_ = frame;
  this->tf_listener = tf_listener;

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

alvar::Pose
TagPositionList::get_laser_line_pose(fawkes::LaserLineInterface *laser_line_if)
{
  alvar::Pose pose;

  double x = laser_line_if->end_point_1(0) + ( laser_line_if->end_point_2(0) - laser_line_if->end_point_1(0) ) / 2;
  double y = laser_line_if->end_point_1(1) + ( laser_line_if->end_point_2(1) - laser_line_if->end_point_1(1) ) / 2;
  double ori = fawkes::normalize_mirror_rad( laser_line_if->bearing() );

  fawkes::tf::Quaternion f_q = fawkes::tf::create_quaternion_from_yaw(ori);
  double q[4];
  fawkes::tf::Point f_p(x, y, 0.);

  fawkes::tf::Pose f_p_in(f_q, f_p);

  fawkes::tf::Stamped<fawkes::tf::Pose> f_sp_in(f_p_in, laser_line_if->timestamp(), laser_line_if->frame_id());

  fawkes::tf::Stamped<fawkes::tf::Pose> f_sp_out;

  try {
//    logger_->log_info("tag_vision", "Transform from %s to %s", laser_line_if->frame_id(), frame_.c_str());
    tf_listener->transform_pose(frame_, f_sp_in, f_sp_out);
  } catch (fawkes::Exception &e) {
    f_sp_in.stamp = fawkes::Time(0, 0);
    tf_listener->transform_pose(frame_, f_sp_in, f_sp_out);
//    logger_->log_warn("tag_vision", "Can't transform laser-line; error %s\nuse newest", e.what());
  }

  pose.translation[0] = f_sp_out.getOrigin().getX() * 1000;
  pose.translation[1] = f_sp_out.getOrigin().getY() * 1000;
  pose.translation[2] = f_sp_out.getOrigin().getZ() * 1000;

  fawkes::tf::Quaternion q_out = f_sp_out.getRotation();
  fawkes::tf::Quaternion q_rot_for_cam = fawkes::tf::create_quaternion_from_rpy(M_PI_2, 0, -M_PI_2);
  q_out = q_out * q_rot_for_cam;
  q[0] = q_out.getW();
  q[1] = q_out.getX();
  q[2] = q_out.getY();
  q[3] = q_out.getZ();

  pose.SetQuaternion(q);

  return pose;
}

alvar::Pose
TagPositionList::get_nearest_laser_line_pose(alvar::Pose tag_pose, std::vector<fawkes::LaserLineInterface*> *laser_line_ifs)
{
  double dist_closest = 10000000000000000.;

  alvar::Pose ll_pose_clostest;// = get_laser_line_pose( laser_line_ifs->at(0) );
  ll_pose_clostest.translation[0] = 10000000.;
  ll_pose_clostest.translation[1] = 10000000.;
  ll_pose_clostest.translation[2] = 10000000.;
  // for each laser_line
  for (unsigned int i = 0; i < laser_line_ifs->size(); ++i) {
    laser_line_ifs->at(i)->read();
    if (laser_line_ifs->at(i)->visibility_history() > 0) {
      // get centeroid
      // transform to what we need ;)
      alvar::Pose ll_pose_current;
      try {
        ll_pose_current = get_laser_line_pose( laser_line_ifs->at(i) );
      } catch(fawkes::Exception &e) {
        continue;
      }
      // check if clostest (translation)
      double dist_current  =  sqrt(
                                    ( ll_pose_current.translation[0] - tag_pose.translation[0] ) * ( ll_pose_current.translation[0] - tag_pose.translation[0] ) +
                                    ( ll_pose_current.translation[1] - tag_pose.translation[1] ) * ( ll_pose_current.translation[1] - tag_pose.translation[1] ) +
                                    ( ll_pose_current.translation[2] - tag_pose.translation[2] ) * ( ll_pose_current.translation[2] - tag_pose.translation[2] )
                                  );
      // save closest
      if (dist_current < dist_closest) {
        ll_pose_clostest = ll_pose_current;
        dist_closest = dist_current;
      }
    }
  }

  // check if the choosen laser-line is ok, to use (don't use anything far away
  if (dist_closest <= 150) {
    return ll_pose_clostest;
  } else {
//    logger_->log_info("tag_vision", "can't find sutable laser-line, use tag; dist is: %lf", dist_closest);
    return tag_pose;
  }
}

/**
 * Assignes every marker found to an interface. The interface will stay the same for
 * a marker as long as the marker is considered seen (visibility history > 0). It
 * also updates the Marker IDs on the TagVision interface.
 *
 * @param marker_list The detected markers.
 */
void TagPositionList::update_blackboard(std::vector<alvar::MarkerData> *marker_list, std::vector<fawkes::LaserLineInterface*> *laser_line_ifs)
{
  int i = 0;
  for(alvar::MarkerData& marker: *marker_list)
  {
    // skip the marker, if the pose is directly on the camera (error)
    alvar::Pose tmp_pose = marker.pose;
    if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 && tmp_pose.translation[2]<1){
      logger_->log_info("tag_vision", "don't use tag");
      continue;
    }
    try {
      alvar::Pose ll_pose = get_nearest_laser_line_pose(tmp_pose, laser_line_ifs);
//      logger_->log_info("tag_vision", "%i before: %f\t%f\t%f", i, tmp_pose.translation[0], tmp_pose.translation[1], tmp_pose.translation[2]);
      tmp_pose = ll_pose;
//      logger_->log_info("tag_vision", "%i after:  %f\t%f\t%f", i, tmp_pose.translation[0], tmp_pose.translation[1], tmp_pose.translation[2]);
    } catch (std::exception &e) {
      logger_->log_error("tag_vision", "some strange exception that where not expected");
    }
    ++i;
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
