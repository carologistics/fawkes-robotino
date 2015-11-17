/***************************************************************************
 *  gazsim_tag_vision_plugin.cpp - Plugin provides ground-truth tag vision
 *
 *  Created: Thu Nov 05 20:20:38 2015
 *  Copyright  2015 Frederik Zwilling
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

#include "gazsim_tag_vision_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <set>

#include <interfaces/SwitchInterface.h>
#include <interfaces/Position3DInterface.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class TagVisionSimThread "gazsim_tag_vision_thread.h"
 * Thread gets ground truth tag-vision results from gazebo and writes them to the blackboard
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
TagVisionSimThread::TagVisionSimThread()
  : Thread("TagVisionSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void TagVisionSimThread::init()
{
  logger->log_debug(name(), "Initializing TagVisionSim Plugin");

  //read config values
  gazebo_topic_ = config->get_string("/gazsim/topics/tag-vision");
  tag_if_name_prefix_ = config->get_string("/gazsim/tag-vision/tag-if-name-prefix");
  info_if_name_ = config->get_string("/gazsim/tag-vision/info-if-name");
  number_interfaces_ = config->get_int("/gazsim/tag-vision/number-interfaces");
  visibility_history_increase_per_update_ = config->get_int("/gazsim/tag-vision/visibility-history-increase-per-update");
  
  //open interfaces
  info_if_ = blackboard->open_for_writing<fawkes::TagVisionInterface>(info_if_name_.c_str());
  for(int i = 0; i < number_interfaces_; i++)
  {
    std::ostringstream ss;
    ss << tag_if_name_prefix_.c_str() << i;
    tag_pos_ifs_[i] = blackboard->open_for_writing<fawkes::Position3DInterface>
      (ss.str().c_str());
  } 

  //subscribing to gazebo tag-vision messages
  tag_vision_sub_ = gazebonode->Subscribe
    (gazebo_topic_, &TagVisionSimThread::on_tag_vision_msg, this);

  new_data_ = false;
}

void TagVisionSimThread::finalize()
{
  for(std::map<int, Position3DInterface*>::iterator it = tag_pos_ifs_.begin(); it != tag_pos_ifs_.end(); it++)
  {
    blackboard->close(it->second);
  }
  blackboard->close(info_if_);
}

void TagVisionSimThread::loop()
{
  if(new_data_)
  {
    new_data_ = false;
    //go through all visible tags and write the data into the interfaces
    std::set<int> used_ifs;
    for(int i = 0; i < last_msg_.pose_size(); i++)
    {
      gazebo::msgs::Pose pose = last_msg_.pose(i);
      int tag_id = pose.id();
      //find associated interface or unused interface index
      int if_index = -1;
      if(map_id_if_.find(tag_id) != map_id_if_.end())
      {
        if_index = map_id_if_[tag_id];
      }
      else
      {
        for(int j = 0; j < number_interfaces_; j++)
        {
          if(tag_pos_ifs_[j]->visibility_history() <= 0)
          {
            if_index = j;
            map_id_if_[tag_id] = if_index;
            break;
          }
        }
      }
      if(if_index == -1)
      {
        //no interface left to write data
        logger->log_info(name(), "Not enough interfaces to write all tag-poses to the blackboard.\n");
        continue;
      }
      tag_pos_ifs_[if_index]->set_translation(0, pose.position().x());
      tag_pos_ifs_[if_index]->set_translation(1, pose.position().y());
      tag_pos_ifs_[if_index]->set_translation(2, pose.position().z());
      tag_pos_ifs_[if_index]->set_rotation(0, pose.orientation().x());
      tag_pos_ifs_[if_index]->set_rotation(1, pose.orientation().y());
      tag_pos_ifs_[if_index]->set_rotation(2, pose.orientation().z());
      tag_pos_ifs_[if_index]->set_rotation(3, pose.orientation().w());
      tag_pos_ifs_[if_index]->set_frame("/gazsim_tag_vision");
      //compute visibility history
      int current_vis_hist = tag_pos_ifs_[if_index]->visibility_history();
      if(current_vis_hist < 0)
      {
        current_vis_hist = 0;
      }
      current_vis_hist += visibility_history_increase_per_update_;
      tag_pos_ifs_[if_index]->set_visibility_history(current_vis_hist);
      tag_pos_ifs_[if_index]->write();
      info_if_->set_tag_id(if_index, tag_id);
      used_ifs.insert(if_index);
    }
    //clear tags which became unvisibe and count visible tags
    int number_found_tags = 0;
    for(int i = 0; i < number_interfaces_; i++)
    {
      if(used_ifs.find(i) == used_ifs.end())
      {
        //compute visibility history
        int current_vis_hist = tag_pos_ifs_[i]->visibility_history();
        if(current_vis_hist > 0)
        {
          current_vis_hist = 0;
        }
        current_vis_hist -= visibility_history_increase_per_update_;
        tag_pos_ifs_[i]->set_visibility_history(current_vis_hist);
        tag_pos_ifs_[i]->write();
        info_if_->set_tag_id(i, 0);
      }
      else
      {
        number_found_tags++;
      }
    }
    info_if_->set_tags_visible(number_found_tags);
    info_if_->write();
 }
}

void TagVisionSimThread::on_tag_vision_msg(ConstPosesStampedPtr &msg)
{
  // logger->log_info(name(), "Got new TagVision result from gazebo.\n");
  last_msg_.CopyFrom(*msg);
  new_data_ = true;
}
