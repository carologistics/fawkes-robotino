/***************************************************************************
 *  gazsim-navgraph-generator_thread.cpp - Thread for generating navgraph without exploration phase
 *
 *  Created: Mon Feb 15 11:31:11 2016
 *  Copyright  2016  David Schmidt
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

#include "gazsim-navgraph-generator_thread.h"

//#include <tf/types.h>
//#include <interfaces/Position3DInterface.h>

using namespace fawkes;

/** @class GazsimNavgraphGeneratorThread "gazsim-navgraph-generator_thread.h"
 * Thread for generating navgraph without exploration phase.
 * @author David Schmidt
 */

/** Constructor. */
GazsimNavgraphGeneratorThread::GazsimNavgraphGeneratorThread()
  : Thread("GazsimNavgraphGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void
GazsimNavgraphGeneratorThread::init()
{
  logger->log_debug(name(), "Initializing GazsimNavgraphGenerator Plugin");

  //read config values
  tag_01_ = config->get_string("/gazsim/navgraph-generator/tag01");
	/*
  gazebo_topic_ = config->get_string("/gazsim/topics/tag-vision");
  tag_if_name_prefix_ = config->get_string("/gazsim/tag-vision/tag-if-name-prefix");
  info_if_name_ = config->get_string("/gazsim/tag-vision/info-if-name");
  frame_name_ = config->get_string("/gazsim/tag-vision/frame");
  number_interfaces_ = config->get_int("/gazsim/tag-vision/number-interfaces");
  visibility_history_increase_per_update_ = config->get_int("/gazsim/tag-vision/visibility-history-increase-per-update");
	*/
  
  //open interfaces
  /*
  info_if_ = blackboard->open_for_writing<fawkes::TagVisionInterface>(info_if_name_.c_str());
  for(int i = 0; i < number_interfaces_; i++)
  {
    std::ostringstream ss;
    ss << tag_if_name_prefix_.c_str() << i;
    tag_pos_ifs_[i] = blackboard->open_for_writing<fawkes::Position3DInterface>
      (ss.str().c_str());
  } 
  */
  
  //subscribing to gazebo tag-vision messages
  subscriber_tag_01_ = gazebonode->Subscribe
    (tag_01_, &GazsimNavgraphGeneratorThread::on_tag_vision_msg, this);
}

void
GazsimNavgraphGeneratorThread::finalize()
{
  // blackboard->close(pose_if_);
}

void
GazsimNavgraphGeneratorThread::loop()
{
  if(new_data_)
  {
	new_data_ = false;
	/*
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
	  tag_pos_ifs_[if_index]->set_frame(frame_name_.c_str());
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


	  // publish the transform
	  tf::Transform transform(tf::Quaternion(pose.orientation().x(),
											 pose.orientation().y(),
											 pose.orientation().z(),
											 pose.orientation().w()),
							  tf::Vector3(pose.position().x(),
										  pose.position().y(),
										  pose.position().z()));
	  Time time(clock);
	  std::string tag_frame_name = std::string("tag_")+std::to_string(if_index);
	  tf::StampedTransform stamped_transform(transform,time,frame_name_,tag_frame_name);
	  tf_publisher->send_transform(stamped_transform);
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
	*/
 }
}


void GazsimNavgraphGeneratorThread::on_tag_vision_msg(ConstPosesStampedPtr &msg)
{
  // logger->log_info(name(), "Got new TagVision result from gazebo.\n");
  last_msg_.CopyFrom(*msg);
  new_data_ = true;
}
