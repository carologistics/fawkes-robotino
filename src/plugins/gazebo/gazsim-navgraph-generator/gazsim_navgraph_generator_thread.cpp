/***************************************************************************
 *	gazsim_navgraph_generator_thread.cpp - Thread for generating navgraph
 *without exploration phase
 *
 *	Created: Mon Feb 15 11:31:11 2016
 *	Copyright	2016	David Schmidt
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

#include "gazsim_navgraph_generator_thread.h"

#include <interfaces/NavGraphWithMPSGeneratorInterface.h>

#include <string>
#include <unordered_map>

using namespace fawkes;

/** @class GazsimNavgraphGeneratorThread "gazsim_navgraph_generator_thread.h"
 * Thread for generating navgraph without exploration phase.
 * @author David Schmidt
 */

/** Constructor. */
GazsimNavgraphGeneratorThread::GazsimNavgraphGeneratorThread()
: Thread("GazsimNavgraphGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE),
  task_finished_(false),
  computation_is_running_(false)
{
}

void
GazsimNavgraphGeneratorThread::init()
{
	logger->log_debug(name(), "Initializing GazsimNavgraphGenerator Plugin");

	// read config values
	tags_            = config->get_strings("/gazsim/navgraph-generator/all-active-tags");
	related_mps_     = config->get_strings("/gazsim/navgraph-generator/related-mps");
	nav_gen_if_name_ = config->get_string("/gazsim/navgraph-generator/nav-gen-if-name");

	// open interfaces
	nav_gen_if_ = blackboard->open_for_reading<fawkes::NavGraphWithMPSGeneratorInterface>(
	  nav_gen_if_name_.data());

	// subscribing to gazebo tag messages
	for (unsigned i = 0; i < tags_.size(); ++i)
		subscriber_tags_.push_back(
		  gazebo_world_node->Subscribe(tags_[i], &GazsimNavgraphGeneratorThread::on_tag_msg, this));
	tag_msgs_.clear();

	// sort the mpsIDs to the tagIDs in mps_id_
	get_mpsID_by_tagID();
}

void
GazsimNavgraphGeneratorThread::finalize()
{
}

void
GazsimNavgraphGeneratorThread::loop()
{
	// check if navgraph is already computed
	if (task_finished_)
		return;
	// check if computation of navgraph is running
	if (computation_is_running_) {
		nav_gen_if_->read();
		if (nav_gen_if_->is_final()) {
			task_finished_          = true;
			computation_is_running_ = false;
			blackboard->close(nav_gen_if_);
			logger->log_info(name(), "Navgraph is generated!");
		}
		return;
	}
	// check if all tag-messages were received
	if (tag_msgs_.size() < tags_.size())
		return;

	// check if the transform map nedded by navgraph-generator exists otherwise
	// wait
	if (!tf_listener->frame_exists("map")) {
		logger->log_debug(name(), "Waiting until frame map exists");
		return;
	}

	// send the position of all tags
	for (std::map<int, gazebo::msgs::Pose>::iterator it = tag_msgs_.begin(); it != tag_msgs_.end();
	     ++it) {
		//		logger->log_info(name(), "tag %i gets sent to
		// NavgraphGenerator.",
		//		                 (*it).first);
		send_station_msg((*it).first, (*it).second);
	}
	bool *allFalse = new bool[24];
	for (int i = 0; i < 24; ++i)
		allFalse[i] = false;
	NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage *delete_explo_navgraph_msg =
	  new NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage(allFalse);
	// delete_explo_navgraph_msg->set_zones(allFalse);
	nav_gen_if_->msgq_enqueue(delete_explo_navgraph_msg);
	compute_msg_ = new NavGraphWithMPSGeneratorInterface::ComputeMessage();
	nav_gen_if_->msgq_enqueue(compute_msg_);
	computation_is_running_ = true;

	logger->log_info(name(), "Start unsubscribing!");
	while (!subscriber_tags_.empty()) {
		// subscriber_tags_.back()->Unsubscribe();
		// gazebo 4.x seems to unsubscribe by deleting pointer on next line
		subscriber_tags_.pop_back();
	}
	logger->log_info(name(), "Finished unsubscribing!");
}

void
GazsimNavgraphGeneratorThread::on_tag_msg(ConstPosePtr &msg)
{
	int underscore = msg->name().find('_');
	int id         = std::atoi(msg->name().substr(underscore + 1).data());
	if (msg->position().x() > -1 && msg->position().x() < 1 && msg->position().y() < 0)
		return;
	tag_msgs_[id].CopyFrom(*msg);
}

void
GazsimNavgraphGeneratorThread::get_mpsID_by_tagID()
{
	if (tags_.size() != related_mps_.size()) {
		logger->log_error(name(),
		                  "There are %zu tags defined, but %zu!=%zu related mps!",
		                  tags_.size(),
		                  related_mps_.size(),
		                  tags_.size());
		return;
	}
	for (unsigned i = 0; i < tags_.size(); ++i) {
		int underscore = tags_[i].find('_');
		int slash      = tags_[i].substr(underscore + 1).find('/');
		int id         = std::atoi(tags_[i].substr(underscore + 1, slash).data());
		mps_id_[id]    = related_mps_[i];
		//		logger->log_info(name(), "Full tag name:  %s", tags_[i].data());
		//		logger->log_info(name(), "Extracted id:   %i", id);
		//		logger->log_info(name(), "Related MPS-ID: %s",
		// mps_id_[id].data());
	}
}

void
GazsimNavgraphGeneratorThread::send_station_msg(int id, gazebo::msgs::Pose pose)
{
	NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage *stationMsg =
	  new NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage();
	stationMsg->set_name(mps_id_[id].data());
	stationMsg->set_side(NavGraphWithMPSGeneratorInterface::Side::INPUT);
	stationMsg->set_frame("map");
	stationMsg->set_tag_translation(0, pose.position().x());
	stationMsg->set_tag_translation(1, pose.position().y());
	stationMsg->set_tag_translation(2, pose.position().z());
	stationMsg->set_tag_rotation(0, pose.orientation().x());
	stationMsg->set_tag_rotation(1, pose.orientation().y());
	stationMsg->set_tag_rotation(2, pose.orientation().z());
	stationMsg->set_tag_rotation(3, pose.orientation().w());
	auto zone_coords = get_zone_coords(pose.position().x(), pose.position().y());
	stationMsg->set_zone_coords(zone_coords.data());
	/*
	logger->log_info(name(), "ID:%i", id);
	logger->log_info(name(), "Name:%s", mps_id_[id].data());
	logger->log_info(
	  name(), "Position:%f,%f,%f", pose.position().x(), pose.position().y(), pose.position().z());
	logger->log_info(name(), "Zone: (%d,%d)", zone_coords[0], zone_coords[1]);
	*/
	nav_gen_if_->msgq_enqueue(stationMsg);
}

std::array<int16_t, 2>
GazsimNavgraphGeneratorThread::get_zone_coords(float x, float y)
{
	std::array<int16_t, 2> coords;
	coords[0] = x < 0 ? floor(x) : ceil(x);
	coords[1] = y < 0 ? floor(y) : ceil(y);
	return coords;
}
