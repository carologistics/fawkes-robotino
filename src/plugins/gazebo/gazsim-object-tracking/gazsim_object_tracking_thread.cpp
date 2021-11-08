/***************************************************************************
 *  gazsim_object_tracking.cpp - Thread simulates object tracking of
 *      workpieces, conveyor belts, and slides while providing curresponding
 *      target frames used for visual servoing in Gazebo
 *
 *  Created: Sun May 16 12:03:10 2021
 *  Copyright  2021  Matteo Tschesche
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

#include "gazsim_object_tracking_thread.h"

#include <aspect/logging.h>
#include <interfaces/Position3DInterface.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <math.h>
#include <stdio.h>

using namespace fawkes;
using namespace gazebo;

/** @class ObjectTrackingThread "gazsim_object_tracking_thread.h"
 * Thread Simulates the Object Tracker in Gazebo
 * @author Matteo Tschesche
 */

/** Constructor. */
ObjectTrackingThread::ObjectTrackingThread()
: Thread("ObjectTrackingThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void
ObjectTrackingThread::init()
{
	logger->log_info(name(), "Initializing Simulation of the Workpiece (Puck) Positions");

	//read cofig values
	gps_topic_ = config->get_string("/gazsim/topics/gps");

	//subscribing to gazebo publisher
	localization_sub_ =
	  gazebo_world_node->Subscribe(gps_topic_, &ObjectTrackingThread::on_localization_msg, this);

	//init tracking
	tracked_pucks_ = 0;
	tracking_      = false;

	logger->log_info(name(), "Initializing Simulation of the MPS Positions");

	//read cofig values
	factory_topic_ = config->get_string("/gazsim/topics/factory");

	//subscribing to gazebo publisher
	factory_sub_ =
	  gazebo_world_node->Subscribe(factory_topic_, &ObjectTrackingThread::on_factory_msg, this);

	//read cofig values for computing expected position and target frames
	puck_size_   = double(config->get_float("plugins/object_tracking/puck_values/puck_size"));
	puck_height_ = double(config->get_float("plugins/object_tracking/puck_values/puck_height"));

	belt_height_ = double(config->get_float("plugins/object_tracking/belt_values/belt_height"));
	belt_lenght_ = double(config->get_float("plugins/object_tracking/belt_values/belt_lenght"));
	belt_offset_side_ =
	  double(config->get_float("plugins/object_tracking/belt_values/belt_offset_side"));

	slide_offset_side_ =
	  double(config->get_float("plugins/object_tracking/slide_values/slide_offset_side"));
	slide_height_ = double(config->get_float("plugins/object_tracking/slide_values/slide_height"));

	left_shelf_offset_side_ =
	  double(config->get_float("plugins/object_tracking/shelf_values/left_shelf_offset_side"));
	middle_shelf_offset_side_ =
	  double(config->get_float("plugins/object_tracking/shelf_values/middle_shelf_offset_side"));
	right_shelf_offset_side_ =
	  double(config->get_float("plugins/object_tracking/shelf_values/right_shelf_offset_side"));
	shelf_height_ = double(config->get_float("plugins/object_tracking/shelf_values/shelf_height"));

	gripper_offset_ =
	  double(config->get_float("plugins/object_tracking/target_frame_offsets/gripper_offset"));
	base_offset_ =
	  double(config->get_float("plugins/object_tracking/target_frame_offsets/base_offset"));
}

//updating Puck positions:
void
ObjectTrackingThread::on_localization_msg(ConstPosePtr &msg)
{
	//if object name starts with puck:
	if (!std::string(msg->name()).rfind("puck", 0)) {
		//is puck already tracked?
		for (int i = 0; i < tracked_pucks_; i++) {
			//if yes, update position
			if (!std::string(msg->name()).compare(puck_names_[i])) {
				puck_positions_[i][0] = msg->position().x();
				puck_positions_[i][1] = msg->position().y();
				puck_positions_[i][2] = msg->position().z();
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 "updated position");
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 std::to_string(positions_[i][0]).c_str());
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 std::to_string(positions_[i][1]).c_str());
				//logger->log_info(std::string(msg->name()).c_str(),
				//                 std::to_string(positions_[i][2]).c_str());
				return;
			}
			//else start tracking...
		}

		if (tracked_pucks_ < 50) {
			puck_names_[tracked_pucks_]        = std::string(msg->name());
			puck_positions_[tracked_pucks_][0] = msg->position().x();
			puck_positions_[tracked_pucks_][1] = msg->position().y();
			puck_positions_[tracked_pucks_][2] = msg->position().z();
			tracked_pucks_++;
			logger->log_info(std::string(msg->name()).c_str(), "is now tracked.");
		} else {
			logger->log_error(std::string(msg->name()).c_str(), "cannot be tracked! Array full!");
		}
	}
}

//save MPS positions at the start:
void
ObjectTrackingThread::on_factory_msg(ConstFactoryPtr &msg)
{
	//if MPS
	if (!std::string(msg->clone_model_name()).rfind("M-", 0)
	    || !std::string(msg->clone_model_name()).rfind("C-", 0)) {
		//find index and update position
		for (int i = 0; i < 14; i++) {
			if (!std::string(msg->clone_model_name()).compare(mps_names_[i])) {
				mps_positions_[i][0] = msg->pose().position().x();
				mps_positions_[i][1] = msg->pose().position().y();
				//converting wierd orientation value into Gazebo yaw value
				mps_positions_[i][2] = 2 * std::asin(msg->pose().orientation().z());
				logger->log_info(std::string(msg->clone_model_name()).c_str(), "updated position");
				//testing:
				if (!std::string(msg->clone_model_name()).compare(mps_names_[10])) {
					start_tracking(false, "C-CS1", "SHELF_RIGHT");
				}
				//logger->log_info(std::string(msg->clone_model_name()).c_str(),
				//                 std::to_string(mps_positions_[i][0]).c_str());
				//logger->log_info(std::string(msg->clone_model_name()).c_str(),
				//                 std::to_string(mps_positions_[i][1]).c_str());
				//logger->log_info(std::string(msg->clone_model_name()).c_str(),
				//                 std::to_string(mps_positions_[i][2]).c_str());
				return;
			}
		}
		logger->log_error(std::string(msg->clone_model_name()).c_str(), "not found.");
	}
}

//start tracking:
//  grab: true (track WP), false (track conveyor belt or slide)
//  mps_name: e.g. M-CS2
//  mps_side: input, output, slide, shelf_left, shelf_middle, shelf_right
void
ObjectTrackingThread::start_tracking(bool grab, std::string mps_name, std::string mps_side)
{
	bool mps_found = false;
	grab_          = grab;

	//find corresponding MPS
	for (int i = 0; i < 14; i++) {
		//find index and get MPS reference position
		if (!mps_name.compare(mps_names_[i])) {
			mps_x_    = mps_positions_[i][0];
			mps_y_    = mps_positions_[i][1];
			mps_ori_  = mps_positions_[i][2];
			mps_found = true;
			logger->log_info(mps_name.c_str(), "found MPS");
			logger->log_info(mps_name.c_str(), std::to_string(mps_positions_[i][0]).c_str());
			logger->log_info(mps_name.c_str(), std::to_string(mps_positions_[i][1]).c_str());
			logger->log_info(mps_name.c_str(), std::to_string(mps_positions_[i][2]).c_str());
		}
	}
	if (!mps_found) {
		logger->log_error(mps_name.c_str(), " is an invalid MPS-name!");
		return;
	}

	//find expected object position as middle point
	if (!mps_side.compare("INPUT_CONVEYOR")) {
		middle_x_ = compute_middle_x(belt_offset_side_);
		middle_y_ = compute_middle_y(belt_offset_side_);
		middle_z_ = belt_height_ + puck_height_ / 2;
	} else if (!mps_side.compare("OUTPUT_CONVEYOR")) {
		middle_x_ =
		  mps_x_ + belt_offset_side_ * cos(mps_ori_) + (belt_lenght_ / 2 - puck_size_) * sin(mps_ori_);
		middle_y_ =
		  mps_y_ + belt_offset_side_ * sin(mps_ori_) - (belt_lenght_ / 2 - puck_size_) * cos(mps_ori_);
		middle_z_ = belt_height_ + puck_height_ / 2;
	} else if (!mps_side.compare("SLIDE")) {
		middle_x_ = compute_middle_x(slide_offset_side_);
		middle_y_ = compute_middle_y(slide_offset_side_);
		middle_z_ = slide_height_ + puck_height_ / 2;
	} else if (!mps_side.compare("SHELF_LEFT")) {
		middle_x_ = compute_middle_x(left_shelf_offset_side_);
		middle_y_ = compute_middle_y(left_shelf_offset_side_);
		middle_z_ = shelf_height_ + puck_height_ / 2;
	} else if (!mps_side.compare("SHELF_MIDDLE")) {
		middle_x_ = compute_middle_x(middle_shelf_offset_side_);
		middle_y_ = compute_middle_y(middle_shelf_offset_side_);
		middle_z_ = shelf_height_ + puck_height_ / 2;
	} else if (!mps_side.compare("SHELF_RIGHT")) {
		middle_x_ = compute_middle_x(right_shelf_offset_side_);
		middle_y_ = compute_middle_y(right_shelf_offset_side_);
		middle_z_ = shelf_height_ + puck_height_ / 2;
	} else {
		logger->log_error(mps_side.c_str(), " is an invalid MPS-side!");
		return;
	}

	logger->log_info(name(), "Computed Middle Point --------------------");
	logger->log_info("middle_x: ", std::to_string(middle_x_).c_str());
	logger->log_info("middle_y: ", std::to_string(middle_y_).c_str());
	logger->log_info("middle_z: ", std::to_string(middle_z_).c_str());

	//if grab == true, find nearest workpiece (puck) to the middle point
	if (grab_) {
		double min_dist = 999;
		for (int i = 0; i < tracked_pucks_; i++) {
			if (distance_middle_to_puck(i) < min_dist) {
				min_dist          = distance_middle_to_puck(i);
				closest_puck_ind_ = i;
			}
		}
		//update middle position to closest WP middle point
		middle_x_ = puck_positions_[closest_puck_ind_][0];
		middle_y_ = puck_positions_[closest_puck_ind_][1];
		middle_z_ = puck_positions_[closest_puck_ind_][2];

		logger->log_info(name(), "Closest Puck Position -------------------");
		logger->log_info("middle_x: ", std::to_string(middle_x_).c_str());
		logger->log_info("middle_y: ", std::to_string(middle_y_).c_str());
		logger->log_info("middle_z: ", std::to_string(middle_z_).c_str());
	}
	tracking_ = true;
}

double
ObjectTrackingThread::compute_middle_x(double x_offset)
{
	return mps_x_ + x_offset * cos(mps_ori_) - (belt_lenght_ / 2 - puck_size_) * sin(mps_ori_);
}

double
ObjectTrackingThread::compute_middle_y(double y_offset)
{
	return mps_y_ + y_offset * sin(mps_ori_) + (belt_lenght_ / 2 - puck_size_) * cos(mps_ori_);
}

double
ObjectTrackingThread::distance_middle_to_puck(int puck_ind)
{
	return sqrt(pow(middle_x_ - puck_positions_[puck_ind][0], 2)
	            + pow(middle_y_ - puck_positions_[puck_ind][1], 2)
	            + pow(middle_z_ - puck_positions_[puck_ind][2], 2));
}
