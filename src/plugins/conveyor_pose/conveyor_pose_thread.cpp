/***************************************************************************
 *  conveyor_pose_thread.cpp - conveyor_pose thread
 *
 *  Created: Thr 12. April 16:28:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
 *             2018 Victor Mataré
 *             2018 Morian Sonnet
 *
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

#include "conveyor_pose_thread.h"

#include "recognition_thread.h"

#include <core/exceptions/system.h>
#include <interfaces/ConveyorPoseInterface.h>
#include <interfaces/LaserLineInterface.h>
#include <interfaces/SwitchInterface.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <tf/types.h>
#include <utils/math/angle.h>
#include <utils/time/clock.h>

#include <cmath>
#include <cstdio>

using namespace fawkes;

/** @class ConveyorPoseThread "conveyor_pose_thread.cpp"
 * Plugin to detect the conveyor belt in a pointcloud (captured from Intel
 * RealSense)
 * @author Tobias Neumann
 */

/** Constructor. */
ConveyorPoseThread::ConveyorPoseThread()
: Thread("ConveyorPoseThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  ConfigurationChangeHandler(CFG_PREFIX),
  fawkes::TransformAspect(fawkes::TransformAspect::BOTH, "conveyor_pose"),
  result_fitness_(std::numeric_limits<double>::min()),
  syncpoint_ready_for_icp_name_("/perception/conveyor_pose/clouds_ready"),
  cloud_out_raw_name_("raw"),
  cloud_out_trimmed_name_("trimmed"),
  current_mps_type_(ConveyorPoseInterface::DEFAULT_TYPE),
  current_mps_target_(ConveyorPoseInterface::DEFAULT_TARGET),
  recognition_thread_(nullptr),
  realsense_switch_(nullptr)
{
}

std::string
ConveyorPoseThread::get_model_path(ConveyorPoseInterface *           iface,
                                   ConveyorPoseInterface::MPS_TYPE   type,
                                   ConveyorPoseInterface::MPS_TARGET target)
{
	std::string path = std::string(CFG_PREFIX "/reference_models/")
	                   + iface->enum_tostring("MPS_TYPE", type) + "_"
	                   + iface->enum_tostring("MPS_TARGET", target);
	if (config->exists(path)) {
		logger->log_info(name(),
		                 "Override for %s_%s: %s",
		                 iface->enum_tostring("MPS_TYPE", type),
		                 iface->enum_tostring("MPS_TARGET", target),
		                 config->get_string(path).c_str());
		return CONFDIR "/" + config->get_string(path);
	} else {
		switch (target) {
		case ConveyorPoseInterface::INPUT_CONVEYOR:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/input_conveyor");
		case ConveyorPoseInterface::OUTPUT_CONVEYOR:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/output_conveyor");
		case ConveyorPoseInterface::SHELF_LEFT:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/shelf_left");
		case ConveyorPoseInterface::SHELF_MIDDLE:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/shelf_middle");
		case ConveyorPoseInterface::SHELF_RIGHT:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/shelf_right");
		case ConveyorPoseInterface::SLIDE:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/slide");
		case ConveyorPoseInterface::DEFAULT_TARGET:
			return CONFDIR "/" + config->get_string(CFG_PREFIX "/reference_models/default");
		case ConveyorPoseInterface::LAST_MPS_TARGET_ELEMENT: return "";
		}
	}

	return "";
}

void
ConveyorPoseThread::init()
{
	config->add_change_handler(this);

	syncpoint_ready_for_icp_ =
	  syncpoint_manager->get_syncpoint(name(), syncpoint_ready_for_icp_name_);
	syncpoint_ready_for_icp_->register_emitter(name());

	cfg_debug_mode_  = config->get_bool(CFG_PREFIX "/debug");
	cfg_force_shelf_ = config->get_int_or_default(CFG_PREFIX "/force_shelf", -1);
	cloud_in_name_   = config->get_string(CFG_PREFIX "/cloud_in");

	cfg_if_prefix_ = config->get_string(CFG_PREFIX "/if/prefix");
	if (cfg_if_prefix_.back() != '/')
		cfg_if_prefix_.append("/");

	laserlines_names_ = config->get_strings(CFG_PREFIX "/if/laser_lines");

	conveyor_frame_id_ = config->get_string(CFG_PREFIX "/conveyor_frame_id");

	recognition_thread_->cfg_icp_max_corr_dist_ =
	  config->get_float(CFG_PREFIX "/icp/max_correspondence_dist");
	recognition_thread_->cfg_icp_min_corr_dist_ =
	  double(config->get_float(CFG_PREFIX "/icp/min_correspondence_dist"));
	recognition_thread_->cfg_icp_max_iterations_ = config->get_int(CFG_PREFIX "/icp/max_iterations");
	recognition_thread_->cfg_icp_refinement_factor_ =
	  double(config->get_float(CFG_PREFIX "/icp/refinement_factor"));
	recognition_thread_->cfg_icp_tf_epsilon_ =
	  double(config->get_float(CFG_PREFIX "/icp/transformation_epsilon"));
	recognition_thread_->cfg_icp_min_loops_    = config->get_uint(CFG_PREFIX "/icp/min_loops");
	recognition_thread_->cfg_icp_max_loops_    = config->get_uint(CFG_PREFIX "/icp/max_loops");
	recognition_thread_->cfg_icp_max_retries_  = config->get_uint(CFG_PREFIX "/icp/max_retries");
	recognition_thread_->cfg_icp_auto_restart_ = config->get_bool(CFG_PREFIX "/icp/auto_restart");

	recognition_thread_->cfg_icp_hv_inlier_thresh_ =
	  config->get_float(CFG_PREFIX "/icp/hv/inlier_threshold");
	recognition_thread_->cfg_icp_hv_penalty_thresh_ =
	  config->get_float(CFG_PREFIX "/icp/hv/penalty_threshold");
	recognition_thread_->cfg_icp_hv_support_thresh_ =
	  config->get_float(CFG_PREFIX "/icp/hv/support_threshold");

	recognition_thread_->cfg_icp_shelf_hv_inlier_thresh_ =
	  config->get_float(CFG_PREFIX "/icp/hv/shelf_inlier_threshold");
	recognition_thread_->cfg_icp_shelf_hv_penalty_thresh_ =
	  config->get_float(CFG_PREFIX "/icp/hv/shelf_penalty_threshold");
	recognition_thread_->cfg_icp_shelf_hv_support_thresh_ =
	  config->get_float(CFG_PREFIX "/icp/hv/shelf_support_threshold");

	bb_pose_ =
	  blackboard->open_for_writing<ConveyorPoseInterface>((cfg_if_prefix_ + "status").c_str());
	bb_pose_->set_current_mps_type(bb_pose_->DEFAULT_TYPE);
	bb_pose_->set_current_mps_target(bb_pose_->DEFAULT_TARGET);

	bb_pose_->write();

	// Init of station target hints
	cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR];
	cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR];
	cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT];
	cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT];
	cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE];
	cfg_target_hint_[ConveyorPoseInterface::SLIDE];
	cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET];

	cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/input_conveyor/x");
	cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][Y_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/input_conveyor/y");
	cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/input_conveyor/z");

	cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/input_conveyor/x");
	// Y * -1, because it should be the opposite of the input conveyor
	cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][Y_DIR] =
	  -config->get_float(CFG_PREFIX "/icp/hint/input_conveyor/y");
	cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/input_conveyor/z");

	cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/left_shelf/x");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][Y_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/left_shelf/y");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/left_shelf/z");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/middle_shelf/x");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][Y_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/middle_shelf/y");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/middle_shelf/z");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/right_shelf/x");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][Y_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/right_shelf/y");
	cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/right_shelf/z");
	cfg_target_hint_[ConveyorPoseInterface::SLIDE][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/slide/x");
	cfg_target_hint_[ConveyorPoseInterface::SLIDE][Y_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/slide/y");
	cfg_target_hint_[ConveyorPoseInterface::SLIDE][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/slide/z");

	cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET][X_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/default/x");
	cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET][Y_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/default/y");
	cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET][Z_DIR] =
	  config->get_float(CFG_PREFIX "/icp/hint/default/z");

	// Init of station type hints

	cfg_type_offset_[ConveyorPoseInterface::BASE_STATION];
	cfg_type_offset_[ConveyorPoseInterface::CAP_STATION];
	cfg_type_offset_[ConveyorPoseInterface::RING_STATION];
	cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION];
	cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION];

	cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][X_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/base_station/x", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][Y_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/base_station/y", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][Z_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/base_station/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][X_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/cap_station/x", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][Y_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/cap_station/y", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][Z_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/cap_station/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::RING_STATION][X_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/ring_station/x", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::RING_STATION][Y_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/ring_station/y", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::RING_STATION][Z_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/ring_station/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][X_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/delivery_station/x", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][Y_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/delivery_station/y", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][Z_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/delivery_station/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][X_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/storage_station/x", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][Y_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/storage_station/y", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][Z_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/storage_station/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::DEFAULT_TYPE][X_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/default/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::DEFAULT_TYPE][Y_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/default/z", 0.f);
	cfg_type_offset_[ConveyorPoseInterface::DEFAULT_TYPE][Z_DIR] =
	  config->get_float_or_default(CFG_PREFIX "/icp/conveyor_offset/default/z", 0.f);

	//-- initial guess source
	std::string cfg_bb_initial_guess_topic =
	  config->get_string(CFG_PREFIX "/icp/init_guess_interface_id");
	bb_init_guess_pose_ =
	  blackboard->open_for_reading<fawkes::Position3DInterface>(cfg_bb_initial_guess_topic.c_str());

	cfg_max_timediff_external_pc_ =
	  static_cast<double>(config->get_float(CFG_PREFIX "/icp/max_timediff_external_pc"));
	cfg_external_timeout_ =
	  static_cast<double>(config->get_float(CFG_PREFIX "/icp/external_timeout"));

	cfg_enable_switch_ = config->get_bool(CFG_PREFIX "/switch_default");

	// cut information

	cfg_left_cut_no_ll_   = config->get_float(CFG_PREFIX "/without_ll/left_cut");
	cfg_right_cut_no_ll_  = config->get_float(CFG_PREFIX "/without_ll/right_cut");
	cfg_top_cut_no_ll_    = config->get_float(CFG_PREFIX "/without_ll/top_cut");
	cfg_bottom_cut_no_ll_ = config->get_float(CFG_PREFIX "/without_ll/bottom_cut");
	cfg_front_cut_no_ll_  = config->get_float(CFG_PREFIX "/without_ll/front_cut");
	cfg_back_cut_no_ll_   = config->get_float(CFG_PREFIX "/without_ll/back_cut");

	cfg_left_cut_   = config->get_float(CFG_PREFIX "/with_ll/left_cut");
	cfg_right_cut_  = config->get_float(CFG_PREFIX "/with_ll/right_cut");
	cfg_top_cut_    = config->get_float(CFG_PREFIX "/with_ll/top_cut");
	cfg_bottom_cut_ = config->get_float(CFG_PREFIX "/with_ll/bottom_cut");
	cfg_front_cut_  = config->get_float(CFG_PREFIX "/with_ll/front_cut");
	cfg_back_cut_   = config->get_float(CFG_PREFIX "/with_ll/back_cut");

	cfg_shelf_left_cut_no_ll_   = config->get_float(CFG_PREFIX "/shelf/without_ll/left_cut");
	cfg_shelf_right_cut_no_ll_  = config->get_float(CFG_PREFIX "/shelf/without_ll/right_cut");
	cfg_shelf_top_cut_no_ll_    = config->get_float(CFG_PREFIX "/shelf/without_ll/top_cut");
	cfg_shelf_bottom_cut_no_ll_ = config->get_float(CFG_PREFIX "/shelf/without_ll/bottom_cut");
	cfg_shelf_front_cut_no_ll_  = config->get_float(CFG_PREFIX "/shelf/without_ll/front_cut");
	cfg_shelf_back_cut_no_ll_   = config->get_float(CFG_PREFIX "/shelf/without_ll/back_cut");

	cfg_shelf_left_cut_   = config->get_float(CFG_PREFIX "/shelf/with_ll/left_cut");
	cfg_shelf_right_cut_  = config->get_float(CFG_PREFIX "/shelf/with_ll/right_cut");
	cfg_shelf_top_cut_    = config->get_float(CFG_PREFIX "/shelf/with_ll/top_cut");
	cfg_shelf_bottom_cut_ = config->get_float(CFG_PREFIX "/shelf/with_ll/bottom_cut");
	cfg_shelf_front_cut_  = config->get_float(CFG_PREFIX "/shelf/with_ll/front_cut");
	cfg_shelf_back_cut_   = config->get_float(CFG_PREFIX "/shelf/with_ll/back_cut");

	cfg_shelf_left_off_   = config->get_float(CFG_PREFIX "/shelf/left_off");
	cfg_shelf_middle_off_ = config->get_float(CFG_PREFIX "/shelf/middle_off");
	cfg_shelf_right_off_  = config->get_float(CFG_PREFIX "/shelf/right_off");

	cfg_voxel_grid_leaf_size_ = config->get_float(CFG_PREFIX "/voxel_grid/leaf_size");

	cfg_bb_realsense_switch_name_ =
	  config->get_string_or_default(CFG_PREFIX "/realsense_switch", "realsense");
	wait_time_ = Time(double(config->get_float_or_default(CFG_PREFIX "/realsense_wait_time", 1.0f)));

	// laserline bearing threshold
	cfg_ll_bearing_thresh_ = config->get_float(CFG_PREFIX "/ll/bearing_threshold");

	// Load reference pcl for shelf, input belt (with cone for input -> output
	// machines, without cone for output <- -> output machines), output belt
	// (without cone) and slide
	for (int i = ConveyorPoseInterface::DEFAULT_TYPE;
	     i != ConveyorPoseInterface::LAST_MPS_TYPE_ELEMENT;
	     i++) {
		ConveyorPoseInterface::MPS_TYPE mps_type = static_cast<ConveyorPoseInterface::MPS_TYPE>(i);
		for (int j = ConveyorPoseInterface::DEFAULT_TARGET;
		     j != ConveyorPoseInterface::LAST_MPS_TARGET_ELEMENT;
		     j++) {
			ConveyorPoseInterface::MPS_TARGET mps_target =
			  static_cast<ConveyorPoseInterface::MPS_TARGET>(j);

			type_target_to_path_[{mps_type, mps_target}] = get_model_path(bb_pose_, mps_type, mps_target);
		}
	}

	// always use the output_conveyor model for Storage station and Base station
	type_target_to_path_[{ConveyorPoseInterface::BASE_STATION,
	                      ConveyorPoseInterface::INPUT_CONVEYOR}] =
	  type_target_to_path_[{ConveyorPoseInterface::BASE_STATION,
	                        ConveyorPoseInterface::OUTPUT_CONVEYOR}];
	type_target_to_path_[{ConveyorPoseInterface::STORAGE_STATION,
	                      ConveyorPoseInterface::INPUT_CONVEYOR}] =
	  type_target_to_path_[{ConveyorPoseInterface::STORAGE_STATION,
	                        ConveyorPoseInterface::OUTPUT_CONVEYOR}];

	trimmed_scene_.reset(new Cloud());

	cfg_model_origin_frame_ = config->get_string(CFG_PREFIX "/model_origin_frame");
	cfg_record_model_       = config->get_bool_or_default(CFG_PREFIX "/record_model", false);

	// if recording is set, a pointcloud will be written to the the
	// cfg_record_path_ else load pcd file for every station and calculate model
	// with normals

	if (cfg_record_model_) {
		std::string reference_station =
		  config->get_string_or_default(CFG_PREFIX "/record_path", "recorded_file.pcd");
		cfg_record_path_            = CONFDIR "/" + reference_station;
		std::string new_record_path = cfg_record_path_;

		bool   exists;
		FILE * tmp;
		size_t count = 0;

		do {
			exists = false;
			tmp    = ::fopen(new_record_path.c_str(), "r");
			if (tmp) {
				exists = true;
				::fclose(tmp);
				if (cfg_record_path_.substr(cfg_record_path_.length() - 4) == ".pcd")
					new_record_path = cfg_record_path_.substr(0, cfg_record_path_.length() - 4)
					                  + std::to_string(count++) + ".pcd";
				else
					new_record_path = cfg_record_path_ + std::to_string(count++);
			}
		} while (exists);

		cfg_record_path_ = new_record_path;

		logger->log_info(name(), "Writing point cloud to %s", cfg_record_path_.c_str());
		model_.reset(new Cloud());
	} else {
		// Loading PCD file and calculation of model with normals for ALL! stations
		for (const auto &pair : type_target_to_path_) {
			CloudPtr model(new Cloud());
			if (pcl::io::loadPCDFile(pair.second, *model) < 0)
				throw fawkes::CouldNotOpenFileException(pair.second.c_str());

			type_target_to_model_.insert({pair.first, model});
		}

		model_ = type_target_to_model_[{ConveyorPoseInterface::DEFAULT_TYPE,
		                                ConveyorPoseInterface::DEFAULT_TARGET}];
	}

	cloud_in_registered_ = false;

	cloud_out_raw_     = new Cloud();
	cloud_out_trimmed_ = new Cloud();
	cloud_out_model_   = new Cloud();
	pcl_manager->add_pointcloud(cloud_out_raw_name_.c_str(), cloud_out_raw_);
	pcl_manager->add_pointcloud(cloud_out_trimmed_name_.c_str(), cloud_out_trimmed_);
	pcl_manager->add_pointcloud("model", cloud_out_model_);

	for (std::string ll : laserlines_names_) {
		laserlines_.push_back(blackboard->open_for_reading<fawkes::LaserLineInterface>(ll.c_str()));
	}

	realsense_switch_ =
	  blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());

	if (cfg_debug_mode_) {
		recognition_thread_->enable();
		switch (cfg_force_shelf_) {
		case 0: current_mps_target_ = fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_LEFT; break;
		case 1: current_mps_target_ = fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_MIDDLE; break;
		case 2: current_mps_target_ = fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_RIGHT; break;
		default: break;
		}
	}
}

void
ConveyorPoseThread::finalize()
{
	pcl_manager->remove_pointcloud(cloud_out_raw_name_.c_str());
	pcl_manager->remove_pointcloud(cloud_out_trimmed_name_.c_str());
	pcl_manager->remove_pointcloud("model");
	blackboard->close(bb_init_guess_pose_);
	blackboard->close(realsense_switch_);
	blackboard->close(bb_pose_);
}

void
ConveyorPoseThread::loop()
{
	//-- skip processing if camera is not enabled
	realsense_switch_->read();

	// Check for Messages in ConveyorPoseInterface and update information if
	// needed
	while (!bb_pose_->msgq_empty()) {
		if (bb_pose_->msgq_first_is<ConveyorPoseInterface::RunICPMessage>()) {
			// Update station related information
			logger->log_info(name(), "Received RunICPMessage");
			ConveyorPoseInterface::RunICPMessage *msg =
			  bb_pose_->msgq_first<ConveyorPoseInterface::RunICPMessage>();
			update_station_information(*msg);

			result_pose_.release();

			best_laser_line_    = nullptr;
			have_initial_guess_ = false;

			// Schedule restart of recognition thread
			recognition_thread_->retries_ = 0;
			recognition_thread_->schedule_restart();

			if (!cfg_debug_mode_ && !cfg_record_model_)
				// Set new timeout on incoming message!
				initial_guess_deadline_.reset(new Time(clock->now() + cfg_external_timeout_));
		} else if (bb_pose_->msgq_first_is<ConveyorPoseInterface::StopICPMessage>()) {
			recognition_thread_->disable();
			if (!have_initial_guess_)
				logger->log_error(name(), "Stopped without ever getting an initial guess");
		} else {
			logger->log_warn(name(), "Unknown message received");
		}
		bb_pose_->msgq_pop();
	}

	if (need_to_wait()) {
		logger->log_debug(name(),
		                  "Waiting for %s for %f sec, still %f sec remaining",
		                  cfg_bb_realsense_switch_name_.c_str(),
		                  wait_time_.in_sec(),
		                  (wait_start_ + wait_time_ - Time()).in_sec());
		return;
	}

	if (!cfg_record_model_ && !recognition_thread_->enabled())
		return;

	if (!realsense_switch_->is_enabled()) {
		logger->log_warn(name(), "Waiting for RealSense camera to be enabled");
		return;
	}

	if (update_input_cloud() && get_initial_guess()) {
		have_initial_guess_ = true;

		CloudPtr cloud_in(new Cloud(**cloud_in_));

		size_t   in_size  = cloud_in->points.size();
		CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
		size_t   out_size = cloud_vg->points.size();
		if (in_size == out_size) {
			logger->log_error(name(), "Voxel Grid failed, skipping loop!");
			return;
		}

		trimmed_scene_ = cloud_trim(cloud_vg);

		cloud_publish(cloud_in, cloud_out_raw_);
		cloud_publish(trimmed_scene_, cloud_out_trimmed_);

		if (cfg_record_model_) {
			record_model();
			cloud_publish(model_, cloud_out_model_);

			tf::Stamped<tf::Pose> initial_pose_cam;
			try {
				tf_listener->transform_pose(trimmed_scene_->header.frame_id,
				                            tf::Stamped<tf::Pose>(initial_guess_odom_,
				                                                  Time(0, 0),
				                                                  initial_guess_odom_.frame_id),
				                            initial_pose_cam);
			} catch (tf::TransformException &e) {
				logger->log_error(name(), e);
				return;
			}
			tf_publisher->send_transform(tf::StampedTransform(initial_pose_cam,
			                                                  initial_pose_cam.stamp,
			                                                  initial_pose_cam.frame_id,
			                                                  "conveyor_pose_initial_guess"));
		} else {
			{
				MutexLocker locked{&bb_mutex_};
				try {
					if (result_pose_) {
						pose_write();
						pose_publish_tf(*result_pose_);
						result_pose_.reset();
					}
				} catch (std::exception &e) {
					logger->log_error(name(), "Unexpected exception: %s", e.what());
				}
			}

			{
				MutexLocker locked{&cloud_mutex_};

				scene_ = trimmed_scene_;
				syncpoint_ready_for_icp_->emit(name());
			}
		} // ! cfg_record_model_
	}   // update_input_cloud()
}

bool
ConveyorPoseThread::initial_guess_deadline_reached()
{
	return initial_guess_deadline_ && clock->now() > *initial_guess_deadline_;
}

bool
ConveyorPoseThread::get_initial_guess()
{
	bb_init_guess_pose_->read();
	fawkes::Time last_external = *bb_init_guess_pose_->timestamp();

	// Definition of PCL timestamp: Epoch + usec:
	fawkes::Time last_pc = fawkes::Time(0, 0) + static_cast<long int>(input_pc_header_.stamp);

	LaserLineInterface *ll = laserline_get_best_fit();
	if (ll)
		best_laser_line_ = ll;

	if (std::abs(last_pc - &last_external) < cfg_max_timediff_external_pc_
	    && bb_init_guess_pose_->visibility_history() > 0) {
		//-- try to aquire external initial guess (e.g. conveyor_plane)
		fawkes::MutexLocker locked(&cloud_mutex_);

		try {
			if (set_external_initial_tf(initial_guess_odom_))
				return true;

		} catch (fawkes::tf::TransformException &ex) {
			logger->log_warn(name(),
			                 "External initial guess could not be transformed to odom-frame: %s",
			                 ex.what_no_backtrace());
		}
	}

	if (best_laser_line_ && (!initial_guess_deadline_ || initial_guess_deadline_reached())) {
		//-- try to aquire initial guess from laser_line as fallback
		const Time *laserline_time = best_laser_line_->timestamp();
		if (std::abs(last_pc - laserline_time) < cfg_max_timediff_external_pc_ && best_laser_line_) {
			fawkes::MutexLocker locked(&cloud_mutex_);
			try {
				if (set_laserline_initial_tf(initial_guess_odom_)) {
					initial_guess_deadline_ = nullptr;
					return true;
				}

			} catch (fawkes::tf::TransformException &ex) {
				logger->log_warn(name(),
				                 "Laserline initial guess could not be transformed to "
				                 "odom-frame: %s",
				                 ex.what_no_backtrace());
			}
		}
	}

	return false;
}

bool
ConveyorPoseThread::set_external_initial_tf(fawkes::tf::Stamped<fawkes::tf::Pose> &out_guess)
{
	//-- try to set read pose to initial guess, iff it was valid
	bb_init_guess_pose_->read();
	//-- compose translational part
	btVector3 init_origin;
	double *  blackboard_origin = bb_init_guess_pose_->translation();
	init_origin.setX(static_cast<btScalar>(blackboard_origin[0]));
	init_origin.setY(static_cast<btScalar>(blackboard_origin[1]));
	init_origin.setZ(static_cast<btScalar>(blackboard_origin[2]));

	//-- compose orientation
	btMatrix3x3  init_basis;
	double *     blackboard_orientation = bb_init_guess_pose_->rotation();
	btQuaternion init_orientation(static_cast<btScalar>(blackboard_orientation[0]) //-- x
	                              ,
	                              static_cast<btScalar>(blackboard_orientation[1]) //-- y
	                              ,
	                              static_cast<btScalar>(blackboard_orientation[2]) //-- z
	                              ,
	                              static_cast<btScalar>(blackboard_orientation[3])); //-- w

	init_basis.setRotation(init_orientation);

	//-- compose pose in original frame
	tf::Stamped<tf::Pose> pose_orig_frame;
	pose_orig_frame.setOrigin(init_origin);
	pose_orig_frame.setBasis(init_basis);
	pose_orig_frame.frame_id = bb_init_guess_pose_->frame();

	//-- transform external pose to odom frame (throws an exception handled at the
	// calling code)
	fawkes::tf::Stamped<fawkes::tf::Pose> pose_orig_frame_transformed;
	tf_listener->transform_pose("odom",
	                            tf::Stamped<tf::Pose>(pose_orig_frame,
	                                                  Time(0, 0),
	                                                  pose_orig_frame.frame_id),
	                            pose_orig_frame_transformed);

	//-- check if initial guess would get invalid is invalid
	const btVector3 &   translation = pose_orig_frame_transformed.getOrigin();
	const btQuaternion &orientation = pose_orig_frame_transformed.getRotation();
	if (std::isnan(translation.getX()) || std::isinf(translation.getX())
	    || std::isnan(translation.getY()) || std::isinf(translation.getY())
	    || std::isnan(translation.getZ()) || std::isinf(translation.getZ())
	    || std::isnan(orientation.getX()) || std::isinf(orientation.getX())
	    || std::isnan(orientation.getY()) || std::isinf(orientation.getY())
	    || std::isnan(orientation.getZ()) || std::isinf(orientation.getZ())
	    || std::isnan(orientation.getW()) || std::isinf(orientation.getW())
	    || std::abs(orientation.length() - btScalar(1.)) > std::numeric_limits<btScalar>::epsilon()) {
		logger->log_warn(name(),
		                 "External initial_guess_odom invalid [||R|| = %.10e, t = (%f, %f, %f)]",
		                 static_cast<double>(orientation.length()),
		                 static_cast<double>(translation.getX()),
		                 static_cast<double>(translation.getY()),
		                 static_cast<double>(translation.getZ()));
		return false;
	}

	//-- set the final init guess only the all steps success
	out_guess = pose_orig_frame_transformed;

	return true;
}

bool
ConveyorPoseThread::set_laserline_initial_tf(fawkes::tf::Stamped<fawkes::tf::Pose> &out_guess)
{
	tf::Stamped<tf::Pose> initial_guess;

	// Vector from end point 1 to end point 2
	if (!tf_listener->transform_origin(best_laser_line_->end_point_frame_2(),
	                                   best_laser_line_->end_point_frame_1(),
	                                   initial_guess,
	                                   Time(0, 0))) {
		logger->log_error(name(),
		                  "Failed to transform from %s to %s",
		                  best_laser_line_->end_point_frame_2(),
		                  best_laser_line_->end_point_frame_1());
		return false;
	}

	// Halve that to get line center
	initial_guess.setOrigin(initial_guess.getOrigin() / 2);

	// Add distance from center to mps_target
	initial_guess.setOrigin(initial_guess.getOrigin()
	                        + tf::Vector3{double(cfg_target_hint_[current_mps_target_][X_DIR]),
	                                      double(cfg_target_hint_[current_mps_target_][Y_DIR]),
	                                      double(cfg_target_hint_[current_mps_target_][Z_DIR])});

	if (current_mps_target_ == ConveyorPoseInterface::MPS_TARGET::INPUT_CONVEYOR) {
		// Add distance offset for station type
		initial_guess.setOrigin(initial_guess.getOrigin()
		                        + tf::Vector3{double(cfg_type_offset_[current_mps_type_][X_DIR]),
		                                      double(cfg_type_offset_[current_mps_type_][Y_DIR]),
		                                      double(cfg_type_offset_[current_mps_type_][Z_DIR])});
	} else if (current_mps_target_ == ConveyorPoseInterface::MPS_TARGET::OUTPUT_CONVEYOR) {
		// Invert Y axis of the offset
		initial_guess.setOrigin(initial_guess.getOrigin()
		                        + tf::Vector3{double(cfg_type_offset_[current_mps_type_][X_DIR]),
		                                      double(-cfg_type_offset_[current_mps_type_][Y_DIR]),
		                                      double(cfg_type_offset_[current_mps_type_][Z_DIR])});
	}

	initial_guess.setRotation(tf::Quaternion(tf::Vector3(0, 1, 0), -M_PI / 2)
	                          * tf::Quaternion(tf::Vector3(0, 0, 1), -M_PI / 2));

	// Transform with Time(0,0) to just get the latest transform (throws an
	// exception handled at the calling code)
	tf::Stamped<tf::Pose> initial_guess_transdormed;
	tf_listener->transform_pose("odom",
	                            tf::Stamped<tf::Pose>(initial_guess,
	                                                  Time(0, 0),
	                                                  initial_guess.frame_id),
	                            initial_guess_transdormed);

	//-- check if initial guess would get invalid is invalid
	const btVector3 &   translation = initial_guess_transdormed.getOrigin();
	const btQuaternion &orientation = initial_guess_transdormed.getRotation();
	if (std::isnan(translation.getX()) || std::isinf(translation.getX())
	    || std::isnan(translation.getY()) || std::isinf(translation.getY())
	    || std::isnan(translation.getZ()) || std::isinf(translation.getZ())
	    || std::abs(orientation.length() - btScalar(1.)) > std::numeric_limits<btScalar>::epsilon()) {
		logger->log_warn(name(),
		                 "Laserline initial_guess_odom invalid [||R|| = %.10e, t = "
		                 "(%f, %f, %f)]",
		                 static_cast<double>(orientation.length()),
		                 static_cast<double>(translation.getX()),
		                 static_cast<double>(translation.getY()),
		                 static_cast<double>(translation.getZ()));
		return false;
	}

	//-- set the final init guess only the all steps success
	out_guess = initial_guess_transdormed;

	return true;
}

void
ConveyorPoseThread::record_model()
{
	// Transform model
	tf::Stamped<tf::Pose> pose_cam;
	try {
		tf_listener->transform_origin(cloud_in_->header.frame_id, cfg_model_origin_frame_, pose_cam);
		Eigen::Matrix4f tf_to_cam = pose_to_eigen(pose_cam);
		pcl::transformPointCloud(*trimmed_scene_, *model_, tf_to_cam);

		// Overwrite and atomically rename model so it can be copied at any time
		int rv = pcl::io::savePCDFileASCII(cfg_record_path_, *model_);
		if (rv)
			logger->log_error(name(), "Error %d saving point cloud to %s", rv, cfg_record_path_.c_str());
		else
			::rename((cfg_record_path_ + ".pcd").c_str(), cfg_record_path_.c_str());
	} catch (pcl::IOException &e) {
		logger->log_error(name(),
		                  "Exception saving point cloud to %s: %s",
		                  cfg_record_path_.c_str(),
		                  e.what());
	} catch (tf::TransformException &e) {
		logger->log_error(name(), "Exception recording point cloud: %s", e.what());
	}
}

void
ConveyorPoseThread::update_station_information(ConveyorPoseInterface::RunICPMessage &msg)
{
	ConveyorPoseInterface::MPS_TYPE   mps_type   = msg.mps_type_to_set();
	ConveyorPoseInterface::MPS_TARGET mps_target = msg.mps_target_to_set();

	auto map_it = type_target_to_model_.find({mps_type, mps_target});
	if (map_it == type_target_to_model_.end())
		logger->log_error(name(), "Invalid station type or target: %i,%i", mps_type, mps_target);
	else {
		logger->log_info(name(),
		                 "Setting Station to %s, %s",
		                 bb_pose_->enum_tostring("MPS_TYPE", mps_type),
		                 bb_pose_->enum_tostring("MPS_TARGET", mps_target));

		MutexLocker locked2(&bb_mutex_);

		model_              = map_it->second;
		current_mps_type_   = mps_type;
		current_mps_target_ = mps_target;

		bb_pose_->set_current_mps_type(mps_type);
		bb_pose_->set_current_mps_target(mps_target);
		result_fitness_ = std::numeric_limits<double>::min();
		bb_pose_->set_euclidean_fitness(result_fitness_);
		bb_pose_->set_msgid(msg.id());
		bb_pose_->set_busy(true);
		bb_pose_->write();
	}
}

bool
ConveyorPoseThread::update_input_cloud()
{
	if (pcl_manager->exists_pointcloud(cloud_in_name_.c_str())) {
		if (!cloud_in_registered_) { // do I already have this pc
			cloud_in_ = pcl_manager->get_pointcloud<Point>(cloud_in_name_.c_str());
			if (cloud_in_->points.size() > 0) {
				cloud_in_registered_ = true;
			}
		}

		unsigned long time_old = input_pc_header_.stamp;
		input_pc_header_       = cloud_in_->header;

		return time_old != input_pc_header_.stamp; // true, if there is a new cloud, false otherwise

	} else {
		logger->log_debug(name(), "can't get pointcloud %s", cloud_in_name_.c_str());
		cloud_in_registered_ = false;
		return false;
	}
}

fawkes::LaserLineInterface *
ConveyorPoseThread::laserline_get_best_fit()
{
	fawkes::LaserLineInterface *best_fit  = nullptr;
	double                      min_dist2 = std::numeric_limits<double>::max();

	// get best line
	for (fawkes::LaserLineInterface *ll : laserlines_) {
		ll->read();
		if (!ll->has_writer())
			continue;
		if (ll->visibility_history() <= 2)
			continue;
		if (fabs(ll->bearing()) > cfg_ll_bearing_thresh_)
			continue;

		tf::Stamped<tf::Pose> center_ep;
		if (!tf_listener->transform_origin(
		      ll->end_point_frame_2(), ll->end_point_frame_1(), center_ep, Time(0, 0)))
			continue;

		center_ep.setOrigin(center_ep.getOrigin() / 2);
		// center_ep is is in the endpoint frame

		tf::Stamped<tf::Pose> center;
		try {
			// transform it to the laser frame
			tf_listener->transform_pose(ll->frame_id(), center_ep, center);
		} catch (tf::TransformException &) {
			continue;
		}

		if (center.getOrigin().length2() < min_dist2) {
			best_fit  = ll;
			min_dist2 = center.getOrigin().length2();
		}
	}

	return best_fit;
}

ConveyorPoseThread::CloudPtr
ConveyorPoseThread::cloud_voxel_grid(ConveyorPoseThread::CloudPtr in)
{
	float                                    ls = cfg_voxel_grid_leaf_size_;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
	CloudPtr                                 out(new Cloud);
	vg.setInputCloud(in);
	// logger->log_debug(name(), "voxel leaf size is %f", ls);
	vg.setLeafSize(ls, ls, ls);
	vg.filter(*out);
	out->header = in->header;
	return out;
}

void
ConveyorPoseThread::cloud_publish(ConveyorPoseThread::CloudPtr cloud_in,
                                  fawkes::RefPtr<Cloud>        cloud_out)
{
	**cloud_out       = *cloud_in;
	cloud_out->header = input_pc_header_;
}

void
ConveyorPoseThread::pose_write()
{
	if (!result_pose_) {
		logger->log_error(name(), "BUG: calling pose_write() when result_pose_ is unset!");
		return;
	}
	bb_pose_->set_translation(0, result_pose_->getOrigin().getX());
	bb_pose_->set_translation(1, result_pose_->getOrigin().getY());
	bb_pose_->set_translation(2, result_pose_->getOrigin().getZ());
	bb_pose_->set_rotation(0, result_pose_->getRotation().getX());
	bb_pose_->set_rotation(1, result_pose_->getRotation().getY());
	bb_pose_->set_rotation(2, result_pose_->getRotation().getZ());
	bb_pose_->set_rotation(3, result_pose_->getRotation().getW());

	bb_pose_->set_frame(result_pose_->frame_id.c_str());
	bb_pose_->set_euclidean_fitness(result_fitness_);
	long timestamp[2];
	result_pose_->stamp.get_timestamp(timestamp[0], timestamp[1]);
	bb_pose_->set_input_timestamp(timestamp);
	bb_pose_->write();
}

void
ConveyorPoseThread::pose_publish_tf(const tf::Stamped<tf::Pose> &pose)
{
	// transform data into gripper frame (this is better for later use)
	tf::Stamped<tf::Pose> tf_pose_gripper;
	tf_listener->transform_pose("gripper", pose, tf_pose_gripper);

	// publish the transform from the gripper to the conveyor
	tf::Transform        transform(tf_pose_gripper.getRotation(), tf_pose_gripper.getOrigin());
	tf::StampedTransform stamped_transform(transform,
	                                       tf_pose_gripper.stamp,
	                                       tf_pose_gripper.frame_id,
	                                       conveyor_frame_id_);
	tf_publisher->send_transform(stamped_transform);
}

void
ConveyorPoseThread::start_waiting()
{
	wait_start_ = Time();
}

bool
ConveyorPoseThread::need_to_wait()
{
	return Time() < wait_start_ + wait_time_;
}

void
ConveyorPoseThread::set_cg_thread(RecognitionThread *cg_thread)
{
	recognition_thread_ = cg_thread;
}

void
ConveyorPoseThread::config_value_erased(const char *)
{
}

void
ConveyorPoseThread::config_tag_changed(const char *)
{
}

void
ConveyorPoseThread::config_comment_changed(const Configuration::ValueIterator *)
{
}

void
ConveyorPoseThread::config_value_changed(const Configuration::ValueIterator *v)
{
	if (v->valid()) {
		std::string path       = v->path();
		std::string sufx       = path.substr(strlen(CFG_PREFIX));
		std::string sub_prefix = sufx.substr(0, sufx.substr(1).find("/") + 1);
		std::string full_pfx   = CFG_PREFIX + sub_prefix;
		std::string opt        = path.substr(full_pfx.length());

		if (sub_prefix == "/without_ll") {
			if (opt == "/left_cut")
				change_val(opt, cfg_left_cut_no_ll_, v->get_float());
			else if (opt == "/right_cut")
				change_val(opt, cfg_right_cut_no_ll_, v->get_float());
			else if (opt == "/top_cut")
				change_val(opt, cfg_top_cut_no_ll_, v->get_float());
			else if (opt == "/bottom_cut")
				change_val(opt, cfg_bottom_cut_no_ll_, v->get_float());
			else if (opt == "/front_cut")
				change_val(opt, cfg_front_cut_no_ll_, v->get_float());
			else if (opt == "/back_cut")
				change_val(opt, cfg_back_cut_no_ll_, v->get_float());
		} else if (sub_prefix == "") {
			if (opt == "/debug")
				change_val(opt, cfg_debug_mode_, v->get_bool());
		} else if (sub_prefix == "/with_ll") {
			if (opt == "/left_cut")
				change_val(opt, cfg_left_cut_, v->get_float());
			else if (opt == "/right_cut")
				change_val(opt, cfg_right_cut_, v->get_float());
			else if (opt == "/top_cut")
				change_val(opt, cfg_top_cut_, v->get_float());
			else if (opt == "/bottom_cut")
				change_val(opt, cfg_bottom_cut_, v->get_float());
			else if (opt == "/front_cut")
				change_val(opt, cfg_front_cut_, v->get_float());
			else if (opt == "/back_cut")
				change_val(opt, cfg_back_cut_, v->get_float());
		} else if (sub_prefix == "/shelf") {
			if (opt == "/without_ll/left_cut")
				change_val(opt, cfg_shelf_left_cut_no_ll_, v->get_float());
			else if (opt == "/without_ll/right_cut")
				change_val(opt, cfg_shelf_right_cut_no_ll_, v->get_float());
			else if (opt == "/without_ll/top_cut")
				change_val(opt, cfg_shelf_top_cut_no_ll_, v->get_float());
			else if (opt == "/without_ll/bottom_cut")
				change_val(opt, cfg_shelf_bottom_cut_no_ll_, v->get_float());
			else if (opt == "/without_ll/front_cut")
				change_val(opt, cfg_shelf_front_cut_no_ll_, v->get_float());
			else if (opt == "/without_ll/back_cut")
				change_val(opt, cfg_shelf_back_cut_no_ll_, v->get_float());
			else if (opt == "/with_ll/left_cut")
				change_val(opt, cfg_shelf_left_cut_, v->get_float());
			else if (opt == "/with_ll/right_cut")
				change_val(opt, cfg_shelf_right_cut_, v->get_float());
			else if (opt == "/with_ll/top_cut")
				change_val(opt, cfg_shelf_top_cut_, v->get_float());
			else if (opt == "/with_ll/bottom_cut")
				change_val(opt, cfg_shelf_bottom_cut_, v->get_float());
			else if (opt == "/with_ll/front_cut")
				change_val(opt, cfg_shelf_front_cut_, v->get_float());
			else if (opt == "/with_ll/back_cut")
				change_val(opt, cfg_shelf_back_cut_, v->get_float());
			else if (opt == "/left_off")
				change_val(opt, cfg_shelf_left_off_, v->get_float());
			else if (opt == "/middle_off")
				change_val(opt, cfg_shelf_middle_off_, v->get_float());
			else if (opt == "/right_off")
				change_val(opt, cfg_shelf_right_off_, v->get_float());
		} else if (sub_prefix == "/icp") {
			if (opt == "/max_correspondence_dist")
				change_val(opt, recognition_thread_->cfg_icp_max_corr_dist_, v->get_float());
			else if (opt == "/min_correspondence_dist")
				change_val(opt, recognition_thread_->cfg_icp_min_corr_dist_, double(v->get_float()));
			else if (opt == "/transformation_epsilon")
				change_val(opt, recognition_thread_->cfg_icp_tf_epsilon_, double(v->get_float()));
			else if (opt == "/refinement_factor")
				change_val(opt, recognition_thread_->cfg_icp_refinement_factor_, double(v->get_float()));
			else if (opt == "/max_iterations")
				change_val(opt, recognition_thread_->cfg_icp_max_iterations_, v->get_int());
			else if (opt == "/hv/penalty_threshold")
				change_val(opt, recognition_thread_->cfg_icp_hv_penalty_thresh_, v->get_float());
			else if (opt == "/hv/support_threshold")
				change_val(opt, recognition_thread_->cfg_icp_hv_support_thresh_, v->get_float());
			else if (opt == "/hv/inlier_threshold")
				change_val(opt, recognition_thread_->cfg_icp_hv_inlier_thresh_, v->get_float());
			else if (opt == "/hv/shelf_penalty_threshold")
				change_val(opt, recognition_thread_->cfg_icp_shelf_hv_penalty_thresh_, v->get_float());
			else if (opt == "/hv/shelf_support_threshold")
				change_val(opt, recognition_thread_->cfg_icp_shelf_hv_support_thresh_, v->get_float());
			else if (opt == "/hv/shelf_inlier_threshold")
				change_val(opt, recognition_thread_->cfg_icp_shelf_hv_inlier_thresh_, v->get_float());
			else if (opt == "/min_loops")
				change_val(opt, recognition_thread_->cfg_icp_min_loops_, v->get_uint());
			else if (opt == "/max_loops")
				change_val(opt, recognition_thread_->cfg_icp_max_loops_, v->get_uint());
			else if (opt == "/max_retries")
				change_val(opt, recognition_thread_->cfg_icp_max_retries_, v->get_uint());
			else if (opt == "/auto_restart")
				change_val(opt, recognition_thread_->cfg_icp_auto_restart_, v->get_bool());
			else if (opt == "/hint/input_conveyor/x") {
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][X_DIR],
				           v->get_float());
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][X_DIR],
				           v->get_float());
			} else if (opt == "/hint/input_conveyor/y") {
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][Y_DIR],
				           v->get_float());
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][Y_DIR],
				           -(v->get_float()));
			} else if (opt == "/hint/input_conveyor/z") {
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::INPUT_CONVEYOR][Z_DIR],
				           v->get_float());
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::OUTPUT_CONVEYOR][Z_DIR],
				           v->get_float());
			} else if (opt == "/hint/left_shelf/x")
				change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][X_DIR], v->get_float());
			else if (opt == "/hint/left_shelf/y")
				change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][Y_DIR], v->get_float());
			else if (opt == "/hint/left_shelf/z")
				change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SHELF_LEFT][Z_DIR], v->get_float());
			else if (opt == "/hint/middle_shelf/x")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][X_DIR],
				           v->get_float());
			else if (opt == "/hint/middle_shelf/y")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][Y_DIR],
				           v->get_float());
			else if (opt == "/hint/middle_shelf/z")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::SHELF_MIDDLE][Z_DIR],
				           v->get_float());
			else if (opt == "/hint/right_shelf/x")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][X_DIR],
				           v->get_float());
			else if (opt == "/hint/right_shelf/y")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][Y_DIR],
				           v->get_float());
			else if (opt == "/hint/right_shelf/z")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::SHELF_RIGHT][Z_DIR],
				           v->get_float());
			else if (opt == "/hint/slide/x")
				change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SLIDE][X_DIR], v->get_float());
			else if (opt == "/hint/slide/y")
				change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SLIDE][Y_DIR], v->get_float());
			else if (opt == "/hint/slide/z")
				change_val(opt, cfg_target_hint_[ConveyorPoseInterface::SLIDE][Z_DIR], v->get_float());
			else if (opt == "/hint/default/x")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET][X_DIR],
				           v->get_float());
			else if (opt == "/hint/default/y")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET][Y_DIR],
				           v->get_float());
			else if (opt == "/hint/default/z")
				change_val(opt,
				           cfg_target_hint_[ConveyorPoseInterface::DEFAULT_TARGET][Z_DIR],
				           v->get_float());

			else if (opt == "/conveyor_offset/base_station/x")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][X_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/base_station/y")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][Y_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/base_station/z")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::BASE_STATION][Z_DIR],
				           v->get_float());

			else if (opt == "/conveyor_offset/cap_station/x")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][X_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/cap_station/y")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][Y_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/cap_station/z")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::CAP_STATION][Z_DIR],
				           v->get_float());

			else if (opt == "/conveyor_offset/ring_station/x")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::RING_STATION][X_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/ring_station/y")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::RING_STATION][Y_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/ring_station/z")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::RING_STATION][Z_DIR],
				           v->get_float());

			else if (opt == "/conveyor_offset/delivery_station/x")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][X_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/delivery_station/y")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][Y_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/delivery_station/z")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::DELIVERY_STATION][Z_DIR],
				           v->get_float());

			else if (opt == "/conveyor_offset/storage_station/x")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][X_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/storage_station/y")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][Y_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/storage_station/z")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::STORAGE_STATION][Z_DIR],
				           v->get_float());

			else if (opt == "/conveyor_offset/default/x")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::DEFAULT_TYPE][X_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/default/y")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::DEFAULT_TYPE][Y_DIR],
				           v->get_float());
			else if (opt == "/conveyor_offset/default/z")
				change_val(opt,
				           cfg_type_offset_[ConveyorPoseInterface::DEFAULT_TYPE][Z_DIR],
				           v->get_float());
		} else if (sub_prefix == "/ll") {
			if (opt == "/bearing_threshold")
				change_val(opt, cfg_ll_bearing_thresh_, v->get_float());
		}
		if (recognition_thread_->enabled())
			recognition_thread_->schedule_restart();
	}
}

void
ConveyorPoseThread::bb_set_busy(bool busy)
{
	bb_pose_->set_busy(busy);
	bb_pose_->write();
}

float
ConveyorPoseThread::cloud_resolution() const
{
	return cfg_voxel_grid_leaf_size_;
}

Eigen::Matrix4f
pose_to_eigen(const fawkes::tf::Pose &pose)
{
	const tf::Matrix3x3 &rot   = pose.getBasis();
	const tf::Vector3 &  trans = pose.getOrigin();
	Eigen::Matrix4f      rv(Eigen::Matrix4f::Identity());
	rv.block<3, 3>(0, 0) << float(rot[0][0]), float(rot[0][1]), float(rot[0][2]), float(rot[1][0]),
	  float(rot[1][1]), float(rot[1][2]), float(rot[2][0]), float(rot[2][1]), float(rot[2][2]);
	rv.block<3, 1>(0, 3) << float(trans[X_DIR]), float(trans[Y_DIR]), float(trans[Z_DIR]);
	return rv;
}

fawkes::tf::Pose
eigen_to_pose(const Eigen::Matrix4f &m)
{
	fawkes::tf::Pose rv;
	rv.setOrigin({double(m(0, 3)), double(m(1, 3)), double(m(2, 3))});
	rv.setBasis({double(m(0, 0)),
	             double(m(0, 1)),
	             double(m(0, 2)),
	             double(m(1, 0)),
	             double(m(1, 1)),
	             double(m(1, 2)),
	             double(m(2, 0)),
	             double(m(2, 1)),
	             double(m(2, 2))});
	return rv;
}

/*
 * This function trims the scene such that the gripper is not visible anymore.
 * Further the scene is cut down to the center, so the amount of points to
 * correspond becomes smaller. Last but not least the background (e.g. cables)
 * is cut away, making the model more general.
 *
 * The trimming happens inside the frame of the conveyor cam frame.
 *
 * From the point of the cam the coordinate system is like:
 *             z
 *            /
 *          /
 *        /
 *       *-------x
 *       |
 *       |
 *       |
 *       y
 * */
ConveyorPoseThread::CloudPtr
ConveyorPoseThread::cloud_trim(ConveyorPoseThread::CloudPtr in)
{
	float x_min = -FLT_MAX, x_max = FLT_MAX, y_min = -FLT_MAX, y_max = FLT_MAX, z_min = -FLT_MAX,
	      z_max = FLT_MAX;

	{
		// get position of initial guess in conveyor cam frame
		// rotation is ignored, since rotation values are small
		tf::Stamped<tf::Pose> origin_pose;
		if (cfg_record_model_) {
			tf_listener->transform_origin(cfg_model_origin_frame_, in->header.frame_id, origin_pose);
		} else {
			tf_listener->transform_pose(in->header.frame_id,
			                            tf::Stamped<tf::Pose>(initial_guess_odom_,
			                                                  Time(0, 0),
			                                                  initial_guess_odom_.frame_id),
			                            origin_pose);
		}
		float x_ini = float(origin_pose.getOrigin()[X_DIR]),
		      y_ini = float(origin_pose.getOrigin()[Y_DIR]),
		      z_ini = float(origin_pose.getOrigin()[Z_DIR]);

		if (is_target_shelf()) { // using shelf cut values
			float x_min_temp = x_ini + (float)cfg_shelf_left_cut_,
			      x_max_temp = x_ini + (float)cfg_shelf_right_cut_;
			switch (current_mps_target_) {
			case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_LEFT:
				x_min_temp += (float)cfg_shelf_left_off_;
				x_max_temp += (float)cfg_shelf_left_off_;
				break;
			case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_MIDDLE:
				x_min_temp += (float)cfg_shelf_middle_off_;
				x_max_temp += (float)cfg_shelf_middle_off_;
				break;
			case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_RIGHT:
				x_min_temp += (float)cfg_shelf_right_off_;
				x_max_temp += (float)cfg_shelf_right_off_;
				break;
			default: // This should not happen
				break;
			}
			x_min = std::max(x_min_temp, x_min);
			x_max = std::min(x_max_temp, x_max);

			y_min = std::max(y_ini + (float)cfg_shelf_top_cut_, y_min);
			y_max = std::min(y_ini + (float)cfg_shelf_bottom_cut_, y_max);

			z_min = std::max(z_ini + (float)cfg_shelf_front_cut_, z_min);
			z_max = std::min(z_ini + (float)cfg_shelf_back_cut_, z_max);
		} else { // using general cut values
			x_min = std::max(x_ini + (float)cfg_left_cut_, x_min);
			x_max = std::min(x_ini + (float)cfg_right_cut_, x_max);

			y_min = std::max(y_ini + (float)cfg_top_cut_, y_min);
			y_max = std::min(y_ini + (float)cfg_bottom_cut_, y_max);

			z_min = std::max(z_ini + (float)cfg_front_cut_, z_min);
			z_max = std::min(z_ini + (float)cfg_back_cut_, z_max);
		}
	}

	CloudPtr out(new Cloud);
	for (Point p : *in) {
		if (p.x < x_max && p.x > x_min && p.y < y_max && p.y > y_min && p.z < z_max && p.z > z_min) {
			out->push_back(p);
		}
	}

	out->header = in->header;
	return out;
}

bool
ConveyorPoseThread::is_target_shelf()
{
	switch (current_mps_target_) {
	case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_LEFT:
	case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_MIDDLE:
	case fawkes::ConveyorPoseInterface::MPS_TARGET::SHELF_RIGHT: return true;
	default: return false;
	}
}
