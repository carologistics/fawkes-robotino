/***************************************************************************
 *  object_tracking.cpp - Thread tracks workpieces, conveyor belts, and slides
 *      using yolo while providing curresponding target frames used for visual
 *      servoing
 *
 *  Created: Tue Jan 25 18:25:15 2022
 *  Copyright  2022  Matteo Tschesche
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

#include "object_tracking_thread.h"

#include <aspect/logging.h>
#include <interfaces/ObjectTrackingInterface.h>
#include <navgraph/navgraph.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#include <boost/algorithm/string/predicate.hpp>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdio.h>

using namespace fawkes;
using namespace cv;
using namespace dnn;

/** @class ObjectTrackingThread "object_tracking_thread.h"
 * Thread Tracks Objects and Provides the Curresponding Target Frames
 * @author Matteo Tschesche
 */

/** Constructor. */
ObjectTrackingThread::ObjectTrackingThread()
: Thread("ObjectTrackingThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
  fawkes::TransformAspect(fawkes::TransformAspect::BOTH_DEFER_PUBLISHER)
{
}

void
ObjectTrackingThread::init()
{
	logger->log_info(name(), "Initializing Object Tracker");
	std::cout << "OpenCV version : " << CV_VERSION << std::endl;

	//read config values for computing expected position and target frames
	puck_size_   = config->get_float("plugins/object_tracking/puck_values/puck_size");
	puck_height_ = config->get_float("plugins/object_tracking/puck_values/puck_height");

	belt_height_ = config->get_float("plugins/object_tracking/belt_values/belt_height");
	belt_lenght_ = config->get_float("plugins/object_tracking/belt_values/belt_lenght");
	belt_size_   = config->get_float("plugins/object_tracking/belt_values/belt_size");

	slide_offset_side_ = config->get_float("plugins/object_tracking/slide_values/slide_offset_side");
	slide_height_      = config->get_float("plugins/object_tracking/slide_values/slide_height");

	left_shelf_offset_side_ =
	  config->get_float("plugins/object_tracking/shelf_values/left_shelf_offset_side");
	middle_shelf_offset_side_ =
	  config->get_float("plugins/object_tracking/shelf_values/middle_shelf_offset_side");
	right_shelf_offset_side_ =
	  config->get_float("plugins/object_tracking/shelf_values/right_shelf_offset_side");
	shelf_height_ = config->get_float("plugins/object_tracking/shelf_values/shelf_height");

	gripper_offset_pick_ =
	  config->get_float("plugins/object_tracking/target_frame_offsets/gripper_offset_pick");
	gripper_offset_put_ =
	  config->get_float("plugins/object_tracking/target_frame_offsets/gripper_offset_put");
	base_offset_ = config->get_float("plugins/object_tracking/target_frame_offsets/base_offset");

	//get camera params
	focal_length_     = 580;
	camera_width_     = config->get_int("plugins/object_tracking/camera_intrinsics/width");
	camera_height_    = config->get_int("plugins/object_tracking/camera_intrinsics/height");
	camera_ppx_       = config->get_float("plugins/object_tracking/camera_intrinsics/ppx");
	camera_ppy_       = config->get_float("plugins/object_tracking/camera_intrinsics/ppy");
	camera_fx_        = config->get_float("plugins/object_tracking/camera_intrinsics/fx");
	camera_fy_        = config->get_float("plugins/object_tracking/camera_intrinsics/fy");
	camera_model_     = config->get_int("plugins/object_tracking/camera_intrinsics/model");
	camera_coeffs_[0] = config->get_float("plugins/object_tracking/camera_intrinsics/k1");
	camera_coeffs_[1] = config->get_float("plugins/object_tracking/camera_intrinsics/k2");
	camera_coeffs_[2] = config->get_float("plugins/object_tracking/camera_intrinsics/p1");
	camera_coeffs_[3] = config->get_float("plugins/object_tracking/camera_intrinsics/p2");
	camera_coeffs_[4] = config->get_float("plugins/object_tracking/camera_intrinsics/k3");

	//get params for saved image usage
	use_saved_         = config->get_bool("plugins/object_tracking/saved/use_saved");
	image_path_        = this->config->get_string(("plugins/object_tracking/saved/image_path"));
	saved_object_type_ = static_cast<ObjectTrackingInterface::TARGET_OBJECT_TYPE>(
	  config->get_int("plugins/object_tracking/saved/saved_object_type"));

	rotate_image_        = config->get_bool("plugins/object_tracking/rotate_image");
	target_frame_        = config->get_string("plugins/object_tracking/target_frame");
	cam_frame_           = config->get_string("plugins/object_tracking/camera_frame");
	max_acceptable_dist_ = config->get_float("plugins/object_tracking/max_acceptable_dist");

	//needed for realsense 3d projection
	intrinsics_.width     = camera_width_;
	intrinsics_.height    = camera_height_;
	intrinsics_.ppx       = camera_ppx_;
	intrinsics_.ppy       = camera_ppy_;
	intrinsics_.fx        = camera_fx_;
	intrinsics_.fy        = camera_fy_;
	intrinsics_.model     = static_cast<rs2_distortion>(camera_model_);
	intrinsics_.coeffs[0] = camera_coeffs_[0];
	intrinsics_.coeffs[1] = camera_coeffs_[1];
	intrinsics_.coeffs[2] = camera_coeffs_[2];
	intrinsics_.coeffs[3] = camera_coeffs_[3];
	intrinsics_.coeffs[4] = camera_coeffs_[4];

	//set object params
	//               {Unset, Workpiece, Conveyor, Slide} //TODO: get rid of Unset, if element -1 should get called, print error
	object_widths_ = {0.0, 0.04, 0.03, 0.0585};

	//get NN params
	weights_path_  = this->config->get_string(("plugins/object_tracking/yolo/weights_path"));
	config_path_   = this->config->get_string(("plugins/object_tracking/yolo/config_path"));
	confThreshold_ = this->config->get_float(("plugins/object_tracking/yolo/confThreshold"));
	nmsThreshold_  = this->config->get_float(("plugins/object_tracking/yolo/nmsThreshold"));
	inpWidth_      = this->config->get_int(("plugins/object_tracking/yolo/width"));
	inpHeight_     = this->config->get_int(("plugins/object_tracking/yolo/height"));

	//set NN params
	scale_  = 0.00392; //to normalize inputs: 0.00392 * 255 = 1
	swapRB_ = true;

	//set up network
	net_ = readNet(weights_path_, config_path_);
	net_.setPreferableBackend(DNN_BACKEND_DEFAULT);
	net_.setPreferableTarget(DNN_TARGET_CPU);
	//get name of output layer
	outName_ = net_.getUnconnectedOutLayersNames();

	//set up weighted average filter
	//-------------------------------------------------------------------------
	filter_weights_[0] = 0.4;  // current response
	filter_weights_[1] = 0.3;  // last response
	filter_weights_[2] = 0.15; // 2. last response
	filter_weights_[3] = 0.1;  // 3. last response
	filter_weights_[4] = 0.05; // 4. last response
	//-------------------------------------------------------------------------

	filter_size_ = sizeof(filter_weights_) / sizeof(filter_weights_[0]);

	//normalize filter_weights
	float weights_sum = 0;
	for (size_t i = 0; i < filter_size_; i++) {
		weights_sum += filter_weights_[i];
	}
	for (size_t i = 0; i < filter_size_; i++) {
		filter_weights_[i] = filter_weights_[i] / weights_sum;
	}

	//open ObjectTrackingInterface for writing
	object_tracking_if_name_ = config->get_string("plugins/object_tracking/if_name");
	object_tracking_if_ =
	  blackboard->open_for_writing<ObjectTrackingInterface>(object_tracking_if_name_.c_str());

	object_tracking_if_->set_current_object_type(object_tracking_if_->DEFAULT_TYPE);
	object_tracking_if_->set_current_expected_mps(object_tracking_if_->DEFAULT_MPS);
	object_tracking_if_->set_current_expected_side(object_tracking_if_->DEFAULT_SIDE);
	object_tracking_if_->set_msgid(0);
	object_tracking_if_->set_detected(false);

	object_tracking_if_->write();

	//shared memory buffer--------------------

	// Image Buffer ID
	shm_id_ = config->get_string("plugins/object_tracking/buffer/shm_image_id");

	shm_id_res_          = config->get_string("plugins/object_tracking/buffer/shm_image_id_res");
	shm_buffer_results_  = new firevision::SharedMemoryImageBuffer(shm_id_res_.c_str(),
                                                                firevision::BGR,
                                                                camera_width_,
                                                                camera_height_);
	std::string frame_id = this->config->get_string("plugins/object_tracking/buffer/frame");
	shm_buffer_results_->set_frame_id(frame_id.c_str());

	//initialize publisher objects----------
	object_pos_frame_ = this->config->get_string("plugins/object_tracking/tf/object_pos_frame");
	weighted_object_pos_frame_ =
	  this->config->get_string("plugins/object_tracking/tf/weighted_object_pos_frame");

	tf_add_publisher(object_pos_frame_.c_str());
	object_pos_pub = tf_publishers[object_pos_frame_];

	tf_add_publisher(weighted_object_pos_frame_.c_str());
	weighted_object_pos_pub = tf_publishers[weighted_object_pos_frame_];
	//--------------------------------------

	name_it_    = 0;
	tracking_   = false;
	shm_active_ = false;
}

void
ObjectTrackingThread::loop()
{
	//handle incomming messages
	//-------------------------------------------------------------------------
	while (!object_tracking_if_->msgq_empty()) {
		if (object_tracking_if_->msgq_first_is<ObjectTrackingInterface::StartTrackingMessage>()) {
			logger->log_info(name(), "Received StartTrackingMessage");

			ObjectTrackingInterface::StartTrackingMessage *msg =
			  object_tracking_if_->msgq_first<ObjectTrackingInterface::StartTrackingMessage>();
			current_object_type_   = msg->object_type_to_set();
			current_expected_mps_  = msg->expected_mps_to_set();
			current_expected_side_ = msg->expected_side_to_set();

			//compute expected object position based on navgraph
			compute_expected_position();

			//transform from map to target (depends on target_frame in object_tracking.yaml)
			fawkes::tf::Stamped<fawkes::tf::Point> exp_pos_target;
			tf_listener->transform_point(target_frame_, exp_pos_, exp_pos_target);

			//initialize past responses for weighted average using expected positions
			past_responses_.clear();
			for (size_t i = 0; i < filter_size_ - 1; i++) {
				past_responses_.push_front(exp_pos_target);
			}

			//initialize weighted average as expected position
			weighted_object_pos_target_.stamp    = fawkes::Time(0.0);
			weighted_object_pos_target_.frame_id = target_frame_;
			weighted_object_pos_target_.setX(exp_pos_target.getX());
			weighted_object_pos_target_.setY(exp_pos_target.getY());
			weighted_object_pos_target_.setZ(exp_pos_target.getZ());

			//activate shared memory buffer
			if (!shm_active_)
				set_shm();

			//start interface
			tracking_      = true;
			msgid_         = 0;
			starting_time_ = fawkes::Time(clock);
			loop_count_    = 0;
			object_tracking_if_->set_current_object_type(current_object_type_);
			object_tracking_if_->set_current_expected_mps(current_expected_mps_);
			object_tracking_if_->set_current_expected_side(current_expected_side_);
		} else if (object_tracking_if_->msgq_first_is<ObjectTrackingInterface::StopTrackingMessage>()) {
			logger->log_info(name(), "Received StopTrackingMessage");

			tracking_ = false;
			msgid_    = 0;
			object_tracking_if_->set_msgid(msgid_);
			object_tracking_if_->set_detected(false);
			object_tracking_if_->set_current_object_type(ObjectTrackingInterface::DEFAULT_TYPE);
			object_tracking_if_->set_current_expected_mps(ObjectTrackingInterface::DEFAULT_MPS);
			object_tracking_if_->set_current_expected_side(ObjectTrackingInterface::DEFAULT_SIDE);

			object_tracking_if_->write();
		} else {
			logger->log_warn(name(), "Unknown message received");
		}
		object_tracking_if_->msgq_pop();
	}
	//-------------------------------------------------------------------------

	//check if tracking is active
	if (!use_saved_ && !tracking_)
		return;

	//get image
	//-------------------------------------------------------------------------

	//get all filenames in the given directory or the filename of the image path
	if (use_saved_ && filenames_.empty())
		glob(image_path_ + "*", filenames_);

	fawkes::Time start_time(clock);

	Mat          image;
	fawkes::Time capture_time;

	if (use_saved_) {
		current_object_type_ = saved_object_type_;

		bool found_image = false;
		if (name_it_ >= filenames_.size())
			return;

		while (name_it_ < filenames_.size() && !found_image) {
			//check if png or jpg file
			if (boost::algorithm::ends_with(filenames_[name_it_], ".png")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".PNG")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".jpg")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".JPG")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".jpeg")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".JPEG")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".jfif")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".JFIF")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".pjpeg")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".PJPEG")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".pjp")
			    || boost::algorithm::ends_with(filenames_[name_it_], ".PJP")) {
				image       = imread(filenames_[name_it_]);
				found_image = true;
			} else if (name_it_ + 1 >= filenames_.size()) {
				name_it_++;
				return;
			}
			name_it_++;
		}
	} else {
		//read from sharedMemoryBuffer and convert into Mat
		image = Mat(camera_height_, camera_width_, CV_8UC3, shm_buffer_->buffer()).clone();
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		capture_time = shm_buffer_->capture_time();
	}

	if (rotate_image_)
		rotate(image, image, ROTATE_180);
	//-------------------------------------------------------------------------

	//detect objects
	std::vector<std::array<float, 4>> out_boxes;
	fawkes::Time                      before_detect(clock);
	detect_objects(image, out_boxes);
	fawkes::Time after_detect(clock);

	//save results
	//std::string new_img_name = image_path_.insert(image_path_.find("."), "_results");
	//imwrite(new_img_name, image);

	//update results for saved images in webview
	if (use_saved_) {
		for (size_t i = 0; i < out_boxes.size(); ++i) {
			float pos[3];
			float wp_additional_height = 0;
			compute_3d_point_direct_yolo(out_boxes[i], 0.0, pos, wp_additional_height);

			//draw bounding box on the image
			cv::Rect rect_bb;
			convert_bb_yolo2rect(out_boxes[i], rect_bb);
			rectangle(image, rect_bb, Scalar(0, 0, 255), 2);

			//write 3d position under it
			std::stringstream sx;
			std::stringstream sy;
			std::stringstream sz;
			sx << std::fixed << std::setprecision(3) << pos[0];
			sy << std::fixed << std::setprecision(3) << pos[1];
			sz << std::fixed << std::setprecision(3) << pos[2];
			std::string pos_str = sx.str() + " " + sy.str() + " " + sz.str();
			//logger->log_info("cam_gripper: ", pos_str.c_str());

			cv::putText(image,
			            pos_str,
			            cv::Point(rect_bb.x, rect_bb.y + rect_bb.height + 23),
			            cv::FONT_HERSHEY_SIMPLEX,
			            0.85,
			            cv::Scalar(0, 0, 255),
			            2.5,
			            true);
		}
		//set resulting image in shared memory buffer
		firevision::convert(firevision::BGR,
		                    firevision::BGR,
		                    image.data,
		                    shm_buffer_results_->buffer(),
		                    camera_width_,
		                    camera_height_);

		//save results
		//std::string new_img_name = "/home/mtschesche/Pictures/realsense_sequence_3_norot_results/" + std::to_string(name_it_ -1) + ".jpg";
		//imwrite(new_img_name, image);

		fawkes::Time after_projection(clock);
		//logger->log_info("load image time ", std::to_string(before_detect - &start_time).c_str());
		//logger->log_info("detection time  ", std::to_string(after_detect - &before_detect).c_str());
		//logger->log_info("box time        ", std::to_string(after_projection - &after_detect).c_str());
		//logger->log_info("overall time    ", std::to_string(after_projection - &start_time).c_str());

		return;
	}

	// logger->log_info("expected pos", "frame: map");
	// logger->log_info("exp_pos_[0]: ", std::to_string(exp_pos_.getX()).c_str());
	// logger->log_info("exp_pos_[1]: ", std::to_string(exp_pos_.getY()).c_str());
	// logger->log_info("exp_pos_[2]: ", std::to_string(exp_pos_.getZ()).c_str());

	//get yaw difference between robot and mps
	//TODO: get mps angle through laser-lines?
	fawkes::tf::Quaternion mps_q = fawkes::tf::create_quaternion_from_yaw(double(mps_ori_));
	tf::Point              mps_pos(mps_x_, mps_y_, 0.0);
	fawkes::tf::Pose       mps_pose(mps_q, mps_pos);
	fawkes::tf::Stamped<fawkes::tf::Pose> mps_pose_map(mps_pose, capture_time, "map");
	fawkes::tf::Stamped<fawkes::tf::Pose> mps_pose_base;
	tf_listener->transform_pose("base_link", mps_pose_map, mps_pose_base);

	float mps_angle = fawkes::tf::get_yaw(mps_pose_base.getRotation());
	//logger->log_info("mps_yaw: ", std::to_string(mps_angle).c_str());
	if (current_expected_side_ != ObjectTrackingInterface::OUTPUT_CONVEYOR) {
		mps_angle += M_PI;
	}

	float cur_object_pos[3];
	Rect  closest_box;
	float additional_height = 0;
	//get 3d position of closest bounding box to last weighted average in cam_gripper frame
	fawkes::tf::Stamped<fawkes::tf::Point> expected_pos_cam;
	tf_listener->transform_point(cam_frame_, exp_pos_, expected_pos_cam);
	bool detected = closest_position(
	  out_boxes, expected_pos_cam, mps_angle, cur_object_pos, closest_box, additional_height);
	std::string                            pos_str;
	fawkes::tf::Stamped<fawkes::tf::Point> cur_object_pos_target;
	if (detected) {
		//draw bounding box
		rectangle(image, closest_box, Scalar(0, 255, 0), 2);

		//write 3d position in cam_gripper frame on the image
		std::stringstream sx;
		std::stringstream sy;
		std::stringstream sz;
		sx << std::fixed << std::setprecision(3) << cur_object_pos[0];
		sy << std::fixed << std::setprecision(3) << cur_object_pos[1];
		sz << std::fixed << std::setprecision(3) << cur_object_pos[2];
		pos_str = sx.str() + " " + sy.str() + " " + sz.str();

		// logger->log_info("cur obj", "frame: cam_gripper");
		// logger->log_info("cur_object_pos[0]: ", std::to_string(cur_object_pos[0]).c_str());
		// logger->log_info("cur_object_pos[1]: ", std::to_string(cur_object_pos[1]).c_str());
		// logger->log_info("cur_object_pos[2]: ", std::to_string(cur_object_pos[2]).c_str());

		//transform current response into target frame
		fawkes::tf::Stamped<fawkes::tf::Point> cur_object_pos_cam;
		cur_object_pos_cam.stamp    = capture_time;
		cur_object_pos_cam.frame_id = cam_frame_;
		cur_object_pos_cam.setX(cur_object_pos[0]);
		cur_object_pos_cam.setY(cur_object_pos[1]);
		cur_object_pos_cam.setZ(cur_object_pos[2]);
		tf_listener->transform_point(target_frame_, cur_object_pos_cam, cur_object_pos_target);

		//update object pos transform
		tf::Quaternion       q(0.0, 0.0, 0.0);
		tf::Vector3          v(cur_object_pos[0], cur_object_pos[1], cur_object_pos[2]);
		tf::Transform        tf_object_pos(q, v);
		tf::StampedTransform stf_object_pos(tf_object_pos, capture_time, cam_frame_, object_pos_frame_);
		object_pos_pub->send_transform(stf_object_pos);
	} else {
		//if object is not detected, use expected position instead
		pos_str = "X.XXX X.XXX X.XXX";

		//transform from map to target
		tf_listener->transform_point(target_frame_, exp_pos_, cur_object_pos_target);
	}
	cv::putText(image,
	            pos_str,
	            cv::Point(10, 470),
	            cv::FONT_HERSHEY_SIMPLEX,
	            1.5,
	            cv::Scalar(0, 255, 0),
	            3.5,
	            true);

	//set resulting image in shared memory buffer
	firevision::convert(firevision::BGR,
	                    firevision::BGR,
	                    image.data,
	                    shm_buffer_results_->buffer(),
	                    camera_width_,
	                    camera_height_);

	//save results
	//std::string new_img_name = "/tmp/yolo_results/" + std::to_string(name_it_) + ".jpg";
	//imwrite(new_img_name, image);
	//name_it_ ++;
	fawkes::Time after_projection(clock);

	//compute weighted average
	//-------------------------------------------------------------------------

	//logger->log_info("cur pos", "frame: odom");
	//logger->log_info("filter_weight: ", std::to_string(filter_weights_[0]).c_str());
	//logger->log_info("cur_object_pos_target[0]: ",
	//                 std::to_string(cur_object_pos_target.getX()).c_str());
	//logger->log_info("cur_object_pos_target[1]: ",
	//                 std::to_string(cur_object_pos_target.getY()).c_str());
	//logger->log_info("cur_object_pos_target[2]: ",
	//                 std::to_string(cur_object_pos_target.getZ()).c_str());

	//use weighted average to improve robustness of object position
	double weighted_object_pos[3];

	weighted_object_pos[0] = filter_weights_[0] * cur_object_pos_target.getX();
	weighted_object_pos[1] = filter_weights_[0] * cur_object_pos_target.getY();
	weighted_object_pos[2] = filter_weights_[0] * cur_object_pos_target.getZ();

	for (size_t i = 0; i < past_responses_.size(); i++) {
		//logger->log_info("past response", std::to_string(i).c_str());
		//logger->log_info("filter_weight: ", std::to_string(filter_weights_[1 + i]).c_str());
		//logger->log_info("past_responses_[0]: ", std::to_string(past_responses_[i].getX()).c_str());
		//logger->log_info("past_responses_[1]: ", std::to_string(past_responses_[i].getY()).c_str());
		//logger->log_info("past_responses_[2]: ", std::to_string(past_responses_[i].getZ()).c_str());
		//fawkes::tf::Stamped<fawkes::tf::Point> response_target;
		//tf_listener->transform_point(target_frame_, past_responses_[i], response_target);
		weighted_object_pos[0] += filter_weights_[1 + i] * past_responses_[i].getX();
		weighted_object_pos[1] += filter_weights_[1 + i] * past_responses_[i].getY();
		weighted_object_pos[2] += filter_weights_[1 + i] * past_responses_[i].getZ();
	}
	past_responses_.push_front(cur_object_pos_target);
	past_responses_.pop_back();

	//update weighted object pos transform
	tf::Quaternion       q(0.0, 0.0, 0.0);
	tf::Vector3          v(weighted_object_pos[0], weighted_object_pos[1], weighted_object_pos[2]);
	tf::Transform        tf_weighted_object_pos(q, v);
	tf::StampedTransform stf_weighted_object_pos(tf_weighted_object_pos,
	                                             capture_time,
	                                             target_frame_,
	                                             weighted_object_pos_frame_);
	weighted_object_pos_pub->send_transform(stf_weighted_object_pos);

	// logger->log_info("weighted pos", "frame: odom");
	// logger->log_info("weighted_object_pos[0]: ", std::to_string(weighted_object_pos[0]).c_str());
	// logger->log_info("weighted_object_pos[1]: ", std::to_string(weighted_object_pos[1]).c_str());
	// logger->log_info("weighted_object_pos[2]: ", std::to_string(weighted_object_pos[2]).c_str());

	//transform weighted average into base_link
	fawkes::tf::Stamped<fawkes::tf::Point> weighted_object_pos_base;
	weighted_object_pos_target_.stamp    = fawkes::Time(0.0);
	weighted_object_pos_target_.frame_id = target_frame_;
	weighted_object_pos_target_.setX(weighted_object_pos[0]);
	weighted_object_pos_target_.setY(weighted_object_pos[1]);
	weighted_object_pos_target_.setZ(weighted_object_pos[2]);
	tf_listener->transform_point("base_link", weighted_object_pos_target_, weighted_object_pos_base);
	//-------------------------------------------------------------------------

	//compute target frames
	//-------------------------------------------------------------------------
	//logger->log_info("weighted pos", "frame: base_link");
	//logger->log_info("weighted_object_pos_base[0]: ",
	//                 std::to_string(weighted_object_pos_base.getX()).c_str());
	//logger->log_info("weighted_object_pos_base[1]: ",
	//                 std::to_string(weighted_object_pos_base.getY()).c_str());
	//logger->log_info("weighted_object_pos_base[2]: ",
	//                 std::to_string(weighted_object_pos_base.getZ()).c_str());

	double gripper_target[3];
	double base_target[3];
	compute_target_frames(weighted_object_pos_base, mps_angle, gripper_target, base_target);
	// logger->log_info("target frames", "frame: base_link");
	// logger->log_info("gripper_target[0]: ", std::to_string(gripper_target[0]).c_str());
	// logger->log_info("gripper_target[1]: ", std::to_string(gripper_target[1]).c_str());
	// logger->log_info("gripper_target[2]: ", std::to_string(gripper_target[2]).c_str());
	// logger->log_info("base_target[0]: ", std::to_string(base_target[0]).c_str());
	// logger->log_info("base_target[1]: ", std::to_string(base_target[1]).c_str());
	// logger->log_info("base_target[2]: ", std::to_string(base_target[2]).c_str());

	//update interface
	object_tracking_if_->set_gripper_frame(0, gripper_target[0]);
	object_tracking_if_->set_gripper_frame(1, gripper_target[1]);
	object_tracking_if_->set_gripper_frame(2, gripper_target[2]);
	object_tracking_if_->set_base_frame(0, base_target[0]);
	object_tracking_if_->set_base_frame(1, base_target[1]);
	object_tracking_if_->set_base_frame(5, base_target[2]);

	msgid_++;
	object_tracking_if_->set_msgid(msgid_);
	object_tracking_if_->set_detected(detected);
	if (detected) {
		object_tracking_if_->set_additional_height(0, additional_height);
	}
	object_tracking_if_->write();

	fawkes::Time after_interface_update(clock);
	loop_count_++;
	//double average_loop = (after_interface_update - &starting_time_) / loop_count_;

	//logger->log_info("load image time ", std::to_string(before_detect - &start_time).c_str());
	//logger->log_info("detection time  ", std::to_string(after_detect - &before_detect).c_str());
	//logger->log_info("box time        ", std::to_string(after_projection - &after_detect).c_str());
	//logger->log_info("interface time  ", std::to_string(after_interface_update - &after_projection).c_str());
	//logger->log_info("overall time    ", std::to_string(after_interface_update - &start_time).c_str());
	//logger->log_info("loop count      ", std::to_string(loop_count_).c_str());
	//logger->log_info("average loop    ", std::to_string(average_loop).c_str());
}

void
ObjectTrackingThread::compute_expected_position()
{
	float exp_pos_map[3];

	if (current_object_type_ == ObjectTrackingInterface::DEFAULT_TYPE
	    || current_expected_mps_ == ObjectTrackingInterface::DEFAULT_MPS
	    || current_expected_side_ == ObjectTrackingInterface::DEFAULT_SIDE) {
		logger->log_warn(name(), "Default value was send");
		return;
	}

	//find corresponding MPS
	std::string mps_name = object_tracking_if_->enum_tostring("EXPECTED_MPS", current_expected_mps_);
	mps_name             = mps_name.replace(1, 1, "-");

	fawkes::NavGraphNode node = navgraph->node(mps_name);
	if (!node.is_valid()) {
		std::string warnstring = "No navgraph node for " + mps_name + "!";
		logger->log_warn(name(), warnstring.c_str());
		mps_x_   = 0;
		mps_y_   = 0;
		mps_ori_ = 0;
		return;
	}

	mps_x_   = node.x();
	mps_y_   = node.y();
	mps_ori_ = 0;
	if (node.has_property("orientation")) {
		mps_ori_ = node.property_as_float("orientation");
	}
	logger->log_info("mps_x_: ", std::to_string(mps_x_).c_str());
	logger->log_info("mps_y_: ", std::to_string(mps_y_).c_str());
	logger->log_info("mps_ori_: ", std::to_string(mps_ori_).c_str());

	//find expected object position as middle point
	switch (current_expected_side_) {
	case ObjectTrackingInterface::INPUT_CONVEYOR:
		exp_pos_map[0] = compute_middle_x(0);
		exp_pos_map[1] = compute_middle_y(0);
		exp_pos_map[2] = belt_height_;
		break;
	case ObjectTrackingInterface::OUTPUT_CONVEYOR:
		exp_pos_map[0] = mps_x_ - (belt_lenght_ / 2) * cos(mps_ori_);
		exp_pos_map[1] = mps_y_ - (belt_lenght_ / 2) * sin(mps_ori_);
		exp_pos_map[2] = belt_height_;
		break;
	case ObjectTrackingInterface::SLIDE:
		exp_pos_map[0] = compute_middle_x(slide_offset_side_);
		exp_pos_map[1] = compute_middle_y(slide_offset_side_);
		exp_pos_map[2] = slide_height_;
		break;
	case ObjectTrackingInterface::SHELF_LEFT:
		exp_pos_map[0] = compute_middle_x(left_shelf_offset_side_);
		exp_pos_map[1] = compute_middle_y(left_shelf_offset_side_);
		exp_pos_map[2] = shelf_height_;
		break;
	case ObjectTrackingInterface::SHELF_MIDDLE:
		exp_pos_map[0] = compute_middle_x(middle_shelf_offset_side_);
		exp_pos_map[1] = compute_middle_y(middle_shelf_offset_side_);
		exp_pos_map[2] = shelf_height_;
		break;
	case ObjectTrackingInterface::SHELF_RIGHT:
		exp_pos_map[0] = compute_middle_x(right_shelf_offset_side_);
		exp_pos_map[1] = compute_middle_y(right_shelf_offset_side_);
		exp_pos_map[2] = shelf_height_;
		break;
	default:
		logger->log_error(object_tracking_if_->enum_tostring("EXPECTED_SIDE", current_expected_side_),
		                  " is an invalid MPS-side!");
		return;
	}

	if (current_object_type_ == ObjectTrackingInterface::WORKPIECE) {
		exp_pos_map[2] += puck_height_ / 2;
		if (current_expected_side_ == ObjectTrackingInterface::OUTPUT_CONVEYOR) {
			exp_pos_map[0] += (puck_size_ / 2) * cos(mps_ori_);
			exp_pos_map[1] += (puck_size_ / 2) * sin(mps_ori_);
		} else {
			exp_pos_map[0] -= (puck_size_ / 2) * cos(mps_ori_);
			exp_pos_map[1] -= (puck_size_ / 2) * sin(mps_ori_);
		}
	} else if (current_object_type_ == ObjectTrackingInterface::CONVEYOR_BELT_FRONT) {
		exp_pos_map[2] -= belt_size_ / 2;
	}

	logger->log_info("exp_pos_map[0]: ", std::to_string(exp_pos_map[0]).c_str());
	logger->log_info("exp_pos_map[1]: ", std::to_string(exp_pos_map[1]).c_str());
	logger->log_info("exp_pos_map[2]: ", std::to_string(exp_pos_map[2]).c_str());
	//transform into stamped point
	exp_pos_.stamp    = fawkes::Time(0.0);
	exp_pos_.frame_id = "map";
	exp_pos_.setX(exp_pos_map[0]);
	exp_pos_.setY(exp_pos_map[1]);
	exp_pos_.setZ(exp_pos_map[2]);
}

float
ObjectTrackingThread::compute_middle_x(float x_offset)
{
	return mps_x_ + x_offset * sin(mps_ori_) + (belt_lenght_ / 2) * cos(mps_ori_);
}

float
ObjectTrackingThread::compute_middle_y(float y_offset)
{
	return mps_y_ - y_offset * cos(mps_ori_) + (belt_lenght_ / 2) * sin(mps_ori_);
}

void
ObjectTrackingThread::set_shm()
{
	//read only
	shm_buffer_ = new firevision::SharedMemoryImageBuffer(shm_id_.c_str(), true);
	if (!shm_buffer_->is_valid()) {
		throw fawkes::Exception("Shared memory segment not valid");
	} else {
		shm_active_ = true;
	}
}

void
ObjectTrackingThread::detect_objects(Mat image, std::vector<std::array<float, 4>> &out_boxes)
{
	std::vector<float>                confidences;
	std::vector<Rect>                 boxes;
	std::vector<Mat>                  results;
	std::vector<std::array<float, 4>> yolo_bbs;

	Mat blob = blobFromImage(image, scale_, Size(inpWidth_, inpHeight_), Scalar(), swapRB_);
	net_.setInput(blob);
	net_.forward(results, outName_);
	//results: L x N x (5 + #classes): 3 x 5808(in last layer) x [center_x, center_y, width, height, background_class, WORKPIECE, CONVEYOR, SLIDE]

	//check each yolo-layer output
	for (size_t l = 0; l < outName_.size(); l++) {
		//pointer to access results' data
		float *data = (float *)results[l].data;

		//confidence thresholding
		for (int j = 0; j < results[l].rows; ++j, data += results[l].cols) {
			//take confidence for target class - filter other classes
			float confidence = data[4 + (int)current_object_type_];
			if (confidence > confThreshold_) {
				std::array<float, 4> yolo_bb = {data[0], data[1], data[2], data[3]};
				yolo_bbs.push_back(yolo_bb);

				Rect rect_bb;
				convert_bb_yolo2rect(yolo_bb, rect_bb);
				boxes.push_back(rect_bb);

				confidences.push_back(confidence);
			}
		}
	}

	//non-maximum suppression
	std::vector<int> indices;
	NMSBoxes(boxes, confidences, confThreshold_, nmsThreshold_, indices);
	for (size_t i = 0; i < indices.size(); ++i) {
		out_boxes.push_back(yolo_bbs[indices[i]]);
	}
}

void
ObjectTrackingThread::convert_bb_yolo2rect(std::array<float, 4> yolo_bbox, Rect &rect_bbox)
{
	int centerX = (int)(yolo_bbox[0] * camera_width_);
	int centerY = (int)(yolo_bbox[1] * camera_height_);
	int width   = (int)(yolo_bbox[2] * camera_width_);
	int height  = (int)(yolo_bbox[3] * camera_height_);
	int left    = centerX - width / 2;
	int top     = centerY - height / 2;
	rect_bbox   = Rect(left, top, width, height);
}

bool
ObjectTrackingThread::closest_position(std::vector<std::array<float, 4>>      bounding_boxes,
                                       fawkes::tf::Stamped<fawkes::tf::Point> ref_pos,
                                       float                                  mps_angle,
                                       float                                  closest_pos[3],
                                       Rect &                                 closest_box,
                                       float &                                additional_height)
{
	float  min_dist = max_acceptable_dist_;
	size_t box_id;

	for (size_t i = 0; i < bounding_boxes.size(); ++i) {
		float pos[3];
		float wp_additional_height = 0;
		compute_3d_point_direct_yolo(bounding_boxes[i], mps_angle, pos, wp_additional_height);
		float dist = sqrt((pos[0] - ref_pos.getX()) * (pos[0] - ref_pos.getX())
		                  + (pos[1] - ref_pos.getY()) * (pos[1] - ref_pos.getY())
		                  + (pos[2] - ref_pos.getZ()) * (pos[2] - ref_pos.getZ()));
		//logger->log_warn(name(), std::to_string(dist).c_str());
		//logger->log_info("pos[0]: ", std::to_string(pos[0]).c_str());
		//logger->log_info("pos[1]: ", std::to_string(pos[1]).c_str());
		//logger->log_info("pos[2]: ", std::to_string(pos[2]).c_str());
		//logger->log_info("ref[0]: ", std::to_string(ref_pos.getX()).c_str());
		//logger->log_info("ref[1]: ", std::to_string(ref_pos.getY()).c_str());
		//logger->log_info("ref[2]: ", std::to_string(ref_pos.getZ()).c_str());
		if (dist < min_dist) {
			min_dist          = dist;
			closest_pos[0]    = pos[0];
			closest_pos[1]    = pos[1];
			closest_pos[2]    = pos[2];
			box_id            = i;
			additional_height = wp_additional_height;
		}
	}

	if (min_dist == max_acceptable_dist_) {
		// logger->log_warn(name(), "No detection close enough!");
		return false;
	}

	//convert closest bounding box into Rect to draw it on images
	convert_bb_yolo2rect(bounding_boxes[box_id], closest_box);

	return true;
}

void
ObjectTrackingThread::compute_3d_point(Rect bounding_box, float point[3])
{
	//get distance using triangle similarity
	float dist     = focal_length_ * object_widths_[(int)current_object_type_] / bounding_box.width;
	float pixel[2] = {(float)bounding_box.x + bounding_box.width / 2,
	                  (float)bounding_box.y + bounding_box.height / 2};
	rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, dist);
}

void
ObjectTrackingThread::compute_3d_point_direct(Rect bounding_box, float mps_angle, float point[3])
{
	//adjusted from rs2_deproject_pixel_to_point at
	// https://github.com/IntelRealSense/librealsense/blob/master/src/rs.cpp#L3598

	//delta values (correct if no distortion):
	float dx_1 = (bounding_box.x - camera_ppx_) / camera_fx_;
	float dx_2 = (bounding_box.x + bounding_box.width - camera_ppx_) / camera_fx_;
	float dy   = (bounding_box.y + bounding_box.height / 2 - camera_ppy_) / camera_fy_;

	// delta values
	// float cx_1 = x_1;
	// float cx_2 = x_2;
	// float cy = y;

	if (camera_model_ == 2) { //Inverse Brown-Conrady distortion
		//compute delta_x_1 and delta_y considering distortion
		converge_delta_ibc(dx_1, dy, dx_1, dy);

		//compute delta_x_2 and delta_y considering distortion
		converge_delta_ibc(dx_2, dy, dx_2, dy);
	}

	if (dx_1 == dx_2) {
		logger->log_error(name(), "Width of 0: Cannot project into 3D space!");
		point[0] = 0;
		point[1] = 0;
		point[2] = 0;
		return;
	}

	//workpieces have an equal width from all angles
	if (current_object_type_ == ObjectTrackingInterface::WORKPIECE) {
		mps_angle = 0;
	}

	//percieved object width from angle
	float object_width = cos(mps_angle) * object_widths_[(int)current_object_type_];

	//distance towards this perception + additional adjustments through angle
	float dist = object_width / (dx_2 - dx_1)
	             + sin(abs(mps_angle)) * object_widths_[(int)current_object_type_] / 2;

	//compute middle point with deltas and distance
	float depth_point[3];
	depth_point[0] = (dx_1 + dx_2) * dist / 2;
	depth_point[1] = -dy * dist;
	depth_point[2] = dist;

	if (!use_saved_ && !rotate_image_) {
		depth_point[0] = -depth_point[0];
		depth_point[1] = -depth_point[1];
	}

	//adjust for cam_gripper frame
	point[0] = depth_point[2];
	point[1] = -depth_point[0];
	point[2] = depth_point[1];
}

void
ObjectTrackingThread::compute_3d_point_direct_yolo(std::array<float, 4> bounding_box,
                                                   float                mps_angle,
                                                   float                point[3],
                                                   float &              wp_additional_height)
{
	float bb_left    = bounding_box[0] - bounding_box[2] / 2;
	float bb_right   = bounding_box[0] + bounding_box[2] / 2;
	float bb_bottom  = bounding_box[1] + bounding_box[3] / 2;
	float bb_top     = bounding_box[1] - bounding_box[3] / 2;
	float bb_centerY = bounding_box[1];

	//delta values (correct if no distortion):
	float dx_left   = (bb_left * camera_width_ - camera_ppx_) / camera_fx_;
	float dx_right  = (bb_right * camera_width_ - camera_ppx_) / camera_fx_;
	float dy_bottom = (bb_bottom * camera_height_ - camera_ppy_) / camera_fy_;
	float dy_top    = (bb_top * camera_height_ - camera_ppy_) / camera_fy_;
	float dy_center = (bb_centerY * camera_height_ - camera_ppy_) / camera_fy_;

	if (dx_left == dx_right) {
		logger->log_error(name(), "Width of 0: Cannot project into 3D space!");
		point[0] = 0;
		point[1] = 0;
		point[2] = 0;
		return;
	}

	if (current_object_type_ == ObjectTrackingInterface::WORKPIECE) {
		//workpieces have an equal width from all angles
		//distance towards this perception + additional adjustments through angle
		float dist = object_widths_[(int)current_object_type_] / (dx_right - dx_left);

		//compute base middle point with deltas and distance
		// using the bottom point + wp_height/2
		point[0] = dist;
		point[1] = (dx_left + dx_right) * dist / 2;
		point[2] = dy_top * dist + puck_height_ / 2;

		wp_additional_height = max(puck_height_ / 2, dy_bottom * dist - point[2]);
	} else {
		//percieved object width from angle
		float object_width = cos(mps_angle) * object_widths_[(int)current_object_type_];

		//distance towards this perception + additional adjustments through angle
		float dist = object_width / (dx_right - dx_left)
		             + sin(abs(mps_angle)) * object_widths_[(int)current_object_type_] / 2;

		//compute middle point with deltas and distance
		point[0] = dist;
		point[1] = (dx_left + dx_right) * dist / 2;
		point[2] = dy_center * dist;

		wp_additional_height = 0;
	}
}

void
ObjectTrackingThread::converge_delta_ibc(float dx_start, float dy_start, float dx, float dy)
{
	dx = dx_start;
	dy = dy_start;

	// 10 iterations are supposed to converge
	for (size_t i = 0; i < 10; i++) {
		float r2 = dx * dx + dy * dy;
		float icdist =
		  (float)1
		  / (float)(1 + ((camera_coeffs_[4] * r2 + camera_coeffs_[1]) * r2 + camera_coeffs_[0]) * r2);
		float xq        = dx / icdist;
		float yq        = dy / icdist;
		float epsilon_x = 2 * camera_coeffs_[2] * xq * yq + camera_coeffs_[3] * (r2 + 2 * xq * xq);
		float epsilon_y = 2 * camera_coeffs_[3] * xq * yq + camera_coeffs_[2] * (r2 + 2 * yq * yq);
		dx              = (dx_start - epsilon_x) * icdist;
		dy              = (dy_start - epsilon_y) * icdist;
	}
}

void
ObjectTrackingThread::compute_target_frames(fawkes::tf::Stamped<fawkes::tf::Point> object_pos,
                                            float                                  mps_angle,
                                            double gripper_target[3],
                                            double base_target[3])
{
	//compute target gripper frame first
	switch (current_object_type_) {
	case ObjectTrackingInterface::WORKPIECE:
		gripper_target[0] = object_pos.getX();
		gripper_target[1] = object_pos.getY();
		gripper_target[2] = object_pos.getZ() + gripper_offset_pick_;
		break;
	case ObjectTrackingInterface::CONVEYOR_BELT_FRONT:
		gripper_target[2] = object_pos.getZ() + gripper_offset_put_ + belt_size_ / 2 + puck_height_ / 2;
		break;
	case ObjectTrackingInterface::SLIDE_FRONT:
		gripper_target[2] = object_pos.getZ() + gripper_offset_put_ + puck_height_ / 2;
		break;
	default:
		logger->log_error(object_tracking_if_->enum_tostring("TARGET_OBJECT_TYPE",
		                                                     current_object_type_),
		                  " is an invalid Target!");
		return;
	}

	if (current_object_type_ == ObjectTrackingInterface::CONVEYOR_BELT_FRONT
	    || current_object_type_ == ObjectTrackingInterface::SLIDE_FRONT) {
		gripper_target[0] = object_pos.getX() + cos(mps_angle) * puck_size_;
		gripper_target[1] = object_pos.getY() - sin(mps_angle) * puck_size_;
	}

	//compute target base frame
	base_target[0] = gripper_target[0] - cos(mps_angle) * base_offset_;
	base_target[1] = gripper_target[1] + sin(mps_angle) * base_offset_;
	base_target[2] = mps_angle;
}
