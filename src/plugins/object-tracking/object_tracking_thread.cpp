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
#include <math.h>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  fawkes::TransformAspect(fawkes::TransformAspect::BOTH, "object_tracking")
{
}

void
ObjectTrackingThread::init()
{
	logger->log_info(name(), "Initializing Object Tracker");

	//read config values for computing expected position and target frames
	puck_size_   = config->get_float("plugins/object_tracking/puck_values/puck_size");
	puck_height_ = config->get_float("plugins/object_tracking/puck_values/puck_height");

	belt_height_      = config->get_float("plugins/object_tracking/belt_values/belt_height");
	belt_lenght_      = config->get_float("plugins/object_tracking/belt_values/belt_lenght");
	belt_offset_side_ = config->get_float("plugins/object_tracking/belt_values/belt_offset_side");

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

	rotate_image_ = config->get_bool("plugins/object_tracking/rotate_image");

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
	outName_ = net_.getUnconnectedOutLayersNames().back();

	//set up weighted average filter
	filter_weights_[0] = 0.15; //expected position
	filter_weights_[1] = 0.35; //current response
	filter_weights_[2] = 0.20; //last response
	filter_weights_[3] = 0.15; //2. last response
	filter_weights_[4] = 0.10; //3. last response
	filter_weights_[5] = 0.05; //4, last response

	filter_size_ = sizeof(filter_weights_) / sizeof(filter_weights_[0]);

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
	shm_id_  = config->get_string("plugins/object_tracking/buffer/shm_image_id");
	frame_id = this->config->get_string("plugins/object_tracking/buffer/frame");

	//--------------------------------------
	name_it_    = 0;
	tracking_   = false;
	shm_active_ = false;
}

void
ObjectTrackingThread::loop()
{
	//handle incomming messages
	while (!object_tracking_if_->msgq_empty()) {
		if (object_tracking_if_->msgq_first_is<ObjectTrackingInterface::StartTrackingMessage>()) {
			logger->log_info(name(), "Received StartTrackingMessage");

			ObjectTrackingInterface::StartTrackingMessage *msg =
			  object_tracking_if_->msgq_first<ObjectTrackingInterface::StartTrackingMessage>();
			current_object_type_   = msg->object_type_to_set();
			current_expected_mps_  = msg->expected_mps_to_set();
			current_expected_side_ = msg->expected_side_to_set();

			compute_expected_position();
			//logger->log_info("past_responses_[0][0]: ", std::to_string(past_responses_[0][0]).c_str());

			if (!shm_active_)
				set_shm();

			//start interface
			tracking_ = true;
			msgid_    = 0;
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

	if (!use_saved_ && !tracking_)
		return;

	//get all filenames in the given directory or the filename of the image path
	if (use_saved_ && filenames_.empty())
		glob(image_path_ + "*", filenames_);

	fawkes::Time start_time(clock);

	Mat image;

	if (use_saved_) {
		current_object_type_ = saved_object_type_;

		bool found_image = false;
		if (name_it_ >= filenames_.size())
			return;

		while (name_it_ < filenames_.size() and !found_image) {
			if (boost::algorithm::ends_with(filenames_[name_it_], ".png")
			    || //TODO: catch more image file types
			    boost::algorithm::ends_with(filenames_[name_it_], ".jpg")) {
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
		unsigned char tmp[camera_width_ * camera_height_ * 3];
		firevision::convert(
		  firevision::BGR, firevision::BGR, shm_buffer_->buffer(), tmp, camera_width_, camera_height_);
		image = Mat(camera_width_, camera_height_, CV_8UC3, tmp);
	}

	if (rotate_image_)
		rotate(image, image, ROTATE_180);

	//detect objects
	std::vector<Rect> out_boxes;
	fawkes::Time      before_detect(clock);
	detect_objects(image, out_boxes);
	fawkes::Time after_detect(clock);
	//logger->log_info("boxes found: ", std::to_string(out_boxes.size()).c_str());

	//draw bounding boxes
	//for (Rect box : out_boxes) {
	//	rectangle(image, box, Scalar(0, 255, 0));
	//}

	//display results
	//std::string windowName = "Results";
	//imshow(windowName, image);
	//waitKey(0);
	//destroyWindow(windowName);

	//save results
	//std::string new_img_name = image_path_.insert(image_path_.find("."), "_results");
	//imwrite(new_img_name, image);

	if (use_saved_) {
		//logger->log_info("bounding boxes: ", std::to_string(out_boxes.size()).c_str());
		for (size_t i = 0; i < out_boxes.size(); ++i) {
			//logger->log_info("box: ", std::to_string(i).c_str());
			float pos[3];
			//compute_3d_point(out_boxes[i], pos);
			compute_3d_point_direct(out_boxes[i], 0.0, pos);
			//logger->log_info("x (right): ", std::to_string(pos[0]).c_str());
			//logger->log_info("y (up)   : ", std::to_string(pos[1]).c_str());
			//logger->log_info("z (depth): ", std::to_string(pos[2]).c_str());
		}
		fawkes::Time after_projection(clock);
		logger->log_info("load image time ", std::to_string(before_detect - &start_time).c_str());
		logger->log_info("detection time  ", std::to_string(after_detect - &before_detect).c_str());
		logger->log_info("box time        ", std::to_string(after_projection - &after_detect).c_str());
		logger->log_info("overall time    ", std::to_string(after_projection - &start_time).c_str());

		return;
	}

	//project bounding boxes into 3d points and take closest to expectation
	float exp_pos[3] = {exp_x_, exp_y_, exp_z_};

	//TODO: transform exp_pos to base_frame and only use exp_pos[i] instead of the others from here on

	float cur_object_pos[3];
	bool  detected = closest_position(out_boxes, exp_pos, cur_object_pos);
	//cur_object_pos is set to expected position if no bounding box was close
	// enough and is used the same in the following

	fawkes::Time after_projection(clock);
	logger->log_info("load image time ", std::to_string(before_detect - &start_time).c_str());
	logger->log_info("detection time  ", std::to_string(after_detect - &before_detect).c_str());
	logger->log_info("box time        ", std::to_string(after_projection - &after_detect).c_str());
	logger->log_info("overall time    ", std::to_string(after_projection - &start_time).c_str());

	logger->log_info("cur_object_pos[0]: ", std::to_string(cur_object_pos[0]).c_str());
	logger->log_info("cur_object_pos[1]: ", std::to_string(cur_object_pos[1]).c_str());
	logger->log_info("cur_object_pos[2]: ", std::to_string(cur_object_pos[2]).c_str());

	//use weighted average to improve robustness of object position
	float weighted_object_pos[3];
	weighted_object_pos[0] = filter_weights_[0] * exp_x_;
	weighted_object_pos[1] = filter_weights_[0] * exp_y_;
	weighted_object_pos[2] = filter_weights_[0] * exp_z_;

	weighted_object_pos[0] += filter_weights_[1] * cur_object_pos[0];
	weighted_object_pos[1] += filter_weights_[1] * cur_object_pos[1];
	weighted_object_pos[2] += filter_weights_[1] * cur_object_pos[2];

	for (int i = 0; i < (int)past_responses_.size(); i++) {
		weighted_object_pos[0] += filter_weights_[2 + i] * past_responses_[i][0];
		weighted_object_pos[1] += filter_weights_[2 + i] * past_responses_[i][1];
		weighted_object_pos[2] += filter_weights_[2 + i] * past_responses_[i][2];
	}

	std::array<float, 3> current_response = {cur_object_pos[0], cur_object_pos[1], cur_object_pos[2]};
	past_responses_.push_front(current_response);
	past_responses_.pop_back();

	//TODO: if testing tracking with saved_images, return here

	//TODO: get mps angle through laser-lines
	//TODO: get mps angle through navgraph (mps_ori-robot_ori) if too far away from mps
	float mps_angle = 0;

	float gripper_target[3];
	float base_target[3];
	compute_target_frames(weighted_object_pos, mps_angle, gripper_target, base_target);
	logger->log_info("gripper_target[0]: ", std::to_string(gripper_target[0]).c_str());
	logger->log_info("gripper_target[1]: ", std::to_string(gripper_target[1]).c_str());
	logger->log_info("gripper_target[2]: ", std::to_string(gripper_target[2]).c_str());
	logger->log_info("base_target[0]: ", std::to_string(base_target[0]).c_str());
	logger->log_info("base_target[1]: ", std::to_string(base_target[1]).c_str());
	logger->log_info("base_target[2]: ", std::to_string(base_target[2]).c_str());

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
	object_tracking_if_->write();
}

void
ObjectTrackingThread::compute_expected_position()
{
	logger->log_info(name(), "Start Tracking");

	if (current_object_type_ == ObjectTrackingInterface::DEFAULT_TYPE
	    || current_expected_mps_ == ObjectTrackingInterface::DEFAULT_MPS
	    || current_expected_side_ == ObjectTrackingInterface::DEFAULT_SIDE) {
		logger->log_warn(name(), "Default value was send");
		return;
	}

	//find corresponding MPS
	std::string mps_name = object_tracking_if_->enum_tostring("EXPECTED_MPS", current_expected_mps_);
	mps_name             = mps_name.replace(1, 1, "-");

	//fawkes::NavGraphNode node = navgraph->node(mps_name); //TODO: if empty give error
	float node[3] = {0, 0, 0};

	//TODO: get navgraph to work

	// mps_x_ = node.x();
	// mps_y_ = node.y();
	// mps_ori_ = 0;
	// if (node.has_property("orientation")) {
	// 	mps_ori_ = node.property_as_float("orientation");
	// }
	mps_x_   = node[0];
	mps_y_   = node[1];
	mps_ori_ = node[2];

	//find expected object position as middle point
	switch (current_expected_side_) {
	case ObjectTrackingInterface::INPUT_CONVEYOR:
		exp_x_ = compute_middle_x(belt_offset_side_);
		exp_y_ = compute_middle_y(belt_offset_side_);
		exp_z_ = belt_height_;
		break;
	case ObjectTrackingInterface::OUTPUT_CONVEYOR:
		exp_x_ = mps_x_ + belt_offset_side_ * cos(mps_ori_) + (belt_lenght_ / 2) * sin(mps_ori_);
		exp_y_ = mps_y_ + belt_offset_side_ * sin(mps_ori_) - (belt_lenght_ / 2) * cos(mps_ori_);
		exp_z_ = belt_height_;
		break;
	case ObjectTrackingInterface::SLIDE:
		exp_x_ = compute_middle_x(slide_offset_side_);
		exp_y_ = compute_middle_y(slide_offset_side_);
		exp_z_ = slide_height_;
		break;
	case ObjectTrackingInterface::SHELF_LEFT:
		exp_x_ = compute_middle_x(left_shelf_offset_side_);
		exp_y_ = compute_middle_y(left_shelf_offset_side_);
		exp_z_ = shelf_height_;
		break;
	case ObjectTrackingInterface::SHELF_MIDDLE:
		exp_x_ = compute_middle_x(middle_shelf_offset_side_);
		exp_y_ = compute_middle_y(middle_shelf_offset_side_);
		exp_z_ = shelf_height_;
		break;
	case ObjectTrackingInterface::SHELF_RIGHT:
		exp_x_ = compute_middle_x(right_shelf_offset_side_);
		exp_y_ = compute_middle_y(right_shelf_offset_side_);
		exp_z_ = shelf_height_;
		break;
	default:
		logger->log_error(object_tracking_if_->enum_tostring("EXPECTED_SIDE", current_expected_side_),
		                  " is an invalid MPS-side!");
		return;
	}

	if (current_object_type_ == ObjectTrackingInterface::WORKPIECE) {
		exp_z_ += puck_height_ / 2;
		if (current_expected_side_ == ObjectTrackingInterface::OUTPUT_CONVEYOR) {
			exp_x_ -= puck_size_ * sin(mps_ori_);
			exp_y_ += puck_size_ * cos(mps_ori_);
		} else {
			exp_x_ += puck_size_ * sin(mps_ori_);
			exp_y_ -= puck_size_ * cos(mps_ori_);
		}
	}

	//initialize responses with expected position
	for (int i = 0; i < filter_size_; i++) {
		std::array<float, 3> exp_pos = {exp_x_, exp_y_, exp_z_};
		past_responses_.push_front(exp_pos);
	}
}

float
ObjectTrackingThread::compute_middle_x(float x_offset)
{
	return mps_x_ + x_offset * cos(mps_ori_) - (belt_lenght_ / 2) * sin(mps_ori_);
}

float
ObjectTrackingThread::compute_middle_y(float y_offset)
{
	return mps_y_ + y_offset * sin(mps_ori_) + (belt_lenght_ / 2) * cos(mps_ori_);
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
	//shm_buffer_->set_frame_id(frame_id.c_str());
	image_buffer_ = shm_buffer_->buffer();
}

void
ObjectTrackingThread::detect_objects(Mat image, std::vector<Rect> &out_boxes)
{
	std::vector<float> confidences;
	std::vector<Rect>  boxes;
	std::vector<Mat>   outs;

	Mat blob = blobFromImage(image, 1.0f, Size(inpWidth_, inpHeight_), Scalar(), swapRB_);
	net_.setInput(blob, "", scale_, Scalar());
	net_.forward(outs, outName_);
	//outs: 1xNx(5+#classes) with [center_x, center_y, width, height, background_class, class0, class1, ...]

	if (outs.empty())
		return;

	logger->log_info(name(), "detected something");

	//pointer to access outs' data
	float *data = (float *)outs[0].data;

	//confidence thresholding
	for (int j = 0; j < outs[0].rows; ++j, data += outs[0].cols) {
		//take confidence for target class - filter other classes
		float confidence = data[4 + (int)current_object_type_];
		//logger->log_info("confidence: ", std::to_string(confidence).c_str());
		if (confidence > confThreshold_) {
			logger->log_info(name(), "found something");
			// logger->log_info("confidence: ", std::to_string(confidence).c_str());
			// logger->log_info("pos[0]: ", std::to_string(data[0]).c_str());
			// logger->log_info("pos[1]: ", std::to_string(data[1]).c_str());
			// logger->log_info("pos[2]: ", std::to_string(data[2]).c_str());
			// logger->log_info("pos[2]: ", std::to_string(data[3]).c_str());

			//TODO: also store the original values and use them for the direct 3d point computation, since they are more precise
			//float bb_left = data[0] - data[2]/2;
			//float bb_right = data[0] + data[2]/2;
			//float bb_centerY = data[1];

			int centerX = (int)(data[0] * image.cols);
			int centerY = (int)(data[1] * image.rows);
			int width   = (int)(data[2] * image.cols);
			int height  = (int)(data[3] * image.rows);
			int left    = centerX - width / 2;
			int top     = centerY - height / 2;

			confidences.push_back(confidence);
			boxes.push_back(Rect(left, top, width, height));
		}
	}

	//non-maximum suppression
	std::vector<int> indices;
	NMSBoxes(boxes, confidences, confThreshold_, nmsThreshold_, indices);
	for (size_t i = 0; i < indices.size(); ++i) {
		int idx = indices[i];
		out_boxes.push_back(boxes[idx]);
	}
}

bool
ObjectTrackingThread::closest_position(std::vector<Rect> bounding_boxes,
                                       float             exp_pos[3],
                                       float             closest_pos[3])
{
	// logger->log_info("bounding_boxes.size(): ", std::to_string(bounding_boxes.size()).c_str());
	float max_acceptable_dist = std::numeric_limits<float>::max();

	float min_dist = max_acceptable_dist;
	for (size_t i = 0; i < bounding_boxes.size(); ++i) {
		float pos[3];
		//compute_3d_point(bounding_boxes[i], pos);
		compute_3d_point_direct(bounding_boxes[i], 0.0, pos);
		float dist = sqrt((pos[0] - exp_pos[0]) * (pos[0] - exp_pos[0])
		                  + (pos[1] - exp_pos[1]) * (pos[1] - exp_pos[1])
		                  + (pos[2] - exp_pos[2]) * (pos[2] - exp_pos[2]));
		// logger->log_warn(name(), std::to_string(dist).c_str());
		// logger->log_info("pos[0]: ", std::to_string(pos[0]).c_str());
		// logger->log_info("pos[1]: ", std::to_string(pos[1]).c_str());
		// logger->log_info("pos[2]: ", std::to_string(pos[2]).c_str());
		if (dist < min_dist) {
			min_dist       = dist;
			closest_pos[0] = pos[0];
			closest_pos[1] = pos[1];
			closest_pos[2] = pos[2];
		}
	}

	if (min_dist == max_acceptable_dist) {
		// logger->log_warn(name(), "No detection close enough!");
		//continue with expected position
		closest_pos[0] = exp_x_;
		closest_pos[1] = exp_y_;
		closest_pos[2] = exp_z_;
		return false;
	}
	//TODO: draw closest bounding box on image
	return true;
}

void
ObjectTrackingThread::compute_3d_point(Rect bounding_box, float point[3])
{
	//get distance using triangle similarity
	logger->log_info(name(), "computing 3d point");
	logger->log_info("width: ", std::to_string(bounding_box.width).c_str());
	float dist     = focal_length_ * object_widths_[(int)current_object_type_] / bounding_box.width;
	float pixel[2] = {(float)bounding_box.x + bounding_box.width / 2,
	                  (float)bounding_box.y + bounding_box.height / 2};
	rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, dist);
}

void
ObjectTrackingThread::compute_3d_point_direct(Rect bounding_box, float angle, float point[3])
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

	//percieved object width from angle
	float object_width = cos(angle) * object_widths_[(int)current_object_type_];

	//distance towards this perception + additional adjustments through angle
	float dist =
	  object_width / (dx_2 - dx_1) + sin(abs(angle)) * object_widths_[(int)current_object_type_] / 2;

	//compute middle point with deltas and distance
	point[0] = (dx_1 + dx_2) * dist / 2;
	point[1] = dy * dist;
	point[2] = dist;
}

void
ObjectTrackingThread::converge_delta_ibc(float dx_start, float dy_start, float dx, float dy)
{
	dx = dx_start;
	dy = dy_start;

	// 10 iterations are supposed to converge
	for (int i = 0; i < 10; i++) {
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
ObjectTrackingThread::compute_target_frames(float object_pos[3],
                                            float mps_angle,
                                            float gripper_target[3],
                                            float base_target[3])
{
	//transform from cam_frame to base_frame
	float object_pos_base[3];
	transform_to_base_frame(object_pos, object_pos_base);

	//compute target gripper frame first
	switch (current_object_type_) {
	case ObjectTrackingInterface::WORKPIECE:
		gripper_target[0] = object_pos_base[0];
		gripper_target[1] = object_pos_base[1];
		gripper_target[2] = object_pos_base[2] + gripper_offset_pick_;
		break;
	case ObjectTrackingInterface::CONVEYOR_BELT_FRONT:
		gripper_target[0] = object_pos_base[0] - cos(mps_angle) * puck_size_ * 2;
		gripper_target[1] = object_pos_base[1] - sin(mps_angle) * puck_size_ * 2;
		gripper_target[2] =
		  object_pos_base[2] + gripper_offset_put_ + belt_size_ / 2 + puck_height_ / 2;
		break;
	case ObjectTrackingInterface::SLIDE_FRONT:
		gripper_target[0] = object_pos_base[0] - cos(mps_angle) * puck_size_ * 2;
		gripper_target[1] = object_pos_base[1] - sin(mps_angle) * puck_size_ * 2;
		gripper_target[2] = object_pos_base[2] + gripper_offset_put_ + puck_height_ / 2;
		break;
	default:
		logger->log_error(object_tracking_if_->enum_tostring("TARGET_OBJECT_TYPE",
		                                                     current_object_type_),
		                  " is an invalid Target!");
		return;
	}

	//compute target base frame
	base_target[0] = gripper_target[0] - cos(mps_angle) * base_offset_;
	base_target[1] = gripper_target[1] - sin(mps_angle) * base_offset_;
	base_target[2] = mps_angle;
}

void
ObjectTrackingThread::transform_to_base_frame(float object_pos[3], float object_pos_base[3])
{
	//TODO: write this or use transform instead
	//object_pos in [right, upwards, forwards]
	//object_pos_base in [forwards, left, upwards]
	//tf.transform(object_pos, object_pos_base, "cam_base", "base_frame");
	object_pos_base[0] = 0;
	object_pos_base[1] = 0;
	object_pos_base[2] = 0;
}
