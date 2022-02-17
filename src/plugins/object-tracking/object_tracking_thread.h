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

#ifndef __PLUGINS_OBJECT_TRACKING_THREAD_H_
#define __PLUGINS_OBJECT_TRACKING_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/ObjectTrackingInterface.h>
#include <librealsense2/rsutil.h>
#include <navgraph/aspect/navgraph.h>
#include <tf/types.h>
#include <utils/time/time.h>

#include <cmath>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/dnn.hpp>
#include <string.h>

// firevision camera
#include <fvcams/camera.h>
#include <fvclassifiers/simple.h>
#include <fvutils/adapters/cvmatadapter.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>

namespace fawkes {
class ObjectTrackingInterface;
} // namespace fawkes

namespace firevision {
class SharedMemoryImageBuffer;
} // namespace firevision

class ObjectTrackingThread : public fawkes::Thread,
                             public fawkes::ClockAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::BlockedTimingAspect,
                             //public fawkes::NavGraphAspect,
                             public fawkes::TransformAspect
{
public:
	ObjectTrackingThread();

	virtual void init();
	virtual void loop();

private:
	//MPS values used to compute expected object position on MPS
	//puck values:
	float puck_size_;
	float puck_height_;

	//belt values:
	float belt_size_;
	float belt_height_;
	float belt_lenght_;
	float belt_offset_side_;

	//slide values:
	float slide_size_;
	float slide_offset_side_;
	float slide_height_;

	//shelf values:
	float left_shelf_offset_side_;
	float middle_shelf_offset_side_;
	float right_shelf_offset_side_;
	float shelf_height_;

	//target frame offsets:
	float gripper_offset_pick_;
	float gripper_offset_put_;
	float base_offset_;

	//camera params
	int   camera_width_;
	int   camera_height_;
	float camera_ppx_;
	float camera_ppy_;
	float camera_fx_;
	float camera_fy_;
	int   camera_model_;
	float camera_coeffs_[5];

	//use saved images
	bool                                                use_saved_;
	fawkes::ObjectTrackingInterface::TARGET_OBJECT_TYPE saved_object_type_;
	std::string                                         image_path_;
	std::vector<cv::String>                             filenames_;
	unsigned int                                        name_it_;

	//NN params
	std::string  weights_path_;
	std::string  config_path_;
	float        confThreshold_;
	float        nmsThreshold_;
	int          inpWidth_;
	int          inpHeight_;
	float        scale_;
	bool         swapRB_;
	cv::dnn::Net net_;
	std::string  outName_;

	//weighted average filter
	float                            filter_weights_[6];
	int                              filter_size_;
	std::deque<std::array<float, 3>> past_responses_;

	//camera params
	float          focal_length_;
	rs2_intrinsics intrinsics_;

	std::vector<float> object_widths_;
	bool               rotate_image_;

	//ObjectTrackingInterface to receive messages and update target frames
	std::string                      object_tracking_if_name_;
	fawkes::ObjectTrackingInterface *object_tracking_if_;

	//shared memory buffer
	std::string                          shm_id_;
	std::string                          frame_id;
	firevision::SharedMemoryImageBuffer *shm_buffer_;
	unsigned char *                      image_buffer_;
	bool                                 shm_active_;

	//expected object point:
	float exp_x_;
	float exp_y_;
	float exp_z_;

	//MPS navgraph position:
	float mps_x_;
	float mps_y_;
	float mps_ori_;

	//tracking values
	fawkes::ObjectTrackingInterface::TARGET_OBJECT_TYPE current_object_type_;
	fawkes::ObjectTrackingInterface::EXPECTED_MPS       current_expected_mps_;
	fawkes::ObjectTrackingInterface::EXPECTED_SIDE      current_expected_side_;
	bool                                                tracking_;
	int                                                 msgid_;

	//compute expected position and start tracking
	void  compute_expected_position();
	float compute_middle_x(float x_offset);
	float compute_middle_y(float y_offset);

	//set shared memory buffer to read only
	void set_shm();

	//use yolo to detect objects
	void detect_objects(cv::Mat image, std::vector<cv::Rect> &out_boxes);

	//project bounding boxes into 3d points and take closest to expectation
	bool
	closest_position(std::vector<cv::Rect> bounding_boxes, float exp_pos[3], float closest_pos[3]);
	void compute_3d_point(cv::Rect bounding_box, float point[3]);
	void compute_3d_point_direct(cv::Rect bounding_box, float angle, float point[3]);
	void converge_delta_ibc(float dx_start, float dy_start, float dx, float dy);

	//compute base and gripper target frame
	void compute_target_frames(float object_pos[3],
	                           float mps_angle,
	                           float gripper_target[3],
	                           float base_target[3]);
	void transform_to_base_frame(float object_pos[3], float object_pos_base[3]);
};

#endif
