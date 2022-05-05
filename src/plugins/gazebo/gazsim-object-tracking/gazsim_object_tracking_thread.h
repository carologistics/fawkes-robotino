/***************************************************************************
 *  gazsim_object_tracking.h - Thread simulates object tracking of
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

#ifndef __PLUGINS_GAZSIM_OBJECT_TRACKING_THREAD_H_
#define __PLUGINS_GAZSIM_OBJECT_TRACKING_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/ObjectTrackingInterface.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <tf/types.h>
#include <utils/time/time.h>

#include <cmath>
#include <string.h>

//from Gazebo
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

namespace fawkes {
class Position3DInterface;
class ObjectTrackingInterface;
} // namespace fawkes

class ObjectTrackingThread : public fawkes::Thread,
                             public fawkes::ClockAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::GazeboAspect,
                             public fawkes::TransformAspect
{
public:
	ObjectTrackingThread();

	virtual void init();
	virtual void loop();

private:
	//Subscriber to receive localization data from gazebo
	gazebo::transport::SubscriberPtr localization_sub_;
	gazebo::transport::SubscriberPtr puck_pose_sub_;
	std::string                      gps_topic_;
	gazebo::transport::SubscriberPtr factory_sub_;
	std::string                      factory_topic_;

	//ObjectTrackingInterface to receive messages and update target frames
	fawkes::ObjectTrackingInterface *object_tracking_if_;
	std::string                      object_tracking_if_name_;

	//handler function for incoming localization data messages
	void on_localization_msg(ConstPosePtr &msg);
	void on_puck_pose_msg(ConstPosePtr &msg);
	void on_factory_msg(ConstFactoryPtr &msg);

	//puck tracking data
	int         tracked_pucks_;
	std::string puck_names_[50];

	//robotino tracking data
	double robotino_position_[3];
	double puck_positions_[50][3];

	//MPS tracking data
	std::string mps_names_[14] = {"M-BS",
	                              "M-RS1",
	                              "M-RS2",
	                              "M-CS1",
	                              "M-CS2",
	                              "M-DS",
	                              "M-SS",
	                              "C-BS",
	                              "C-RS1",
	                              "C-RS2",
	                              "C-CS1",
	                              "C-CS2",
	                              "C-DS",
	                              "C-SS"};
	double      mps_positions_[14][3];

	//MPS values used to compute expected object position on MPS
	//puck values:
	double puck_size_;
	double puck_height_;

	//belt values:
	double belt_height_;
	double belt_lenght_;
	double belt_offset_side_;

	//slide values:
	double slide_offset_side_;
	double slide_height_;

	//shelf values:
	double left_shelf_offset_side_;
	double middle_shelf_offset_side_;
	double right_shelf_offset_side_;
	double shelf_height_;

	//target frame offsets:
	double gripper_offset_pick_;
	double gripper_offset_put_;
	double base_offset_;

	//called from skill
	void start_tracking(fawkes::ObjectTrackingInterface::StartTrackingMessage &msg);

	//tracking variables
	fawkes::ObjectTrackingInterface::TARGET_OBJECT_TYPE current_object_type_;
	fawkes::ObjectTrackingInterface::EXPECTED_MPS       current_expected_mps_;
	fawkes::ObjectTrackingInterface::EXPECTED_SIDE      current_expected_side_;
	bool                                                mps_found;

	//tracked MPS:
	double mps_x_;
	double mps_y_;
	double mps_ori_;

	//tracked middle point:
	double middle_x_;
	double middle_y_;
	double middle_z_;

	//if workpiece is tracked, this is the index of the tracked WP
	int closest_puck_ind_;

	//helper functions for calculating target frame
	double                     compute_middle_x(double x_offset);
	double                     compute_middle_y(double y_offset);
	double                     distance_middle_to_puck(int puck_ind);
	std::tuple<double, double> transform_to_base_frame(double wp_x, double wp_y);

	//updating tracker
	fawkes::Time last_sent_time_;
	bool         tracking_;
	int          msgid_;

	//compute and update target frames frequently
	void send_target_frames();
};

#endif
