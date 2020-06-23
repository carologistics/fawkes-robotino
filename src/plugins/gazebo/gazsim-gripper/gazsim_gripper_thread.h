
/***************************************************************************
 *  gazsim_gripper_thread.h - Plugin used to simulate a gripper
 *
 *  Created: Mon Apr 20 18:45:09 2015
 *  Copyright  2015 Frederik Zwilling
 *  Copyright  2019 Mostafa Gomaa
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

#ifndef __PLUGINS_GAZSIM_GRIPPER_THREAD_H_
#define __PLUGINS_GAZSIM_GRIPPER_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/DynamixelServoInterface.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <tf/types.h>
#include <utils/time/time.h>

#include <boost/thread/mutex.hpp>
#include <memory>

// from Gazebo
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

namespace fawkes {
class ArduinoInterface;
} // namespace fawkes

class GazsimGripperThread : public fawkes::Thread,
                            public fawkes::BlockedTimingAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::BlackBoardAspect,
                            public fawkes::GazeboAspect,
                            public fawkes::ClockAspect,
                            public fawkes::TransformAspect
{
public:
	GazsimGripperThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::ArduinoInterface *arduino_if_;

	std::string gripper_if_name_;
	std::string arduino_if_name_;
	std::string cfg_prefix_;

	std::string cfg_gripper_frame_id_;
	std::string cfg_gripper_dyn_x_frame_id_;
	std::string cfg_gripper_dyn_y_frame_id_;
	std::string cfg_gripper_dyn_z_frame_id_;
	std::string cfg_gripper_origin_x_frame_id_;
	std::string cfg_gripper_origin_y_frame_id_;
	std::string cfg_gripper_origin_z_frame_id_;

	float cfg_static_tf_x_home_;
	float cfg_static_tf_y_home_;
	float cfg_static_tf_z_home_;
	float cfg_x_max_;
	float cfg_y_max_;
	float cfg_z_max_;

	fawkes::tf::TransformPublisher *dyn_x_pub;
	fawkes::tf::TransformPublisher *dyn_y_pub;
	fawkes::tf::TransformPublisher *dyn_z_pub;

	void load_config();

	float cur_x_;
	float cur_y_;
	float cur_z_;

	// Publisher to sent msgs to gazebo
	gazebo::transport::PublisherPtr set_gripper_pub_;
	gazebo::transport::PublisherPtr set_conveyor_pub_;

	void send_gripper_msg(int value);

protected:
	/** Mutex to protect data_. Lock whenever accessing it. */
	boost::mutex data_mutex_;
};

#endif
