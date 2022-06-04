/***************************************************************************
 *  direct_com_thread.h - Arduino com thread for direct communication
 *
 *  Created: Mon Apr 04 11:48:36 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
 *                  2022  Matteo Tschesche
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

#ifndef __PLUGINS_ARDUINO_TF_THREAD_H_
#define __PLUGINS_ARDUINO_TF_THREAD_H_

#include "com_message.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/JoystickInterface.h>
#include <tf/types.h>
#include <utils/time/time.h>

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <memory>

namespace fawkes {
class Mutex;
class Clock;
class TimeWait;

class ArduinoInterface;
} // namespace fawkes

class ArduinoTFThread : public fawkes::Thread,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::BlockedTimingAspect,
                        public fawkes::BlackBoardAspect,
                        public fawkes::ClockAspect,
                        public fawkes::TransformAspect

{
public:
	ArduinoTFThread();
	/**
   * @brief Constructor
   *
   * @param cfg_name Name of the config file
   * @param cfg_prefix Tag prefixes of the arduino tf config values
   */
	ArduinoTFThread(std::string &cfg_name, std::string &cfg_prefix);
	virtual ~ArduinoTFThread();

	virtual void init();
	//	virtual void once();
	virtual void loop();
	virtual void finalize();

	/**
   * @brief Resets the current position to a given position
   *
   * @param new_x New X-coordinate
   * @param new_y New Y-coordinate
   * @param new_z New Z-coordinate
   */
	void set_position(float new_x, float new_y, float new_z);

private:
	std::string cfg_gripper_frame_id_;
	std::string cfg_gripper_dyn_x_frame_id_;
	std::string cfg_gripper_dyn_y_frame_id_;
	std::string cfg_gripper_dyn_z_frame_id_;

	std::string cfg_gripper_origin_x_frame_id_;
	std::string cfg_gripper_origin_y_frame_id_;
	std::string cfg_gripper_origin_z_frame_id_;

	std::string cfg_prefix_;
	std::string cfg_name_;

	fawkes::tf::TransformPublisher *dyn_x_pub;
	fawkes::tf::TransformPublisher *dyn_y_pub;
	fawkes::tf::TransformPublisher *dyn_z_pub;

	float cfg_static_tf_x_home_;
	float cfg_static_tf_y_home_;
	float cfg_static_tf_z_home_;
	float cfg_x_max_;
	float cfg_y_max_;
	float cfg_z_max_;

	void load_config();

	// update transform
	virtual void update();

	fawkes::Time end_time_point_;
	float        desired_end_z_pose_;
	float        current_end_z_pose_;

	float cur_step_d_;
	float cur_x_;
	float cur_y_;
	float cur_z_;

protected:
	/** Mutex to protect data_. Lock whenever accessing it. */
	boost::mutex data_mutex_;
};

#endif
