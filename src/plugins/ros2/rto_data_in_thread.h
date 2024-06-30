/***************************************************************************
 *  rto_data_in_thread - Thread to get data input readings of robotino
 *
 *  Created: Oct 2023
 *  Copyright  2023  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#ifndef _ROS2_RTO_DATA_IN_THREAD_H_
#define _ROS2_RTO_DATA_IN_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/RTODataInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <rclcpp/rclcpp.hpp>
#include <rto_msgs/msg/analog_readings.hpp>
#include <rto_msgs/msg/digital_readings.hpp>
class RTODataInThread : public fawkes::Thread,
                        public fawkes::ConfigurableAspect,
                        public fawkes::LoggingAspect,
                        public fawkes::ROS2Aspect,
                        public fawkes::BlackBoardAspect
{
public:
	RTODataInThread();
	virtual ~RTODataInThread();

	virtual void init();
	virtual void finalize();

private:
	fawkes::RTODataInterface *data_iface_;

	rclcpp::Subscription<rto_msgs::msg::AnalogReadings>::SharedPtr  rto_analog_listener_;
	rclcpp::Subscription<rto_msgs::msg::DigitalReadings>::SharedPtr rto_digital_listener_;
};

#endif /* ROS2_RTO_DATA_IN_THREAD_H_ */
