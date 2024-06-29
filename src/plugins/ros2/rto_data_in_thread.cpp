/***************************************************************************
 *  rto_data_in_thread.cpp - Thread to receive robotino input data
 *
 *  Created: Nov 2024
 *  Copyright  2024  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "rto_data_in_thread.h"

using namespace fawkes;

/** @class RTODataInThread "cx_skiller_thread.h"
 * Thread to publish robotinos analog and digital input data to ROS.
 * This thread reads data from the respective topics of the robotino
 * driver and writes it to a fawkes interface.
 * @author Tarik Viehmann
 */

/** Constructor. */
RTODataInThread::RTODataInThread() : Thread("RTODataInThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
RTODataInThread::~RTODataInThread()
{
}

/** Initialize the thread.
 * Register as writer for the blackboard interface and subscribe to data in topics.
 */
void
RTODataInThread::init()
{
	std::string cfg_prefix = "/rto-data-in";
	data_iface_            = blackboard->open_for_writing<RTODataInterface>("robotino_data");

	rclcpp::SubscriptionOptionsBase subopts;
	subopts.ignore_local_publications = true;
	std::string analog_topic =
	  config->get_string_or_default((cfg_prefix + "/analog-topic").c_str(), "analog_readings");
	std::string digital_topic =
	  config->get_string_or_default((cfg_prefix + "/digital-topic").c_str(), "digital_readings");

	rto_analog_listener_ = node_handle->create_subscription<rto_msgs::msg::AnalogReadings>(
	  analog_topic,
	  1,
	  [this](const rto_msgs::msg::AnalogReadings::SharedPtr msg) {
		  const float *values = msg->values.data();
		  data_iface_->set_analog_readings(values);
		  data_iface_->write();
	  },
	  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(subopts));

	rto_digital_listener_ = node_handle->create_subscription<rto_msgs::msg::DigitalReadings>(
	  digital_topic,
	  1,
	  [this](const rto_msgs::msg::DigitalReadings::SharedPtr msg) {
		  std::vector<bool> values = msg->values;

		  bool values_raw[8];
		  for (size_t i = 0; i < values.size(); ++i) {
			  values_raw[i] = values[i];
		  }

		  data_iface_->set_digital_readings(values_raw);
		  data_iface_->write();
	  },
	  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(subopts));
}

/** Close the interface. */
void
RTODataInThread::finalize()
{
	blackboard->close(data_iface_);
}
