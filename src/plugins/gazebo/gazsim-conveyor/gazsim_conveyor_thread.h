
/***************************************************************************
 *  gazsim_conveyor_thread.h - Plugin used to simulate a conveyor vision
 *
 *  Created: Fri Jul 10 11:27:12 2015
 *  Copyright  2015 Randolph Maaßen
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

#ifndef __PLUGINS_GAZSIM_CONVEYOR_THREAD_H_
#define __PLUGINS_GAZSIM_CONVEYOR_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
//#include <interfaces/Position3DInterface.h>
#include <interfaces/ConveyorPoseInterface.h>
#include <interfaces/SwitchInterface.h>
#include <llsf_msgs/ConveyorVisionResult.pb.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <utils/time/time.h>

// from Gazebo
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

#define MAX_LOOP_COUNT_TO_INVISIBLE 5
#define CFG_PREFIX "/plugins/conveyor_pose"

typedef const boost::shared_ptr<llsf_msgs::ConveyorVisionResult const> ConstConveyorVisionResultPtr;

class GazsimConveyorThread : public fawkes::Thread,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::GazeboAspect,
                             public fawkes::TransformAspect
{
public:
	GazsimConveyorThread();

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
	fawkes::ConveyorPoseInterface *pos_if_;
	fawkes::SwitchInterface *      plane_switch_if_;
	fawkes::SwitchInterface *      realsense_switch_if_;
	// fawkes::ConveyorConfigInterface *conv_config_if_;

	std::string conv_pose_if_name_;
	std::string realsense_frame_id_;
	std::string cfg_if_prefix_;
	std::string conveyor_frame_id_;

	gazebo::transport::SubscriberPtr conveyor_vision_sub_;
	void                             on_conveyor_vision_msg(ConstConveyorVisionResultPtr &msg);

	int32_t loopcount_;

	// copy of last msg to write the interface in the next loop
	llsf_msgs::ConveyorVisionResult last_msg_;
	bool                            new_data_;
	const double                    shelf_offset_x = 0.295;
};

#endif
