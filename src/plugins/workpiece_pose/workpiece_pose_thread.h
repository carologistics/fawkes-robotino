
/***************************************************************************
 *  workpiece_pose_thread.h - workpiece_pose
 *
 *  Created: Sat 15 May 16:28:00 CEST 2021
 *  Copyright  2021 Sebastian Eltester
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

#ifndef WORKPIECE_POSE_THREAD_H
#define WORKPIECE_POSE_THREAD_H

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/syncpoint_manager.h>
#include <aspect/tf.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/WorkpiecePoseInterface.h>
#include <plugins/ros/aspect/ros.h>

#define CFG_PREFIX "/plugins/workpiece_pose"

#define X_DIR 0
#define Y_DIR 1
#define Z_DIR 2

namespace fawkes {
class ConveyorPoseInterface;
class SwitchInterface;
class LaserLineInterface;
} // namespace fawkes

class WorkpiecePoseThread : public fawkes::Thread,
                            public fawkes::BlockedTimingAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::BlackBoardAspect,
                            public fawkes::ROSAspect,
                            public fawkes::TransformAspect,
                            public fawkes::SyncPointManagerAspect,
                            public fawkes::ClockAspect
{
public:
	WorkpiecePoseThread();

	virtual void init() override;
	virtual void loop() override;
	virtual void finalize() override;

private:
	// cfg values
	std::string cfg_bb_realsense_switch_name_;
	std::string workpiece_frame_id_;

	// state vars
	bool cfg_enable_switch_;

	// interfaces write
	fawkes::WorkpiecePoseInterface *               wp_pose_;
	fawkes::WorkpiecePoseInterface::WPPoseMessage *msg;

	// interfaces read
	fawkes::SwitchInterface *realsense_switch_;

	std::unique_ptr<fawkes::tf::Stamped<fawkes::tf::Pose>> result_pose_;

	void pose_write();
	void pose_publish_tf(const fawkes::tf::Stamped<fawkes::tf::Pose> &pose);
	bool read_xyz();

protected:
	virtual void
	run() override
	{
		Thread::run();
	}
};

#endif // WORKPIECE_POSE_THREAD_H
