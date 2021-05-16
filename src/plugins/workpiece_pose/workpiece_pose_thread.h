
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
#include <aspect/pointcloud.h>
#include <aspect/syncpoint_manager.h>
#include <aspect/tf.h>
#include <config/change_handler.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/WorkpiecePoseInterface.h>
#include <pcl_utils/compatibility.h>
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
                            public fawkes::ConfigurationChangeHandler,
                            public fawkes::BlackBoardAspect,
                            public fawkes::PointCloudAspect,
                            public fawkes::ROSAspect,
                            public fawkes::TransformAspect,
                            public fawkes::SyncPointManagerAspect,
                            public fawkes::ClockAspect
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	WorkpiecePoseThread();

	virtual void init() override;
	virtual void loop() override;
	virtual void finalize() override;

private:
	typedef pcl::PointXYZ          Point;
	typedef pcl::PointCloud<Point> Cloud;
	typedef Cloud::Ptr             CloudPtr;
	typedef Cloud::ConstPtr        CloudConstPtr;

	// cfg values
	std::string       cfg_if_prefix_;
	std::string       cloud_in_name_;
	const std::string cloud_out_raw_name_;
	const std::string cloud_out_trimmed_name_;
	std::string       cfg_bb_realsense_switch_name_;
	std::string       workpiece_frame_id_;
	float             cfg_voxel_grid_leaf_size_;

	CloudPtr trimmed_scene_;

	// state vars
	bool           cfg_enable_switch_;
	bool           cloud_in_registered_;
	pcl::PCLHeader input_pc_header_;

	// point clouds from pcl_manager
	fawkes::RefPtr<const Cloud> cloud_in_;

	// interfaces write
	fawkes::WorkpiecePoseInterface *wp_pose_;

	// interfaces read
	fawkes::SwitchInterface *realsense_switch_;
	fawkes::Time             wait_start_;
	fawkes::Time             wait_time_;

	float cloud_resolution() const;
	void  cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out);

	void bb_set_busy(bool busy);

	CloudPtr model_;
	CloudPtr scene_;

	fawkes::Mutex cloud_mutex_;
	fawkes::Mutex bb_mutex_;

	fawkes::RefPtr<Cloud> cloud_out_raw_;
	fawkes::RefPtr<Cloud> cloud_out_trimmed_;
	fawkes::RefPtr<Cloud> cloud_out_model_;

	std::unique_ptr<fawkes::tf::Stamped<fawkes::tf::Pose>> result_pose_;

	bool update_input_cloud();

	bool set_external_initial_tf(fawkes::tf::Stamped<fawkes::tf::Pose> &);

	CloudPtr cloud_trim(CloudPtr in);

	pcl::shared_ptr<std::vector<pcl::PointIndices>> cloud_cluster(CloudPtr in);
	CloudPtr                                        cloud_voxel_grid(CloudPtr in);

	void pose_write();
	void record_model();

	void pose_publish_tf(const fawkes::tf::Stamped<fawkes::tf::Pose> &pose);
	void start_waiting();
	bool need_to_wait();

protected:
	virtual void
	run() override
	{
		Thread::run();
	}
};

Eigen::Matrix4f  pose_to_eigen(const fawkes::tf::Pose &pose);
fawkes::tf::Pose eigen_to_pose(const Eigen::Matrix4f &m);

#endif // WORKPIECE_POSE_THREAD_H
