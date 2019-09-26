/***************************************************************************
 *  recognition_thread.h - Align scene to model & publish the TF
 *
 *  Created: Tue Apr 10 15:41:58 2018 +0200
 *  Copyright  2018 Victor Mataré
 *             2018 Morian Sonnet
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

#ifndef CORRESPONDENCE_GROUPING_THREAD_H
#define CORRESPONDENCE_GROUPING_THREAD_H

#include "conveyor_pose_thread.h"

#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/syncpoint_manager.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/registration/icp_nl.h>

#include <atomic>

/** Derive from the ICP algorithm in case we want to override something */
class CustomICP : public pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>
{
public:
	using pcl::IterativeClosestPointNonLinear<pcl::PointXYZ,
	                                          pcl::PointXYZ>::IterativeClosestPointNonLinear;

	/** @return A custom fitness measure of the computed fit */
	double getScaledFitness();
};

/** Run ICP, perform HypothesisVerification and publish the result */
class RecognitionThread : public fawkes::Thread,
                          public fawkes::LoggingAspect,
                          public fawkes::TransformAspect,
                          public fawkes::SyncPointManagerAspect,
                          public fawkes::ClockAspect
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** RecognitionThread constructor
   * @param cp_thread pointer back to the main thread
   */
	RecognitionThread(ConveyorPoseThread *cp_thread);

	virtual void loop() override;
	virtual void init() override;

	/** Continue running this thread */
	void enable();

	/** Stop running this thread */
	void disable();

	/** Re-initialize this thread, e.g. after a config change */
	void schedule_restart();

	/** @return whether this thread is currently enabled (running) */
	bool enabled();

private:
	friend ConveyorPoseThread;

	void restart_icp();
	void publish_result();
	void constrainTransformToGround(fawkes::tf::Stamped<fawkes::tf::Pose> &fittedPose_conv);

	using Point    = ConveyorPoseThread::Point;
	using Cloud    = ConveyorPoseThread::Cloud;
	using CloudPtr = ConveyorPoseThread::CloudPtr;

	ConveyorPoseThread *main_thread_;

	fawkes::RefPtr<fawkes::SyncPoint> syncpoint_ready_for_icp_;

	std::atomic_bool enabled_;
	std::atomic_bool restart_pending_;

	Eigen::Matrix4f initial_tf_;
	CloudPtr        model_;
	CloudPtr        scene_;

	CustomICP                    icp_;
	pcl::PapazovHV<Point, Point> hypot_verif_;
	CloudPtr                     icp_result_;
	Eigen::Matrix4f              final_tf_;

	unsigned int iterations_;
	unsigned int retries_;
	double       last_raw_fitness_;

	std::atomic<float>                cfg_icp_max_corr_dist_;
	std::atomic<double>               cfg_icp_min_corr_dist_;
	std::atomic<double>               cfg_icp_tf_epsilon_;
	std::atomic<double>               cfg_icp_refinement_factor_;
	std::array<std::atomic<float>, 3> cfg_icp_conveyor_hint_;
	std::atomic<int>                  cfg_icp_max_iterations_;
	std::atomic<float>                cfg_icp_hv_penalty_thresh_;
	std::atomic<float>                cfg_icp_hv_support_thresh_;
	std::atomic<float>                cfg_icp_hv_inlier_thresh_;
	std::atomic<float>                cfg_icp_shelf_hv_penalty_thresh_;
	std::atomic<float>                cfg_icp_shelf_hv_support_thresh_;
	std::atomic<float>                cfg_icp_shelf_hv_inlier_thresh_;
	std::atomic<unsigned int>         cfg_icp_min_loops_;
	std::atomic<unsigned int>         cfg_icp_max_loops_;
	std::atomic<unsigned int>         cfg_icp_max_retries_;
	std::atomic_bool                  cfg_icp_auto_restart_;
};

#endif // CORRESPONDENCE_GROUPING_THREAD_H
