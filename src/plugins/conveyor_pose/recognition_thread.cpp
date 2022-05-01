/***************************************************************************
 *  recognition_thread.cpp - Align scene to model & publish the TF
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

#include "recognition_thread.h"

#include "conveyor_pose_thread.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <utils/time/clock.h>

#include <pcl/recognition/impl/hv/hv_papazov.hpp>

using namespace fawkes;

double
CustomICP::getScaledFitness()
{
	double sum_dists = 0;
	for (const pcl::Correspondence &corr : *correspondences_) {
		if (corr.index_match >= 0) {
			sum_dists += 1 / (0.001 + double(std::sqrt(corr.distance)));
		}
	}
	return sum_dists;
}

RecognitionThread::RecognitionThread(ConveyorPoseThread *cp_thread)
: Thread("ICPThread", Thread::OPMODE_CONTINUOUS),
  TransformAspect(fawkes::TransformAspect::BOTH, "conveyor_pose_initial_guess"),
  main_thread_(cp_thread),
  enabled_(false),
  restart_pending_(true)
{
	// Allow finalization while loop() is blocked
	set_prepfin_conc_loop(true);
}

void
RecognitionThread::init()
{
	syncpoint_ready_for_icp_ =
	  syncpoint_manager->get_syncpoint(name(), main_thread_->syncpoint_ready_for_icp_name_);
}

void
RecognitionThread::restart_icp()
{
	logger->log_info(name(), "Restarting ICP");

	syncpoint_ready_for_icp_->wait(name());

	{
		fawkes::MutexLocker locked{&main_thread_->bb_mutex_};
		main_thread_->bb_set_busy(true);
	}

	tf::Stamped<tf::Pose> initial_pose_cam = main_thread_->initial_guess_odom_;

	{
		fawkes::MutexLocker locked{&main_thread_->cloud_mutex_};
		std::vector<int>    tmp;
		model_ = main_thread_->model_;
		pcl::removeNaNFromPointCloud(*model_, *model_, tmp);
		scene_ = main_thread_->scene_;
		pcl::removeNaNFromPointCloud(*scene_, *scene_, tmp);

		model_->header.frame_id = scene_->header.frame_id;

		try {
			tf_listener->transform_pose(scene_->header.frame_id,
			                            tf::Stamped<tf::Pose>(main_thread_->initial_guess_odom_,
			                                                  Time(0, 0),
			                                                  main_thread_->initial_guess_odom_.frame_id),
			                            initial_pose_cam);

		} catch (tf::TransformException &e) {
			//-- exit alignment if transformation is not possile
			logger->log_error(name(),
			                  "Failed to transform initial guess from %s to %s",
			                  main_thread_->initial_guess_odom_.frame_id.c_str(),
			                  scene_->header.frame_id.c_str());
			logger->log_error(name(), e);
			restart_pending_ = true;
			return;
		}
	}

	hypot_verif_.setSceneCloud(scene_);
	hypot_verif_.setResolution(main_thread_->cloud_resolution());
	if (main_thread_->is_target_shelf()) { // use shelf values
		hypot_verif_.setInlierThreshold(cfg_icp_shelf_hv_inlier_thresh_);
		hypot_verif_.setPenaltyThreshold(cfg_icp_shelf_hv_penalty_thresh_);
		hypot_verif_.setSupportThreshold(cfg_icp_shelf_hv_support_thresh_);
	} else { // use general values
		hypot_verif_.setInlierThreshold(cfg_icp_hv_inlier_thresh_);
		hypot_verif_.setPenaltyThreshold(cfg_icp_hv_penalty_thresh_);
		hypot_verif_.setSupportThreshold(cfg_icp_hv_support_thresh_);
	}

	initial_tf_ = pose_to_eigen(initial_pose_cam);

	tf_publisher->send_transform(tf::StampedTransform(initial_pose_cam,
	                                                  initial_pose_cam.stamp,
	                                                  initial_pose_cam.frame_id,
	                                                  "conveyor_pose_initial_guess"));

	icp_result_.reset(new Cloud());
	icp_result_->header = model_->header;

	icp_ = CustomICP();

	// Set tunables
	icp_.setTransformationEpsilon(std::pow(cfg_icp_tf_epsilon_, 2));
	icp_.setMaxCorrespondenceDistance(double(cfg_icp_max_corr_dist_));
	icp_.setInputTarget(scene_);
	icp_.setMaximumIterations(cfg_icp_max_iterations_);

	// Run first alignment with initial estimate
	icp_.setInputSource(model_);
	icp_.align(*icp_result_, initial_tf_);
	final_tf_ = icp_.getFinalTransformation();

	iterations_       = 0;
	last_raw_fitness_ = std::numeric_limits<double>::max();

	restart_pending_ = false;
}

void
RecognitionThread::loop()
{
	if (!enabled_) {
		logger->log_info(name(), "ICP stopped");

		{
			MutexLocker locked{&main_thread_->bb_mutex_};
			main_thread_->bb_set_busy(false);
		}

		while (!enabled_)
			syncpoint_ready_for_icp_->wait(name());
	}

	if (restart_pending_) {
		restart_icp();
		return;
	}

	if (!enabled_ || restart_pending_) // cancel if disabled from ConveyorPoseThread
		return;

	icp_.setInputSource(icp_result_);
	icp_.align(*icp_result_);

	// accumulate transformation between each Iteration
	final_tf_ = icp_.getFinalTransformation() * final_tf_;

	if (!enabled_ || restart_pending_) // cancel if disabled from ConveyorPoseThread
		return;

	// if the difference between this transformation and the previous one
	// is smaller than the threshold, refine the process by reducing
	// the maximal correspondence distance
	bool   epsilon_reached = false;
	double last_tf_len     = eigen_to_pose(icp_.getFinalTransformation()).getOrigin().length();
	if (last_tf_len * last_tf_len < icp_.getTransformationEpsilon()) {
		icp_.setMaxCorrespondenceDistance(icp_.getMaxCorrespondenceDistance()
		                                  * cfg_icp_refinement_factor_);
		epsilon_reached = true;
		logger->log_info(name(),
		                 "Epsilon reached. Max.corr.dist: %f, last_tf_len: %.14f",
		                 icp_.getMaxCorrespondenceDistance(),
		                 last_tf_len);
	} else
		logger->log_info(name(), "last_tf_len: %.9f", last_tf_len);

	if (enabled_ && main_thread_->cfg_debug_mode_)
		publish_result();

	if (iterations_++ >= cfg_icp_min_loops_ || epsilon_reached) {
		// Perform hypothesis verification, i.e. skip results that don't match
		// closely enough according to certain thresholds. See the config file for
		// an explanation of the individual parameters.
		std::vector<Cloud::ConstPtr> icp_result_vector{icp_result_};

		// This resets the result vectors (see hypot_mask below) so we only ever
		// verify the last ICP result
		hypot_verif_.setSceneCloud(scene_);
		hypot_verif_.addModels(icp_result_vector, true);

		hypot_verif_.verify();
		std::vector<bool> hypot_mask;
		hypot_verif_.getMask(hypot_mask);

		if (hypot_mask[0] && icp_.getFitnessScore() < last_raw_fitness_) {
			// Hypothesis verification was successful and and the match improved
			last_raw_fitness_ = icp_.getFitnessScore();

			// In debug mode, publish_result has already been called (see above)
			if (enabled_ && !main_thread_->cfg_debug_mode_)
				publish_result();
		}

		if (icp_.getMaxCorrespondenceDistance() < cfg_icp_min_corr_dist_
		    || iterations_ >= cfg_icp_max_loops_) {
			if (last_raw_fitness_ >= std::numeric_limits<double>::max() - 1) {
				logger->log_warn(name(), "No acceptable fit after %u iterations", iterations_);
				if (cfg_icp_auto_restart_ || retries_++ <= cfg_icp_max_retries_)
					restart_icp();
				else {
					logger->log_warn(name(), "Giving up after %d retries", retries_);
					disable();
				}
			} else {
				if (cfg_icp_auto_restart_)
					restart_icp();
				else
					enabled_ = false;
			}
		}
	}
}

void
RecognitionThread::publish_result()
{
	CloudPtr aligned_model(new Cloud());
	pcl::copyPointCloud(*icp_result_, *aligned_model);
	aligned_model->header = icp_result_->header;
	main_thread_->cloud_publish(aligned_model, main_thread_->cloud_out_model_);

	tf::Stamped<tf::Pose> result_pose{eigen_to_pose(final_tf_),
	                                  fawkes::Time(0, 0)
	                                    + static_cast<long int>(scene_->header.stamp),
	                                  scene_->header.frame_id};

	double new_fitness = (1 / icp_.getFitnessScore()) / 10000;

	{
		MutexLocker locked2(&main_thread_->bb_mutex_);

		if (enabled_) {
			main_thread_->result_fitness_ = new_fitness;
			main_thread_->result_pose_.reset(new tf::Stamped<tf::Pose>{result_pose});
			main_thread_->result_pose_->setRotation(main_thread_->result_pose_->getRotation()
			                                        * tf::Quaternion({1, 0, 0}, -M_PI_2)
			                                        * tf::Quaternion({0, 0, 1}, -M_PI_2));
		}
	} // MutexLocker
}

fawkes::tf::Quaternion
constrainYtoNegZ(fawkes::tf::Quaternion rotation)
{
	fawkes::tf::Transform pose;
	pose.setRotation(rotation);
	fawkes::tf::Matrix3x3 basis    = pose.getBasis();
	fawkes::tf::Vector3   yaxis_tf = basis.getColumn(1);
	fawkes::tf::Vector3   aligned_y(0, 0, -1);
	fawkes::tf::Scalar    angle           = yaxis_tf.angle(aligned_y);
	fawkes::tf::Vector3   rotational_axis = yaxis_tf.cross(aligned_y);
	rotational_axis.normalize();
	fawkes::tf::Quaternion resultRotation(rotational_axis, angle);
	resultRotation *= rotation;
	return resultRotation;
}

void
RecognitionThread::constrainTransformToGround(
  fawkes::tf::Stamped<fawkes::tf::Pose> &fittedPose_conv)
{
	fawkes::tf::Stamped<fawkes::tf::Pose> fittedPose_base;
	tf_listener->transform_pose("base_link", fittedPose_conv, fittedPose_base);
	fittedPose_base.setRotation(constrainYtoNegZ(fittedPose_base.getRotation()));
	tf_listener->transform_pose(fittedPose_conv.frame_id, fittedPose_base, fittedPose_conv);
}

void
RecognitionThread::enable()
{
	enabled_ = true;
}

void
RecognitionThread::disable()
{
	enabled_ = false;
}

void
RecognitionThread::schedule_restart()
{
	restart_pending_ = true;
	enable();
}

bool
RecognitionThread::enabled()
{
	return enabled_;
}
