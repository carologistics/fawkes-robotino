#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/recognition/impl/hv/hv_papazov.hpp>

#include "recognition_thread.h"
#include "conveyor_pose_thread.h"

#include <utils/time/clock.h>

using namespace fawkes;


double CustomICP::getScaledFitness()
{
  double sum_dists = 0;
  for (const pcl::Correspondence &corr : *correspondences_) {
    if (corr.index_match >= 0) {
      //sum_dists += double(std::sqrt(corr.distance));
      sum_dists += 1 / (0.001 + double(std::sqrt(corr.distance)));
    }
  }
  return sum_dists;
  //return double(correspondences_->size()) / sum_dists;
}


RecognitionThread::RecognitionThread(ConveyorPoseThread *cp_thread)
  : Thread("CorrespondenceGrouping", Thread::OPMODE_CONTINUOUS)
  , TransformAspect(fawkes::TransformAspect::BOTH, "conveyor_pose_initial_guess")
  , main_thread_(cp_thread)
  , enabled_(false)
  , do_restart_(false)
{
  // Allow finalization while loop() is blocked
  set_prepfin_conc_loop(true);
}


void RecognitionThread::init()
{
}


void RecognitionThread::restart_icp()
{
  logger->log_info(name(), "Restarting ICP");

  do_restart_ = false;

  //model_with_normals_.reset(new Cloud());
  //scene_with_normals_.reset(new Cloud());
  tf::Stamped<tf::Pose> initial_pose_cam;

  { fawkes::MutexLocker locked { &main_thread_->cloud_mutex_ };
    std::vector<int> tmp;
    model_with_normals_ = main_thread_->model_;
    pcl::removeNaNFromPointCloud(*model_with_normals_, *model_with_normals_, tmp);
    scene_with_normals_ = main_thread_->scene_;
    pcl::removeNaNFromPointCloud(*scene_with_normals_, *scene_with_normals_, tmp);

    try {
      if (!main_thread_->have_laser_line_) {
        tf_listener->transform_pose(
              scene_with_normals_->header.frame_id,
              tf::Stamped<tf::Pose>(initial_guess_icp_odom_, Time(0,0), initial_guess_icp_odom_.frame_id),
              initial_pose_cam);
      }
      else {
        tf_listener->transform_pose(
              scene_with_normals_->header.frame_id,
              tf::Stamped<tf::Pose>(main_thread_->initial_guess_laser_odom_, Time(0,0),
                                    main_thread_->initial_guess_laser_odom_.frame_id),
              initial_pose_cam);
      }
    } catch (tf::TransformException &e) {
      logger->log_error(name(), e);
      enabled_ = false;
      return;
    }
  }

  hypot_verif_.setSceneCloud(scene_with_normals_);
  hypot_verif_.setResolution(main_thread_->cfg_voxel_grid_leaf_size_);
  hypot_verif_.setInlierThreshold(main_thread_->cfg_icp_hv_inlier_thresh_);
  hypot_verif_.setPenaltyThreshold(main_thread_->cfg_icp_hv_penalty_thresh_);
  hypot_verif_.setSupportThreshold(main_thread_->cfg_icp_hv_support_thresh_);

  initial_tf_ = pose_to_eigen(initial_pose_cam);

  tf_publisher->send_transform(
        tf::StampedTransform(
          initial_pose_cam, initial_pose_cam.stamp,
          initial_pose_cam.frame_id, "conveyor_pose_initial_guess"));

  icp_result_.reset(new Cloud());

  icp_ = CustomICP();

  // Set tunables
  icp_.setTransformationEpsilon(std::pow(main_thread_->cfg_icp_tf_epsilon_, 2));
  icp_.setMaxCorrespondenceDistance(double(main_thread_->cfg_icp_max_corr_dist_));
  icp_.setInputTarget(scene_with_normals_);
  icp_.setMaximumIterations(main_thread_->cfg_icp_max_iterations_);

  // Run first alignment with initial estimate
  icp_.setInputSource(model_with_normals_);
  icp_.align(*icp_result_, initial_tf_);
  final_tf_ = icp_.getFinalTransformation();

  iterations_ = 0;
  last_raw_fitness_ = std::numeric_limits<double>::max();
}


void RecognitionThread::loop()
{
  if (!enabled_) {
    if (!do_restart_) {
      logger->log_info(name(), "ICP stopped");

      while (!enabled_)
        wait_enabled_.wait();
    }

    restart_icp();
  }

  if (!enabled_) // cancel if disabled from ConveyorPoseThread
    return;

  icp_.setInputSource(icp_result_);
  icp_.align(*icp_result_);

  //accumulate transformation between each Iteration
  final_tf_ = icp_.getFinalTransformation() * final_tf_;

  if (!enabled_) // cancel if disabled from ConveyorPoseThread
    return;

  //if the difference between this transformation and the previous one
  //is smaller than the threshold, refine the process by reducing
  //the maximal correspondence distance
  bool epsilon_reached = false;
  if (double(std::abs((icp_.getLastIncrementalTransformation() - prev_last_tf_).sum())) < icp_.getTransformationEpsilon()) {
    icp_.setMaxCorrespondenceDistance(icp_.getMaxCorrespondenceDistance () * main_thread_->cfg_icp_refinement_factor_);
    epsilon_reached = true;
  }
  prev_last_tf_ = icp_.getLastIncrementalTransformation();

  if (iterations_++ >= main_thread_->cfg_icp_min_loops_ || epsilon_reached) {
    // Perform hypothesis verification
    std::vector<Cloud::ConstPtr> icp_result_vector { icp_result_ };
    hypot_verif_.setSceneCloud(scene_with_normals_);
    hypot_verif_.addModels(icp_result_vector, true);

    hypot_verif_.verify();
    std::vector<bool> hypot_mask;
    hypot_verif_.getMask(hypot_mask);

    if (hypot_mask[0] && icp_.getFitnessScore() < last_raw_fitness_) {
      // Match improved
      last_raw_fitness_ = icp_.getFitnessScore();
      publish_result();
      logger->log_info(name(), "FIT!");
    }

    if (iterations_ >= main_thread_->cfg_icp_max_loops_) {
      if (last_raw_fitness_ >= std::numeric_limits<double>::max() - 1) {
        logger->log_warn(name(), "No acceptable fit after %u iterations", iterations_);
        restart_icp();
      }
      else {
        if (main_thread_->cfg_icp_auto_restart_)
          restart_icp();
        else
          enabled_ = false;
      }
    }
  }

}


void RecognitionThread::publish_result()
{
  CloudPtr aligned_model(new Cloud());
  pcl::copyPointCloud(*icp_result_, *aligned_model);
  main_thread_->cloud_publish(aligned_model, main_thread_->cloud_out_model_);

  tf::Stamped<tf::Pose> result_pose {
    eigen_to_pose(final_tf_),
    Time { long(scene_with_normals_->header.stamp) / 1000 },
    scene_with_normals_->header.frame_id
  };

  // constrainTransformToGround(result_pose);

  /*double new_fitness = (1 / reg.getFitnessScore())
      / double(std::min(model_with_normals->size(), scene_with_normals->size()));*/
  double new_fitness = (1 / icp_.getFitnessScore()) / 10000;

  { MutexLocker locked2(&main_thread_->bb_mutex_);

    if (enabled_) {
      main_thread_->result_fitness_ = new_fitness;
      main_thread_->result_pose_.reset(new tf::Stamped<tf::Pose> { result_pose });

      try {
        tf_listener->transform_pose(
              "odom",
              tf::Stamped<tf::Pose>(result_pose, Time(0,0), result_pose.frame_id),
              initial_guess_icp_odom_);

      } catch(tf::TransformException &e) {
        logger->log_error(name(), e);
      }
    }
  } // MutexLocker
}

fawkes::tf::Quaternion constrainYtoNegZ(fawkes::tf::Quaternion rotation){
  fawkes::tf::Transform pose;
  pose.setRotation(rotation);
  fawkes::tf::Matrix3x3 basis = pose.getBasis();
  fawkes::tf::Vector3 yaxis_tf = basis.getColumn(1);
  fawkes::tf::Vector3 aligned_y(0, 0, -1);
  fawkes::tf::Scalar angle = yaxis_tf.angle(aligned_y);
  fawkes::tf::Vector3 rotational_axis = yaxis_tf.cross(aligned_y);
  rotational_axis.normalize();
  fawkes::tf::Quaternion resultRotation(rotational_axis, angle);
  resultRotation *= rotation;
  return resultRotation;
}

void RecognitionThread::constrainTransformToGround(fawkes::tf::Stamped<fawkes::tf::Pose>& fittedPose_conv){
  fawkes::tf::Stamped<fawkes::tf::Pose> fittedPose_base;
  tf_listener->transform_pose("base_link", fittedPose_conv, fittedPose_base);
  fittedPose_base.setRotation(constrainYtoNegZ(fittedPose_base.getRotation()));
  tf_listener->transform_pose(fittedPose_conv.frame_id, fittedPose_base, fittedPose_conv);
}

void RecognitionThread::enable()
{
  enabled_ = true;
  wait_enabled_.wake_all();
}


void RecognitionThread::disable()
{ enabled_ = false; }


void RecognitionThread::restart()
{
  disable();
  do_restart_ = true;
}


