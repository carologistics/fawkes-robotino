#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>

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
  : Thread("CorrespondenceGrouping", Thread::OPMODE_WAITFORWAKEUP)
  , TransformAspect(fawkes::TransformAspect::BOTH, "conveyor_pose_initial_guess")
  , main_thread_(cp_thread)
  , initial_guess_tracked_fitness_(std::numeric_limits<double>::min())
{
  // Enable finalization while blocked in loop()
  set_prepfin_conc_loop(true);

  // This thread is expected to be slow, so the main thread might try to wake it up more
  // than once before the loop() is done.
  set_coalesce_wakeups(true);
}


void RecognitionThread::init()
{
}



void RecognitionThread::loop()
{
  pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  Eigen::Matrix4f initial_tf;
  tf::Stamped<tf::Pose> initial_pose_cam;

  { fawkes::MutexLocker locked { &main_thread_->cloud_mutex_ };

    std::vector<int> tmp;
    pcl::removeNaNNormalsFromPointCloud(*main_thread_->model_with_normals_, *model_with_normals, tmp);
    pcl::removeNaNFromPointCloud(*model_with_normals, *model_with_normals, tmp);
    pcl::removeNaNNormalsFromPointCloud(*main_thread_->scene_with_normals_, *scene_with_normals, tmp);
    pcl::removeNaNFromPointCloud(*scene_with_normals, *scene_with_normals, tmp);
    scene_with_normals->header = main_thread_->scene_with_normals_->header;

    try {
      if (initial_guess_tracked_fitness_ >= main_thread_->cfg_icp_track_odom_min_fitness_
          || ( !main_thread_->have_laser_line_
               && initial_guess_tracked_fitness_ >= main_thread_->cfg_icp_track_odom_min_fitness_ / 2) ) {
        tf_listener->transform_pose(
              scene_with_normals->header.frame_id,
              tf::Stamped<tf::Pose>(initial_guess_icp_odom_, Time(0,0), initial_guess_icp_odom_.frame_id),
              initial_pose_cam);
      }
      else {
        if (main_thread_->have_laser_line_)
          tf_listener->transform_pose(
                scene_with_normals->header.frame_id,
                tf::Stamped<tf::Pose>(main_thread_->initial_guess_laser_odom_, Time(0,0),
                                      main_thread_->initial_guess_laser_odom_.frame_id),
                initial_pose_cam);
        else {
          logger->log_error(name(), "Cannot get initial guess: No laser line "
                                    "and no past results above fitness threshold");
          return;
        }
      }
    } catch (tf::TransformException &e) {
      logger->log_error(name(), e);
    }
  }

  initial_tf = pose_to_eigen(initial_pose_cam);

  tf_publisher->send_transform(
        tf::StampedTransform(
          initial_pose_cam, initial_pose_cam.stamp,
          initial_pose_cam.frame_id, "conveyor_pose_initial_guess"));

  MyPointRepresentation point_representation;
  // weigh the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  // Align
  CustomICP reg;
  reg.setTransformationEpsilon (1e-6);
  reg.setMaxCorrespondenceDistance (double(main_thread_->cfg_icp_max_corr_dist_));
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputTarget (scene_with_normals);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f prev;
  reg.setMaximumIterations (2);

  pcl::PointCloud<pcl::PointNormal>::Ptr reg_result(new pcl::PointCloud<pcl::PointNormal>());

  reg.setInputSource (model_with_normals);
  reg.align (*reg_result, initial_tf);

  for (int i = 0; i < 30; ++i) {
    model_with_normals = reg_result;

    // Estimate
    reg.setInputSource (model_with_normals);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () * 0.8);

    prev = reg.getLastIncrementalTransformation ();

    if (main_thread_->icp_cancelled_) {
      logger->log_info(name(), "ICP cancelled at iteration #%d", i);
      return;
    }
  }

  CloudPtr aligned_model(new Cloud());
  pcl::copyPointCloud(*reg_result, *aligned_model);
  main_thread_->cloud_publish(aligned_model, main_thread_->cloud_out_model_);

  tf::Stamped<tf::Pose> result_pose {
    eigen_to_pose(Ti * initial_tf),
    Time { long(scene_with_normals->header.stamp) / 1000 },
    scene_with_normals->header.frame_id
  };

  constrainTransformToGround(result_pose);

  /*double new_fitness = (1 / reg.getFitnessScore())
      / double(std::min(model_with_normals->size(), scene_with_normals->size()));*/
  double new_fitness = (1 / reg.getFitnessScore()) / 10000;

  { MutexLocker locked2(&main_thread_->bb_mutex_);
    if (!main_thread_->icp_cancelled_) {
      main_thread_->result_fitness_ = new_fitness;
      main_thread_->result_pose_.reset(new tf::Stamped<tf::Pose> { result_pose });
    }
  } // MutexLocker

  if (new_fitness > initial_guess_tracked_fitness_) {
    try {
      tf_listener->transform_pose(
            "odom",
            tf::Stamped<tf::Pose>(result_pose, Time(0,0), result_pose.frame_id),
            initial_guess_icp_odom_
      );
      initial_guess_tracked_fitness_ = new_fitness;

    } catch(tf::TransformException &e) {
      logger->log_error(name(), e);
    }
  }

}

void RecognitionThread::constrainTransformToGround(fawkes::tf::Stamped<fawkes::tf::Pose>& fittedPose_conv){
  // fawkes::tf::Stamped<fawkes::tf::Pose> fittedPose_base;
  // tf_listener->transform_pose("base_link", fittedPose_conv, fittedPose_base);
  // fawkes::tf::Matrix3x3 basis = fittedPose_base.getBasis();
  // // fawkes::tf::Vector3 yaxis_possibility_row = basis.getRow(1);
  // //printf("\nrow basis: %f/%f/%f\n", yaxis_possibility_row[0], yaxis_possibility_row[1], yaxis_possibility_row[2]);
  // fawkes::tf::Vector3 yaxis_tf = basis.getColumn(1);
  // //printf("col basis: %f/%f/%f\n", yaxis_tf[0], yaxis_tf[1], yaxis_tf[2]);
  // fawkes::tf::Vector3 aligned_y(0, 0, -1);
  // fawkes::tf::Scalar angle = yaxis_tf.angle(aligned_y);
  // fawkes::tf::Vector3 rotational_axis = yaxis_tf.cross(aligned_y);
  // rotational_axis.normalize();
  // fawkes::tf::Quaternion resultRotation(rotational_axis, angle);
  // fawkes::tf::Quaternion poseRotation = fittedPose_base.getRotation();
  // resultRotation *= poseRotation;
  // fittedPose_base.setRotation(resultRotation);
  //
  // //sanity check
  // basis = fittedPose_base.getBasis();
  // // yaxis_possibility_row = basis.getRow(1);
  // // printf("row basis: %f/%f/%f", yaxis_possibility_row[0], yaxis_possibility_row[1], yaxis_possibility_row[2]);
  // yaxis_tf = basis.getColumn(1);
  // printf("col basis result: %f/%f/%f\n", yaxis_tf[0], yaxis_tf[1], yaxis_tf[2]);
  //
  // // tf_listener->transform_pose(fittedPose_conv.frame_id, fittedPose_base, fittedPose_conv);
}

void MyPointRepresentation::copyToFloatArray (const pcl::PointNormal &p, float * out) const
{
  // < x, y, z, curvature >
  out[0] = p.x;
  out[1] = p.y;
  out[2] = p.z;
  out[3] = p.curvature;
}
