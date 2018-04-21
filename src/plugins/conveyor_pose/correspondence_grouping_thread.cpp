#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>

#include "correspondence_grouping_thread.h"
#include "conveyor_pose_thread.h"

#include <utils/time/clock.h>

using namespace fawkes;

RecognitionThread::RecognitionThread(ConveyorPoseThread *cp_thread)
  : Thread("CorrespondenceGrouping", Thread::OPMODE_WAITFORWAKEUP)
  , main_thread_(cp_thread)
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
  CloudPtr aligned_model(new Cloud());
  Eigen::Matrix4f initial_tf;

  { fawkes::MutexLocker locked { &main_thread_->cloud_mutex_ };

    std::vector<int> tmp;
    pcl::removeNaNNormalsFromPointCloud(*main_thread_->model_with_normals_, *model_with_normals, tmp);
    if (tmp.size())
      logger->log_warn(name(), "Removed %zu NaNNormals from model", tmp.size());
    tmp.clear();
    pcl::removeNaNFromPointCloud(*model_with_normals, *model_with_normals, tmp);
    if (tmp.size())
      logger->log_warn(name(), "Removed %zu NaNs from model", tmp.size());
    tmp.clear();

    pcl::removeNaNNormalsFromPointCloud(*main_thread_->scene_with_normals_, *scene_with_normals, tmp);
    if (tmp.size())
      logger->log_warn(name(), "Removed %zu NaNNormalss from scene", tmp.size());
    tmp.clear();
    pcl::removeNaNFromPointCloud(*scene_with_normals, *scene_with_normals, tmp);
    if (tmp.size())
      logger->log_warn(name(), "Removed %zu NaNs from scene", tmp.size());
    tmp.clear();

    initial_tf = main_thread_->initial_tf_;

    /*pcl::transformPointCloud(*main_thread_->model_, *aligned_model, initial_tf);
    main_thread_->cloud_publish(aligned_model, main_thread_->cloud_out_model_);*/

    MyPointRepresentation point_representation;
    // weigh the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    // Align
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
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
    }


    pcl::copyPointCloud(*model_with_normals, *aligned_model);

    Eigen::Matrix4f m = Ti * initial_tf;

    double new_fitness = (1 / reg.getFitnessScore())
        / double(std::min(model_with_normals->size(), scene_with_normals->size()));

    main_thread_->cloud_publish(aligned_model, main_thread_->cloud_out_model_);
    MutexLocker locked2(&main_thread_->pose_mutex_);
    main_thread_->result_fitness_ = new_fitness;
    main_thread_->result_pose_.setOrigin( { double(m(0,3)), double(m(1,3)), double(m(2,3)) } );
    main_thread_->result_pose_.setBasis( {
                                           double(m(0,0)), double(m(0,1)), double(m(0,2)),
                                           double(m(1,0)), double(m(1,1)), double(m(1,2)),
                                           double(m(2,0)), double(m(2,1)), double(m(2,2))
                                         } );
    main_thread_->result_pose_.frame_id = scene_with_normals->header.frame_id;
    main_thread_->result_pose_.stamp = Time { static_cast<long>(scene_with_normals->header.stamp) };

    main_thread_->pose_write();
    main_thread_->pose_publish_tf(main_thread_->result_pose_);
  }
}


void MyPointRepresentation::copyToFloatArray (const pcl::PointNormal &p, float * out) const
{
  // < x, y, z, curvature >
  out[0] = p.x;
  out[1] = p.y;
  out[2] = p.z;
  out[3] = p.curvature;
}
