
#include <pcl/correspondence.h>
#include <pcl/features/board.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "correspondence_grouping_thread.h"
#include "conveyor_pose_thread.h"

CorrespondenceGroupingThread::CorrespondenceGroupingThread(ConveyorPoseThread *cp_thread)
  : Thread("CorrespondenceGrouping", Thread::OPMODE_WAITFORWAKEUP)
  , main_thread_(cp_thread)
{
  // Enable finalization while blocked in loop()
  set_prepfin_conc_loop(true);

  // This thread is expected to be slow, so the main thread might try to wake it up more
  // than once before the loop() is done.
  set_coalesce_wakeups(true);
}


void CorrespondenceGroupingThread::loop()
{
  CloudPtr scene = main_thread_->get_scene();
  main_thread_->norm_est_.setInputCloud(scene);
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>());
  main_thread_->norm_est_.compute (*scene_normals);

  main_thread_->uniform_sampling_.setInputCloud (scene);
  main_thread_->uniform_sampling_.setRadiusSearch (main_thread_->cfg_scene_ss_);
  CloudPtr scene_keypoints(new Cloud());
  main_thread_->uniform_sampling_.filter(*scene_keypoints);
  logger->log_debug(name(), "Scene total points: %zu, Selected Keypoints: %zu", scene->size(), scene_keypoints->size());

  main_thread_->descr_est_.setInputCloud (scene_keypoints);
  main_thread_->descr_est_.setInputNormals (scene_normals);
  main_thread_->descr_est_.setSearchSurface (scene);
  pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors (new pcl::PointCloud<pcl::SHOT352> ());
  main_thread_->descr_est_.compute (*scene_descriptors);

  /*if (!scene_descriptors->is_dense) {
    logger->log_error(name(), "Failed to compute scene descriptors");
    main_thread_->pose_add_element(false);
  }*/

  //  Find Model-Scene Correspondences with KdTree
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<pcl::SHOT352> match_search;
  match_search.setInputCloud (main_thread_->model_descriptors_);

  //  For each scene keypoint descriptor, find nearest neighbor into the model
  // keypoints descriptor cloud and add it to the correspondences vector.
  double descr_quality = 1;
  for (size_t i = 0; i < scene_descriptors->size (); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
      continue;
    int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < main_thread_->cfg_max_descr_dist_) {
      // add match only if the squared descriptor distance is less than 0.25
      // (SHOT descriptor distances are between 0 and 1 by design)
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
      descr_quality *= 1.0 - double(neigh_sqr_dists[0]);
    }
  }
  logger->log_debug(name(), "Correspondences found: %zu", model_scene_corrs->size());

  //  Actual Clustering
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (main_thread_->cfg_use_hough_) {
    //  Compute (Keypoints) Reference Frames only for Hough
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());

    pcl::BOARDLocalReferenceFrameEstimation<Point, pcl::Normal, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (main_thread_->cfg_rf_rad_);

    rf_est.setInputCloud (main_thread_->model_keypoints_);
    rf_est.setInputNormals (main_thread_->model_normals_);
    rf_est.setSearchSurface (main_thread_->model_);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<Point, Point, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize (main_thread_->cfg_cg_size_);
    clusterer.setHoughThreshold (main_thread_->cfg_cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (main_thread_->model_keypoints_);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
    gc_clusterer.setGCSize (main_thread_->cfg_cg_size_);
    gc_clusterer.setGCThreshold (main_thread_->cfg_cg_thresh_);

    gc_clusterer.setInputCloud (main_thread_->model_keypoints_);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  size_t corrs = 0;
  size_t max_corrs = 0;
  auto best_match = rototranslations.end();
  auto rit = rototranslations.begin();
  auto cit = clustered_corrs.begin();
  for ( ; rit < rototranslations.end() && cit < clustered_corrs.end(); ++rit, ++cit) {
    corrs = cit->size();
    if(corrs > max_corrs) {
      max_corrs = corrs;
      best_match = rit;
    }
    else break;
  }

  pose rv { false };
  if (best_match == rototranslations.end()) {
    main_thread_->pose_add_element(rv);
  }
  else {
    Eigen::Matrix4f &m = *best_match;
    rv.setOrigin( { double(m(0,3)), double(m(1,3)), double(m(2,3)) } );
    rv.setBasis( {
            double(m(0,0)), double(m(0,1)), double(m(0,2)),
            double(m(1,0)), double(m(1,1)), double(m(1,2)),
            double(m(2,0)), double(m(2,1)), double(m(2,2))
    } );
    logger->log_info(name(), "%f %f %f, max_corrs=%zu",
                     rv.getOrigin().getX(), rv.getOrigin().getY(), rv.getOrigin().getZ(),
                     max_corrs
                     );
    rv.valid = true;
    rv.quality = float(descr_quality * model_scene_corrs->size());
    main_thread_->pose_add_element(rv);
  }
}
