/***************************************************************************
 *  conveyor_plane_thread.cpp - conveyor_plane thread
 *
 *  Created: Thr 12. April 16:28:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
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

#include "conveyor_plane_thread.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/distances.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/types.h>
#include <utils/math/angle.h>

#include <cmath>

using namespace fawkes;

/** @class ConveyorPlaneThread "conveyor_plane_thread.cpp"
 * Plugin to detect the conveyor belt in a pointcloud (captured from Intel
 * RealSense)
 * @author Tobias Neumann
 */

/** Constructor. */
ConveyorPlaneThread::ConveyorPlaneThread()
: Thread("ConveyorPlaneThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  fawkes::TransformAspect(fawkes::TransformAspect::BOTH, "conveyor_plane"),
  realsense_switch_(NULL)
{
}

void
ConveyorPlaneThread::init()
{
	const std::string cfg_prefix = "/conveyor_plane/";

	cfg_debug_mode_ = config->get_bool((cfg_prefix + "debug").c_str());

	cloud_in_name_ = config->get_string((cfg_prefix + "cloud_in").c_str());

	const std::string if_prefix = config->get_string((cfg_prefix + "if/prefix").c_str()) + "/";

	cloud_out_inter_1_name_ =
	  if_prefix + config->get_string((cfg_prefix + "if/cloud_out_intermediet").c_str());
	cloud_out_result_name_ =
	  if_prefix + config->get_string((cfg_prefix + "if/cloud_out_result").c_str());
	cfg_bb_conveyor_plane_name_ =
	  if_prefix + config->get_string((cfg_prefix + "if/pose_of_beld").c_str());
	cfg_bb_switch_name_ = if_prefix + config->get_string((cfg_prefix + "if/switch").c_str());

	laserlines_names_ = config->get_strings((cfg_prefix + "if/laser_lines").c_str());

	cfg_pose_close_if_no_new_pointclouds_ =
	  config->get_bool((cfg_prefix + "if/pose_close_if_new_pc").c_str());

	conveyor_frame_id_      = config->get_string((cfg_prefix + "conveyor_frame_id").c_str());
	cfg_pose_diff_          = config->get_float((cfg_prefix + "vis_hist/diff_pose").c_str());
	vis_hist_angle_diff_    = config->get_float((cfg_prefix + "vis_hist/diff_angle").c_str());
	cfg_pose_avg_hist_size_ = config->get_uint((cfg_prefix + "vis_hist/average/size").c_str());
	cfg_pose_avg_min_       = config->get_uint((cfg_prefix + "vis_hist/average/used_min").c_str());
	cfg_allow_invalid_poses_ =
	  config->get_uint((cfg_prefix + "vis_hist/allow_invalid_poses").c_str());

	cfg_enable_switch_     = config->get_bool((cfg_prefix + "switch_default").c_str());
	cfg_use_visualisation_ = config->get_bool((cfg_prefix + "use_visualisation").c_str());

	cfg_crop_cam_x_min = config->get_float((cfg_prefix + "crop/cam/x_min").c_str());
	cfg_crop_cam_x_max = config->get_float((cfg_prefix + "crop/cam/x_max").c_str());
	cfg_crop_cam_y_min = config->get_float((cfg_prefix + "crop/cam/y_min").c_str());
	cfg_crop_cam_y_max = config->get_float((cfg_prefix + "crop/cam/y_max").c_str());
	cfg_crop_cam_z_min = config->get_float((cfg_prefix + "crop/cam/z_min").c_str());
	cfg_crop_cam_z_max = config->get_float((cfg_prefix + "crop/cam/z_max").c_str());
	cfg_mps_top_offset = config->get_float((cfg_prefix + "crop/laserline/mps_top_offset").c_str());
	cfg_crop_laserline_x_min = config->get_float((cfg_prefix + "crop/laserline/x_min").c_str());
	cfg_crop_laserline_x_max = config->get_float((cfg_prefix + "crop/laserline/x_max").c_str());
	cfg_crop_laserline_y_min = config->get_float((cfg_prefix + "crop/laserline/y_min").c_str());
	cfg_crop_laserline_y_max = config->get_float((cfg_prefix + "crop/laserline/y_max").c_str());
	cfg_crop_laserline_z_min = config->get_float((cfg_prefix + "crop/laserline/z_min").c_str());
	cfg_crop_laserline_z_max = config->get_float((cfg_prefix + "crop/laserline/z_max").c_str());

	cfg_plane_dist_threshold_ = config->get_float((cfg_prefix + "plane/dist_threshold").c_str());
	cfg_normal_z_minimum_     = config->get_float((cfg_prefix + "plane/normal_z_minimum").c_str());
	cfg_plane_height_minimum_ = config->get_float((cfg_prefix + "plane/height_minimum").c_str());
	cfg_plane_width_minimum_  = config->get_float((cfg_prefix + "plane/width_minimum").c_str());

	cfg_cluster_tolerance_ = config->get_float((cfg_prefix + "cluster/tolerance").c_str());
	cfg_cluster_size_min_  = config->get_float((cfg_prefix + "cluster/size_min").c_str());
	cfg_cluster_size_max_  = config->get_float((cfg_prefix + "cluster/size_max").c_str());

	cfg_voxel_grid_leave_size_ = config->get_float((cfg_prefix + "voxel_grid/leave_size").c_str());

	cfg_bb_realsense_switch_name_ = "realsense";
	try {
		cfg_bb_realsense_switch_name_ = config->get_string((cfg_prefix + "realsense_switch").c_str());
	} catch (Exception &e) {
	} // ignore, use default
	try {
		wait_time_ = Time(config->get_float((cfg_prefix + "realsense_wait_time").c_str()));
	} catch (Exception &e) {
		wait_time_ = Time(1.);
	} // ignore, use default

	cloud_in_registered_ = false;

	cloud_out_inter_1_ = new Cloud();
	cloud_out_result_  = new Cloud();
	pcl_manager->add_pointcloud(cloud_out_inter_1_name_.c_str(), cloud_out_inter_1_);
	pcl_manager->add_pointcloud(cloud_out_result_name_.c_str(), cloud_out_result_);

	for (std::string ll : laserlines_names_) {
		laserlines_.push_back(blackboard->open_for_reading<fawkes::LaserLineInterface>(ll.c_str()));
	}

	enable_pose_ = false;
	if (!cfg_pose_close_if_no_new_pointclouds_) {
		bb_pose_conditional_open();
	}

	bb_enable_switch_ = blackboard->open_for_writing<SwitchInterface>(cfg_bb_switch_name_.c_str());
	bb_enable_switch_->set_enabled(
	  cfg_debug_mode_ || cfg_enable_switch_); // ignore cfg_enable_switch_ and set to true if debug
	                                          // mode is used
	bb_enable_switch_->write();

	realsense_switch_ =
	  blackboard->open_for_reading<SwitchInterface>(cfg_bb_realsense_switch_name_.c_str());

	visualisation_ = new Visualisation(node_handle);
}

void
ConveyorPlaneThread::finalize()
{
	pcl_manager->remove_pointcloud(cloud_out_inter_1_name_.c_str());
	pcl_manager->remove_pointcloud(cloud_out_result_name_.c_str());
	delete visualisation_;
	blackboard->close(bb_enable_switch_);
	logger->log_info(name(), "Unloading, disabling %s", cfg_bb_realsense_switch_name_.c_str());
	blackboard->close(realsense_switch_);
	bb_pose_conditional_close();
}

void
ConveyorPlaneThread::bb_pose_conditional_open()
{
	if (!enable_pose_) {
		enable_pose_ = true;
		bb_pose_     = blackboard->open_for_writing<fawkes::Position3DInterface>(
      cfg_bb_conveyor_plane_name_.c_str());
	}
}

void
ConveyorPlaneThread::bb_pose_conditional_close()
{
	if (enable_pose_) {
		enable_pose_ = false;
		blackboard->close(bb_pose_);
	}
}

void
ConveyorPlaneThread::loop()
{
	//-- reads message queue and performs en- and disablings
	if_read(); //-- if_read also reads the laserlines

	//-- skip processing if conveyor plane is disabled
	if (!bb_enable_switch_->is_enabled()) {
		return;
	}

	//-- skip processing if camera is not enabled
	realsense_switch_->read();
	if (!realsense_switch_->is_enabled()) {
		logger->log_info(name(), "Camera %s is disabled", cfg_bb_realsense_switch_name_.c_str());
		start_waiting();
		return;
	}

	if (!pc_in_check() || !bb_enable_switch_->is_enabled()) {
		if (enable_pose_) {
			vis_hist_ = -1;
			pose trash;
			trash.valid = false;
			pose_write(trash);
		}
		if (cfg_pose_close_if_no_new_pointclouds_) {
			bb_pose_conditional_close();
		}
		return;
	}
	if (need_to_wait()) {
		logger->log_debug(name(),
		                  "Waiting for %s for %f sec, still %f sec remaining",
		                  cfg_bb_realsense_switch_name_.c_str(),
		                  wait_time_.in_sec(),
		                  (wait_start_ + wait_time_ - Time()).in_sec());
		return;
	}

	//-- if it are enough planes collected to decide whether the avg is valid or
	//not...
	if (poses_.size() >= cfg_allow_invalid_poses_) {
		bb_pose_conditional_open();

		pose pose_average;
		bool pose_average_availabe = pose_get_avg(pose_average);

		if (pose_average_availabe) {
			vis_hist_ = std::max(1, vis_hist_ + 1);

			//-- NOTE: THIS IS HARD CODED SHIFTIGN DIREICTION ALONG THE POSITIVE
			//Y-AXIS,
			//--       SPACIFIED BY THE REALSSENSE MOUNTING.
			//--       I'M SURE THIS COULD AND SHOULD BE DONE MORE FLEXIBLE, BUT I'VE
			//NO IDEA ATM
			//-- shift pose to the top conveyor belt
			pose_average.translation += tf::Vector3(0.f, cfg_plane_height_minimum_ * 0.85f, 0.f);

			pose_write(pose_average);

			pose_publish_tf(pose_average);

			//    tf_send_from_pose_if(pose_current);
			if (cfg_use_visualisation_) {
				visualisation_->marker_draw(header_, pose_average.translation, pose_average.rotation);
			}
		} else {
			vis_hist_ = -1;
			pose trash;
			trash.valid = false;
			pose_write(trash);
			poses_.clear();
		}
	}

	fawkes::LaserLineInterface *ll = NULL;

	bool use_laserline = false;
	try {
		use_laserline = laserline_get_best_fit(ll);
	} catch (tf::TransformException &e) {
		logger->log_error(name(), "Failed to transform laser line: %s", e.what());
	}

	CloudPtr cloud_in(new Cloud(**cloud_in_));

	//-- apply voxelgrid filter
	uint in_size = cloud_in->points.size();

	CloudPtr cloud_vg = cloud_voxel_grid(cloud_in);
	uint     out_size = cloud_vg->points.size();

	if (in_size == out_size) {
		logger->log_error(name(), "Voxel Grid failed, skipping loop!");
		return;
	}

	//-- crop pointcloud
	CloudPtr cloud_cropped(new Cloud);
	cloud_cropped = crop_cloud(cloud_vg, ll, use_laserline);

	//-- publish filter result
	cloud_publish(cloud_cropped, cloud_out_inter_1_);

	// search for best plane
	CloudPtr                    cloud_choosen;
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	do {
		CloudPtr cloud_plane = cloud_get_plane(cloud_cropped, coeff);

		//-- if there was no suffitient planar patch found ...
		if (cloud_plane == NULL || !cloud_plane) {
			//-- ... add as invalid plane
			Eigen::Vector4f invalid(std::numeric_limits<float>::quiet_NaN(),
			                        std::numeric_limits<float>::quiet_NaN(),
			                        std::numeric_limits<float>::quiet_NaN(),
			                        std::numeric_limits<float>::quiet_NaN());
			pose_add_element(invalid, invalid.head<3>());
			return;
		}

		//-- separate connected components laying in the plane
		size_t                                          id;
		pcl::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices = cloud_cluster(cloud_plane);

		//-- if even there were no sufficient connected components on the found
		// surface ...
		if (cluster_indices->size() <= 0) {
			//-- ... add as invalid plane
			Eigen::Vector4f invalid(std::numeric_limits<float>::quiet_NaN(),
			                        std::numeric_limits<float>::quiet_NaN(),
			                        std::numeric_limits<float>::quiet_NaN(),
			                        std::numeric_limits<float>::quiet_NaN());
			pose_add_element(invalid, invalid.head<3>());
			return;
		}

		//-- get biggest connected planar component
		std::vector<CloudPtr> clouds_cluster = cluster_split(cloud_plane, cluster_indices);

		cloud_choosen = cluster_find_biggest(clouds_cluster, id);

		// check if the height is ok (remove shelfs)
		float y_min = -200;
		float y_max = 200;
		for (Point p : *cloud_choosen) {
			if (p.y > y_min) {
				y_min = p.y;
			}
			if (p.y < y_max) {
				y_max = p.y;
			}
		}

		float height = y_min - y_max;
		if (height < cfg_plane_height_minimum_) {
			logger->log_debug(name(),
			                  "Discard plane, because of height restriction. is: %f\tshould: %f",
			                  height,
			                  cfg_plane_height_minimum_);

			pcl::shared_ptr<pcl::PointIndices> extract_indicies(
			  new pcl::PointIndices(cluster_indices->at(id)));
			CloudPtr                   tmp(new Cloud);
			pcl::ExtractIndices<Point> extract;
			extract.setInputCloud(cloud_cropped);
			extract.setIndices(extract_indicies);
			extract.setNegative(true);
			extract.filter(*tmp);
			// logger->log_debug(name(), "After extraction");
			*cloud_cropped = *tmp;
		} else {
			// height is ok
			float x_min = -200;
			float x_max = 200;
			for (Point p : *cloud_choosen) {
				if (p.x > x_min) {
					x_min = p.x;
				}
				if (p.x < x_max) {
					x_max = p.x;
				}
			}

			float width = x_min - x_max;
			if (width < cfg_plane_width_minimum_) {
				logger->log_debug(name(),
				                  "Discard plane, because of width restriction. is: %f\tshould: %f",
				                  width,
				                  cfg_plane_width_minimum_);
				pcl::shared_ptr<pcl::PointIndices> extract_indicies(
				  new pcl::PointIndices(cluster_indices->at(id)));
				CloudPtr                   tmp(new Cloud);
				pcl::ExtractIndices<Point> extract;
				extract.setInputCloud(cloud_cropped);
				extract.setIndices(extract_indicies);
				extract.setNegative(true);
				extract.filter(*tmp);
				*cloud_cropped = *tmp;

			} else {
				// height and width ok
				break;
			}
		}
	} while (true);

	// logger->log_debug(name(),"CONVEYOR-POSE 9: left while true");
	cloud_publish(cloud_choosen, cloud_out_result_);

	// get centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid<Point, float>(*cloud_choosen, centroid);

	Eigen::Vector3f normal;
	normal(0) = coeff->values[0];
	normal(1) = coeff->values[1];
	normal(2) = coeff->values[2];

	pose_add_element(centroid, normal);
}

bool
ConveyorPlaneThread::pc_in_check()
{
	if (pcl_manager->exists_pointcloud(cloud_in_name_.c_str())) { // does the pc exists
		if (!cloud_in_registered_) {                                // do I already have this pc
			cloud_in_ = pcl_manager->get_pointcloud<Point>(cloud_in_name_.c_str());
			if (cloud_in_->points.size() > 0) {
				cloud_in_registered_ = true;
			}
		}

		unsigned long time_old = header_.stamp;
		header_                = cloud_in_->header;

		return time_old != header_.stamp; // true, if there is a new cloud, false otherwise

	} else {
		logger->log_debug(name(), "can't get pointcloud %s", cloud_in_name_.c_str());
		cloud_in_registered_ = false;
		return false;
	}
}

void
ConveyorPlaneThread::if_read()
{
	// enable switch
	bb_enable_switch_->read();

	bool rv = bb_enable_switch_->is_enabled();
	while (!bb_enable_switch_->msgq_empty()) {
		logger->log_info(name(), "RECIEVED SWITCH MESSAGE");
		if (bb_enable_switch_->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
			rv = false;
			bb_pose_->set_visibility_history(-1);
			bb_pose_->write();
		} else if (bb_enable_switch_->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
			rv = true;
		}

		bb_enable_switch_->msgq_pop();
	}
	if (rv != bb_enable_switch_->is_enabled()) {
		if (!cfg_debug_mode_) {
			logger->log_info(name(), "*** enabled: %s", rv ? "yes" : "no");
			bb_enable_switch_->set_enabled(rv);
		} else {
			logger->log_warn(name(),
			                 "*** enabled: %s, ignored because of DEBUG MODE, if will be ENABLED",
			                 rv ? "yes" : "no");
			bb_enable_switch_->set_enabled(true);
		}
		bb_enable_switch_->write();
	}

	// laser lines
	for (fawkes::LaserLineInterface *ll : laserlines_) {
		ll->read();
	}
}

void
ConveyorPlaneThread::pose_add_element(const Eigen::Vector4f &centroid,
                                      const Eigen::Vector3f &normal)
{
	// add element
	poses_.push_front(std::make_pair(centroid, normal));

	// if to full, remove oldest
	while (poses_.size() > cfg_pose_avg_hist_size_) {
		poses_.pop_back();
	}
}

bool
ConveyorPlaneThread::pose_get_avg(pose &out)
{
	// count invalid loops
	unsigned int                                           invalid = 0;
	std::list<std::pair<Eigen::Vector4f, Eigen::Vector3f>> valid_planes;
	for (const std::pair<Eigen::Vector4f, Eigen::Vector3f> &p : poses_) {
		//-- keep plane if valid
		if (!std::isnan(p.first(0))) {
			valid_planes.push_back(p);

		} else {
			++invalid;
		}
	}

	if (valid_planes.size() <= 0) {
		logger->log_warn(name(), "no valid planes to average");
		return false;

	} else if (invalid > cfg_allow_invalid_poses_) {
		logger->log_warn(name(), "view unstable, got %u invalid frames", invalid);
		return false;
	}

	//-- averaging planes
	Eigen::Vector4f median_centroid(0, 0, 0, 0);
	Eigen::Vector3f median_normal(0, 0, 0);
	float           invN = 1.f / static_cast<float>(valid_planes.size());
	for (const std::pair<Eigen::Vector4f, Eigen::Vector3f> &p : valid_planes) {
		median_centroid += p.first * invN;
		median_normal += p.second * invN;
	}

	// remove outliers
	std::list<std::pair<Eigen::Vector4f, Eigen::Vector3f>> planes_used;
	for (const std::pair<Eigen::Vector4f, Eigen::Vector3f> &p : valid_planes) {
		float dist = (p.first - median_centroid).norm();

		if (dist <= cfg_pose_diff_) {
			planes_used.push_back(p);
		}
	}

	if (planes_used.size() < cfg_pose_avg_min_) {
		logger->log_warn(name(), "not enough for average, got: %zu", planes_used.size());
	}

	//-- if there are any inliers kept, due to the outlier removal ...
	if (planes_used.size() > 0) {
		//-- ... recalculate average using valid inliers
		median_centroid = Eigen::Vector4f(0, 0, 0, 0);
		median_normal   = Eigen::Vector3f(0, 0, 0);
		invN            = 1.f / static_cast<float>(planes_used.size());

		for (const std::pair<Eigen::Vector4f, Eigen::Vector3f> &p : planes_used) {
			median_centroid += p.first * invN;
			median_normal += p.second * invN;
		}
	}

	out = calculate_pose(median_centroid, median_normal);

	return true;
}

bool
ConveyorPlaneThread::laserline_get_best_fit(fawkes::LaserLineInterface *&best_fit)
{
	best_fit = laserlines_.front();

	// get best line
	for (fawkes::LaserLineInterface *ll : laserlines_) {
		// just with writer
		if (!ll->has_writer()) {
			continue;
		}
		// just with history
		if (ll->visibility_history() <= 2) {
			continue;
		}
		// just if not too far away
		Eigen::Vector3f center = laserline_get_center_transformed(ll);

		if (std::sqrt(static_cast<float>(center(0) * center(0) + center(2) * center(2))) > 0.8) {
			continue;
		}

		// take with lowest angle
		if (fabs(best_fit->bearing()) > fabs(ll->bearing())) {
			best_fit = ll;
		}
	}

	if (!best_fit->has_writer()) {
		logger->log_info(name(), "no writer for laser lines");
		best_fit = NULL;
		return false;
	}
	if (best_fit->visibility_history() <= 2) {
		best_fit = NULL;
		return false;
	}
	if (fabs(best_fit->bearing()) > 0.35) {
		best_fit = NULL;
		return false;
	} // ~20 deg
	Eigen::Vector3f center = laserline_get_center_transformed(best_fit);

	if (std::sqrt(center(0) * center(0) + center(2) * center(2)) > 0.8) {
		best_fit = NULL;
		return false;
	}

	return true;
}

Eigen::Vector3f
ConveyorPlaneThread::laserline_get_center_transformed(fawkes::LaserLineInterface *ll)
{
	fawkes::tf::Stamped<fawkes::tf::Point> tf_in, tf_out;
	tf_in.stamp    = fawkes::Time(0.); // ll->timestamp();
	tf_in.frame_id = ll->frame_id();
	tf_in.setX(ll->end_point_2(0) + (ll->end_point_1(0) - ll->end_point_2(0)) / 2.);
	tf_in.setY(ll->end_point_2(1) + (ll->end_point_1(1) - ll->end_point_2(1)) / 2.);
	tf_in.setZ(ll->end_point_2(2) + (ll->end_point_1(2) - ll->end_point_2(2)) / 2.);

	try {
		tf_listener->transform_point(header_.frame_id, tf_in, tf_out);
	} catch (tf::ExtrapolationException &) {
		tf_in.stamp = Time(0, 0);
		tf_listener->transform_point(header_.frame_id, tf_in, tf_out);
	}

	Eigen::Vector3f out(tf_out.getX(), tf_out.getY(), tf_out.getZ());

	return out;
}

Eigen::Vector3f
ConveyorPlaneThread::get_conveyor_estimate(fawkes::LaserLineInterface *laser_line,
                                           const std::string &         target_frame,
                                           const fawkes::Time &        desired_stamp)
{
	btVector3 endpt_1(laser_line->end_point_1(0),
	                  laser_line->end_point_1(1),
	                  laser_line->end_point_1(2));
	btVector3 endpt_2(laser_line->end_point_2(0),
	                  laser_line->end_point_2(1),
	                  laser_line->end_point_2(2));
	btVector3 center = endpt_2 + (endpt_1 - endpt_2) * 0.5;

	fawkes::tf::Stamped<fawkes::tf::Point> estimate_source;
	estimate_source.stamp    = desired_stamp; // laser_line->timestamp();
	estimate_source.frame_id = std::string(laser_line->frame_id());
	estimate_source.set_data(center);
	estimate_source += {0, 0, double(cfg_mps_top_offset)};

	fawkes::tf::Stamped<fawkes::tf::Point> estimate_target;
	try {
		tf_listener->transform_point(target_frame, estimate_source, estimate_target);
	} catch (tf::ExtrapolationException &) {
		estimate_source.stamp = Time(0, 0);
		tf_listener->transform_point(target_frame, estimate_source, estimate_target);
	}

	return Eigen::Vector3f(estimate_target.getX(), estimate_target.getY(), estimate_target.getZ());
}

bool
ConveyorPlaneThread::is_inbetween(double a, double b, double val)
{
	double low = std::min(a, b);
	double up  = std::max(a, b);

	if (val >= low && val <= up) {
		return true;
	} else {
		return false;
	}
}

CloudPtr
ConveyorPlaneThread::crop_cloud(CloudPtr                    in_cloud,
                                fawkes::LaserLineInterface *laser_line,
                                bool                        use_laserline)
{
	CloudPtr cropped_cloud(new Cloud);

	Eigen::Vector3f min(1.f, 1.f, 1.f);
	Eigen::Vector3f max(1.f, 1.f, 1.f);

	if (use_laserline) {
		std::string target_frame = header_.frame_id;
		try {
			Eigen::Vector3f cropbox_reference =
			  get_conveyor_estimate(laser_line, target_frame, fawkes::Time(0, 0));
			logger->log_debug(name(),
			                  "ll cropbox_reference = (%f, %f, %f) in %s",
			                  cropbox_reference[0],
			                  cropbox_reference[1],
			                  cropbox_reference[2],
			                  target_frame.c_str());

			min = Eigen::Vector3f(cfg_crop_laserline_x_min,
			                      cfg_crop_laserline_y_min,
			                      cfg_crop_laserline_z_min)
			      + cropbox_reference;
			max = Eigen::Vector3f(cfg_crop_laserline_x_max,
			                      cfg_crop_laserline_y_max,
			                      cfg_crop_laserline_z_max)
			      + cropbox_reference;
		} catch (fawkes::Exception &) {
			use_laserline = false;
		}
	}
	if (!use_laserline) {
		min = Eigen::Vector3f(cfg_crop_cam_x_min, cfg_crop_cam_y_min, cfg_crop_cam_z_min);
		max = Eigen::Vector3f(cfg_crop_cam_x_max, cfg_crop_cam_y_max, cfg_crop_cam_z_max);
	}

	for (const Point &pt : *in_cloud) {
		if (pcl::isFinite<Point>(pt) && pt.x >= min[0] && pt.x <= max[0] && pt.y >= min[1]
		    && pt.y <= max[1] && pt.z >= min[2] && pt.z <= max[2])
			cropped_cloud->push_back(pt);
	}
	cropped_cloud->width    = static_cast<uint32_t>(cropped_cloud->size());
	cropped_cloud->height   = 1;
	cropped_cloud->is_dense = true;

	return cropped_cloud;
}

CloudPtr
ConveyorPlaneThread::cloud_get_plane(CloudPtr in, pcl::ModelCoefficients::Ptr coeff)
{
	CloudPtr out(new Cloud(*in));

	// get planes
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(cfg_plane_dist_threshold_);

	seg.setInputCloud(out);
	seg.segment(*inliers, *coeff);

	if (inliers->indices.size() == 0) {
		logger->log_debug(name(), "Could not estimate a planar model for the given dataset.");
		return CloudPtr();
	}

	// get inliers
	CloudPtr                   tmp(new Cloud);
	pcl::ExtractIndices<Point> extract;
	extract.setInputCloud(out);
	extract.setIndices(inliers);
	extract.setNegative(false);

	extract.filter(*tmp);
	*out = *tmp;

	// check if cloud normal is ok
	return out;
}

pcl::shared_ptr<std::vector<pcl::PointIndices>>
ConveyorPlaneThread::cloud_cluster(CloudPtr in)
{
	// in = cloud_voxel_grid(in);
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(in);

	pcl::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices(
	  new std::vector<pcl::PointIndices>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cfg_cluster_tolerance_);
	ec.setMinClusterSize(cfg_cluster_size_min_);
	ec.setMaxClusterSize(cfg_cluster_size_max_);
	ec.setSearchMethod(tree);
	ec.setInputCloud(in);
	ec.extract(*cluster_indices);

	return cluster_indices;
}

std::vector<CloudPtr>
ConveyorPlaneThread::cluster_split(CloudPtr                                        in,
                                   pcl::shared_ptr<std::vector<pcl::PointIndices>> cluster_indices)
{
	std::vector<CloudPtr> clouds_out;
	for (std::vector<pcl::PointIndices>::const_iterator c_it = cluster_indices->begin();
	     c_it != cluster_indices->end();
	     ++c_it) {
		CloudPtr cloud_cluster(new Cloud);
		for (std::vector<int>::const_iterator p_it = c_it->indices.begin(); p_it != c_it->indices.end();
		     ++p_it) {
			cloud_cluster->points.push_back(in->points[*p_it]);
		}
		cloud_cluster->width    = cloud_cluster->points.size();
		cloud_cluster->height   = 1;
		cloud_cluster->is_dense = true;

		clouds_out.push_back(cloud_cluster);
	}

	return clouds_out;
}

CloudPtr
ConveyorPlaneThread::cluster_find_biggest(std::vector<CloudPtr> clouds_in, size_t &id)
{
	CloudPtr biggest(new Cloud);
	size_t   i = 0;
	for (CloudPtr current : clouds_in) {
		if (biggest->size() < current->size()) {
			biggest = current;
			id      = i;
		}
		++i;
	}

	return biggest;
}

CloudPtr
ConveyorPlaneThread::cloud_voxel_grid(CloudPtr in)
{
	float                                    ls = cfg_voxel_grid_leave_size_;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
	CloudPtr                                 out(new Cloud);
	vg.setInputCloud(in);
	// logger->log_debug(name(), "voxel leaf size is %f", ls);
	vg.setLeafSize(ls, ls, ls);
	vg.filter(*out);
	return out;
}

void
ConveyorPlaneThread::cloud_publish(CloudPtr cloud_in, fawkes::RefPtr<Cloud> cloud_out)
{
	**cloud_out       = *cloud_in;
	cloud_out->header = header_;
}

ConveyorPlaneThread::pose
ConveyorPlaneThread::calculate_pose(Eigen::Vector4f centroid, Eigen::Vector3f normal)
{
	Eigen::Vector3f z = normal.normalized();
	Eigen::Vector3f x;
	x << normal(0), normal(2), normal(1);
	Eigen::Vector3f y = x.cross(z);

	Eigen::Matrix3f rotMatrix;
	rotMatrix << x, y, z;
	Eigen::Quaternion<float> q(rotMatrix);
	fawkes::tf::Vector3      origin(centroid(0), centroid(1), centroid(2));
	fawkes::tf::Quaternion   rot(q.x(), q.y(), q.z(), q.w());

	pose ret;
	ret.translation = origin;
	ret.rotation    = rot;

	return ret;
}

void
ConveyorPlaneThread::tf_send_from_pose_if(pose pose)
{
	fawkes::tf::StampedTransform transform;

	transform.frame_id       = header_.frame_id;
	transform.child_frame_id = conveyor_frame_id_;
	// TODO use time of header, just don't works with bagfiles
	fawkes::Time pt;
	pt.set_time((long)header_.stamp / 1000);
	transform.stamp = pt;

	transform.setOrigin(pose.translation);
	transform.setRotation(pose.rotation);

	tf_publisher->send_transform(transform);
}

void
ConveyorPlaneThread::pose_write(pose pose)
{
	double translation[3], rotation[4];
	translation[0] = pose.translation.x();
	translation[1] = pose.translation.y();
	translation[2] = pose.translation.z();
	rotation[0]    = pose.rotation.x();
	rotation[1]    = pose.rotation.y();
	rotation[2]    = pose.rotation.z();
	rotation[3]    = pose.rotation.w();
	bb_pose_->set_translation(translation);
	bb_pose_->set_rotation(rotation);

	bb_pose_->set_frame(header_.frame_id.c_str());
	fawkes::Time stamp((long)header_.stamp);
	bb_pose_->set_visibility_history(vis_hist_);
	bb_pose_->write();
}

void
ConveyorPlaneThread::pose_publish_tf(pose pose)
{
	// transform data into gripper frame (this is better for later use)
	tf::Stamped<tf::Pose> tf_pose_cam, tf_pose_gripper;
	tf_pose_cam.stamp    = fawkes::Time(0.); // fawkes::Time((long)header_.stamp / 1000);
	tf_pose_cam.frame_id = header_.frame_id;
	tf_pose_cam.setOrigin(
	  tf::Vector3(pose.translation.x(), pose.translation.y(), pose.translation.z()));
	tf_pose_cam.setRotation(
	  tf::Quaternion(pose.rotation.x(), pose.rotation.y(), pose.rotation.z(), pose.rotation.w()));
	tf_listener->transform_pose("gripper", tf_pose_cam, tf_pose_gripper);

	// publish the transform from the gripper to the conveyor
	tf::Transform        transform(tf::create_quaternion_from_yaw(M_PI), tf_pose_gripper.getOrigin());
	tf::StampedTransform stamped_transform(transform,
	                                       tf_pose_gripper.stamp,
	                                       tf_pose_gripper.frame_id,
	                                       conveyor_frame_id_);
	tf_publisher->send_transform(stamped_transform);
}
void
ConveyorPlaneThread::start_waiting()
{
	wait_start_ = Time();
}

bool
ConveyorPlaneThread::need_to_wait()
{
	return Time() < wait_start_ + wait_time_;
}
