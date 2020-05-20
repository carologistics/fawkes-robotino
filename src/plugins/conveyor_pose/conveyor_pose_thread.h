
/***************************************************************************
 *  conveyor_pose_thread.h - conveyor_pose
 *
 *  Created: Thr 12. April 16:28:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
 *             2018 Victor Mataré
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

#ifndef _CONVEYOR_POSE_THREAD_
#define _CONVEYOR_POSE_THREAD_

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
#include <interfaces/ConveyorPoseInterface.h>
#include <interfaces/Position3DInterface.h>
#include <pcl_utils/compatibility.h>
#include <plugins/ros/aspect/ros.h>

//#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>

#include <array>
#include <atomic>
#include <map>
#include <set>
#include <string>

#define CFG_PREFIX "/plugins/conveyor_pose"

#define X_DIR 0
#define Y_DIR 1
#define Z_DIR 2

class RecognitionThread;

namespace fawkes {
class ConveyorPoseInterface;
class SwitchInterface;
class LaserLineInterface;
} // namespace fawkes

class ConveyorPoseThread : public fawkes::Thread,
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

	ConveyorPoseThread();

	virtual void init() override;
	virtual void loop() override;
	virtual void finalize() override;

	/** Used by the plugin constructor to enable data exchange between the two
   * threads
   * @param recog_thread pointer to the thread that does the actual pose
   * recognition
   */
	void set_cg_thread(RecognitionThread *recog_thread);

private:
	friend RecognitionThread;

	typedef pcl::PointXYZ          Point;
	typedef pcl::PointCloud<Point> Cloud;
	typedef Cloud::Ptr             CloudPtr;
	typedef Cloud::ConstPtr        CloudConstPtr;

	std::atomic<double>               result_fitness_;
	const std::string                 syncpoint_ready_for_icp_name_;
	fawkes::RefPtr<fawkes::SyncPoint> syncpoint_ready_for_icp_;

	// cfg values
	std::string              cfg_if_prefix_;
	std::string              cloud_in_name_;
	const std::string        cloud_out_raw_name_;
	const std::string        cloud_out_trimmed_name_;
	std::string              cfg_bb_realsense_switch_name_;
	std::string              conveyor_frame_id_;
	std::vector<std::string> laserlines_names_;

	CloudPtr trimmed_scene_;

	fawkes::ConveyorPoseInterface::MPS_TYPE   current_mps_type_;
	fawkes::ConveyorPoseInterface::MPS_TARGET current_mps_target_;

	std::unique_ptr<fawkes::Time> initial_guess_deadline_;
	fawkes::Position3DInterface * bb_init_guess_pose_;

	int cfg_force_shelf_;

	void        update_station_information(fawkes::ConveyorPoseInterface::RunICPMessage &msg);
	std::string get_model_path(fawkes::ConveyorPoseInterface *iface,
	                           fawkes::ConveyorPoseInterface::MPS_TYPE,
	                           fawkes::ConveyorPoseInterface::MPS_TARGET);

	// Mapping from {Type,Target} to its corresponding model path
	std::map<
	  std::pair<fawkes::ConveyorPoseInterface::MPS_TYPE, fawkes::ConveyorPoseInterface::MPS_TARGET>,
	  std::string>
	  type_target_to_path_;

	// Mapping from station name to preprocessed pointcloud model
	std::map<
	  std::pair<fawkes::ConveyorPoseInterface::MPS_TYPE, fawkes::ConveyorPoseInterface::MPS_TARGET>,
	  CloudPtr>
	  type_target_to_model_;

	RecognitionThread *recognition_thread_;

	bool        cfg_record_model_;
	std::string cfg_model_origin_frame_;
	std::string cfg_record_path_;

	std::atomic<float> cfg_left_cut_;
	std::atomic<float> cfg_right_cut_;
	std::atomic<float> cfg_top_cut_;
	std::atomic<float> cfg_bottom_cut_;
	std::atomic<float> cfg_front_cut_;
	std::atomic<float> cfg_back_cut_;

	std::atomic<float> cfg_left_cut_no_ll_;
	std::atomic<float> cfg_right_cut_no_ll_;
	std::atomic<float> cfg_top_cut_no_ll_;
	std::atomic<float> cfg_bottom_cut_no_ll_;
	std::atomic<float> cfg_front_cut_no_ll_;
	std::atomic<float> cfg_back_cut_no_ll_;

	std::atomic<float> cfg_shelf_left_cut_;
	std::atomic<float> cfg_shelf_right_cut_;
	std::atomic<float> cfg_shelf_top_cut_;
	std::atomic<float> cfg_shelf_bottom_cut_;
	std::atomic<float> cfg_shelf_front_cut_;
	std::atomic<float> cfg_shelf_back_cut_;

	std::atomic<float> cfg_shelf_left_cut_no_ll_;
	std::atomic<float> cfg_shelf_right_cut_no_ll_;
	std::atomic<float> cfg_shelf_top_cut_no_ll_;
	std::atomic<float> cfg_shelf_bottom_cut_no_ll_;
	std::atomic<float> cfg_shelf_front_cut_no_ll_;
	std::atomic<float> cfg_shelf_back_cut_no_ll_;

	std::atomic<float> cfg_shelf_left_off_;
	std::atomic<float> cfg_shelf_middle_off_;
	std::atomic<float> cfg_shelf_right_off_;

	std::atomic<float> cfg_voxel_grid_leaf_size_;

	std::atomic<double> cfg_max_timediff_external_pc_;
	std::atomic<double> cfg_external_timeout_;

	std::map<fawkes::ConveyorPoseInterface::MPS_TARGET, std::array<std::atomic<float>, 3>>
	  cfg_target_hint_;
	std::map<fawkes::ConveyorPoseInterface::MPS_TYPE, std::array<std::atomic<float>, 3>>
	  cfg_type_offset_;

	std::atomic<float> cfg_ll_bearing_thresh_;

	// state vars
	bool           cfg_enable_switch_;
	bool           cloud_in_registered_;
	bool           have_initial_guess_;
	pcl::PCLHeader input_pc_header_;

	// point clouds from pcl_manager
	fawkes::RefPtr<const Cloud> cloud_in_;

	// interfaces write
	fawkes::ConveyorPoseInterface *bb_pose_;

	// interfaces read
	std::vector<fawkes::LaserLineInterface *> laserlines_;
	fawkes::SwitchInterface *                 realsense_switch_;
	fawkes::Time                              wait_start_;
	fawkes::Time                              wait_time_;

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

	fawkes::tf::Stamped<fawkes::tf::Pose> initial_guess_odom_;
	fawkes::LaserLineInterface *          best_laser_line_;

	std::atomic<bool> cfg_debug_mode_;

	bool update_input_cloud();

	fawkes::LaserLineInterface *laserline_get_best_fit();

	bool set_laserline_initial_tf(fawkes::tf::Stamped<fawkes::tf::Pose> &);
	bool set_external_initial_tf(fawkes::tf::Stamped<fawkes::tf::Pose> &);

	CloudPtr cloud_trim(CloudPtr in);

	pcl::shared_ptr<std::vector<pcl::PointIndices>> cloud_cluster(CloudPtr in);
	CloudPtr                                        cloud_voxel_grid(CloudPtr in);

	bool is_target_shelf();

	void pose_write();
	void record_model();

	void pose_publish_tf(const fawkes::tf::Stamped<fawkes::tf::Pose> &pose);
	void start_waiting();
	bool need_to_wait();

	bool initial_guess_deadline_reached();
	bool get_initial_guess();

	virtual void config_value_erased(const char *path) override;
	virtual void config_tag_changed(const char *new_tag) override;
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v) override;
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v) override;

	template <typename T>
	inline void
	change_val(const std::string &setting, std::atomic<T> &var, const T &val)
	{
		if (var != val) {
			logger->log_info(name(),
			                 "Changing %s from %s to %s",
			                 setting.c_str(),
			                 std::to_string(var).c_str(),
			                 std::to_string(val).c_str());
			var = val;
		}
	}

protected:
	virtual void
	run() override
	{
		Thread::run();
	}
};

Eigen::Matrix4f  pose_to_eigen(const fawkes::tf::Pose &pose);
fawkes::tf::Pose eigen_to_pose(const Eigen::Matrix4f &m);

#endif
