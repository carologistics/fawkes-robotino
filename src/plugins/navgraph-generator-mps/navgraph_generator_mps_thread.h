/***************************************************************************
 *  navgraph_generator_mps_thread.h - generate navgraph for MPS game
 *
 *  Created: Sun Jul 13 15:30:03 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_GENERATOR_MPS_NAVGRAPH_GENERATOR_MPS_H_
#define __PLUGINS_NAVGRAPH_GENERATOR_MPS_NAVGRAPH_GENERATOR_MPS_H_

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <interfaces/NavGraphGeneratorInterface.h>

#include <Eigen/Geometry>
#include <array>
#include <map>
#include <string>

namespace fawkes {
class NavGraphWithMPSGeneratorInterface;
class BlackBoardOnMessageWaker;
class NavGraph;
} // namespace fawkes

class NavGraphGeneratorMPSThread : public fawkes::Thread,
                                   public fawkes::ClockAspect,
                                   public fawkes::LoggingAspect,
                                   public fawkes::ConfigurableAspect,
                                   public fawkes::BlackBoardAspect,
                                   public fawkes::TransformAspect,
                                   public fawkes::BlackBoardInterfaceListener
{
public:
	NavGraphGeneratorMPSThread();
	virtual ~NavGraphGeneratorMPSThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void                                               generate_navgraph();
	fawkes::NavGraphGeneratorInterface::ConnectionMode mps_node_insmode(std::string name);

	virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

private:
	std::string                                   cfg_global_frame_;
	float                                         cfg_mps_width_;
	float                                         cfg_mps_length_;
	float                                         cfg_mps_approach_dist_;
	bool                                          cfg_mps_corner_obst_;
	float                                         cfg_map_min_dist_;
	float                                         cfg_map_point_max_dist_;
	fawkes::NavGraphGeneratorInterface::Algorithm cfg_algorithm_;
	std::map<std::string, std::string>            cfg_algo_params_;
	Eigen::Vector2f                               cfg_bounding_box_p1_;
	Eigen::Vector2f                               cfg_bounding_box_p2_;
	size_t                                        cfg_num_wait_zones_;

	unsigned int                               last_id_;
	fawkes::NavGraphGeneratorInterface *       navgen_if_;
	fawkes::NavGraphWithMPSGeneratorInterface *navgen_mps_if_;

	fawkes::BlackBoardOnMessageWaker *msg_waker_;

	fawkes::NavGraph *base_graph_;

	std::vector<unsigned int> exp_zones_;

	unsigned int                                             compute_msgid_;
	std::vector<Eigen::Vector2i>                             wait_zones_;
	std::map<std::string, std::pair<Eigen::Vector2i, float>> mps_wait_zones_;

	typedef struct
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		std::string        tag_frame;
		Eigen::Vector3f    tag_pose_pos;
		Eigen::Quaternionf tag_pose_ori;
		bool               tag_is_input;

		Eigen::Vector3f    pose_pos;
		Eigen::Quaternionf pose_ori;
		float              pose_yaw;

		bool transformed;

		Eigen::Vector3f    input_pos;
		Eigen::Quaternionf input_ori;
		float              input_yaw;

		Eigen::Vector3f              output_pos;
		Eigen::Quaternionf           output_ori;
		float                        output_yaw;
		Eigen::Vector2i              zone;
		std::vector<Eigen::Vector2i> blocked_zones;

		std::vector<Eigen::Vector2f> corners;

	} MPSStation;
	std::map<std::string, MPSStation> stations_;

	static std::map<uint16_t, std::vector<Eigen::Vector2i>> zone_blocking_;
	static std::vector<Eigen::Vector2i>                     reserved_zones_;

	void                                       update_station(std::string     id,
	                                                          bool            input,
	                                                          std::string     frame,
	                                                          double          tag_pos[3],
	                                                          double          tag_ori[4],
	                                                          Eigen::Vector2i zone);
	static inline std::vector<Eigen::Vector2i> blocked_zones(Eigen::Vector2i zone,
	                                                         uint16_t        discrete_ori);
	void generate_wait_zones(size_t count, std::vector<Eigen::Vector2i> &free_zones);
	void generate_mps_wait_zones(std::vector<Eigen::Vector2i> &free_zones);
};

#endif
