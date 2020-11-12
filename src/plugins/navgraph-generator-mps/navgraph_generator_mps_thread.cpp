
/***************************************************************************
 *  navgraph_generator_mps_thread.cpp - Generate navgraph for MPS
 *
 *  Created: Tue Jan 13 15:33:37 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_generator_mps_thread.h"

#include <blackboard/utils/on_message_waker.h>
#include <core/threading/mutex_locker.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
#include <navgraph/navgraph.h>
#include <navgraph/yaml_navgraph.h>
#include <utils/math/angle.h>
#include <utils/math/eigen.h>

#include <limits>
#include <memory>

using namespace fawkes;

/** @class NavGraphGeneratorMPSThread "navgraph_generator_mps_thread.h"
 * Block navgraph paths based on laser clusters.
 * @author Tim Niemueller
 */

namespace fawkes {
namespace workaround {
static inline Eigen::Vector3f
pointAt(const Eigen::ParametrizedLine<float, 3> &l, const float k)
{
#if EIGEN_VERSION_AT_LEAST(3, 2, 0)
	return l.pointAt(k);
#else
	return l.origin() + (l.direction() * k);
#endif
}
} // namespace workaround
} // namespace fawkes

std::map<uint16_t, std::vector<Eigen::Vector2i>> NavGraphGeneratorMPSThread::zone_blocking_ = {
  {0,
   {
     // 0°
     {-1, 0}, //
     {1, 0}   // x|x
   }},
  {45,
   {         // 45°
    {1, 0},  //
    {1, 1},  //  xx
    {0, 1},  // x\x
    {0, -1}, // xx
    {-1, -1},
    {-1, 0}}},
  {90,
   {
     // 90°
     {0, 1}, //  x
     {0, -1} //  -
   }},       //  x
  {135,
   {{0, 1},     // 135°
    {-1, 1},    //
    {-1, 0},    // xx
    {1, 0},     // x/x
    {1, -1},    //  xx
    {0, -1}}}}; // 180°-315° is the same, see constructor.

std::vector<Eigen::Vector2i> NavGraphGeneratorMPSThread::reserved_zones_ = {
  {-5, 2},
  {5, 2},
  {-7, 1},
  {-6, 1},
  {-5, 1},
  {5, 1},
  {6, 1},
  {7, 1},
};

/** Constructor. */
NavGraphGeneratorMPSThread::NavGraphGeneratorMPSThread()
: Thread("NavGraphGeneratorMPSThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("NavGraphGeneratorMPSThread")
{
	zone_blocking_[180] = zone_blocking_[0];
	zone_blocking_[225] = zone_blocking_[45];
	zone_blocking_[270] = zone_blocking_[90];
	zone_blocking_[315] = zone_blocking_[135];
}

/** Destructor. */
NavGraphGeneratorMPSThread::~NavGraphGeneratorMPSThread()
{
}

void
NavGraphGeneratorMPSThread::init()
{
	cfg_global_frame_      = config->get_string("/frames/fixed");
	cfg_mps_width_         = config->get_float("/navgraph-generator-mps/mps-width");
	cfg_mps_length_        = config->get_float("/navgraph-generator-mps/mps-length");
	cfg_mps_approach_dist_ = config->get_float("/navgraph-generator-mps/approach-distance");
	cfg_mps_corner_obst_   = config->get_bool("/navgraph-generator-mps/add-corners-as-obstacles");

	cfg_map_min_dist_       = config->get_float("/navgraph-generator-mps/map-cell-min-dist");
	cfg_map_point_max_dist_ = config->get_float("/navgraph-generator-mps/map-point-max-dist");
	cfg_num_wait_zones_     = size_t(config->get_uint("/navgraph-generator-mps/num-wait-zones"));

	std::string algorithm = config->get_string("/navgraph-generator-mps/algorithm");
	if (algorithm == "voronoi") {
		cfg_algorithm_ = NavGraphGeneratorInterface::ALGORITHM_VORONOI;
	} else if (algorithm == "grid") {
		cfg_algorithm_ = NavGraphGeneratorInterface::ALGORITHM_GRID;
		const std::string                             prefix("/navgraph-generator-mps/grid/");
		std::unique_ptr<Configuration::ValueIterator> i(config->search(prefix));
		while (i->next()) {
			const std::string path(i->path());
			cfg_algo_params_[path.substr(prefix.length())] = i->get_as_string();
		}
	}

	std::string base_graph_file;
	try {
		base_graph_file = config->get_string("/navgraph-generator-mps/base-graph");
	} catch (Exception &e) {
	} // ignored, no base graph

	base_graph_ = NULL;
	if (!base_graph_file.empty()) {
		if (base_graph_file[0] != '/') {
			base_graph_file = std::string(CONFDIR) + "/" + base_graph_file;
		}
		base_graph_ = load_yaml_navgraph(base_graph_file);
	} else {
		logger->log_warn(name(), "No base graph configured");
	}

	std::vector<float> p1   = config->get_floats("/navgraph-generator-mps/bounding-box/p1");
	std::vector<float> p2   = config->get_floats("/navgraph-generator-mps/bounding-box/p2");
	cfg_bounding_box_p1_[0] = p1[0];
	cfg_bounding_box_p1_[1] = p1[1];
	cfg_bounding_box_p2_[0] = p2[0];
	cfg_bounding_box_p2_[1] = p2[1];

	exp_zones_.resize(24, true);

	navgen_if_ = blackboard->open_for_reading<NavGraphGeneratorInterface>("/navgraph-generator");

	navgen_mps_if_ =
	  blackboard->open_for_writing<NavGraphWithMPSGeneratorInterface>("/navgraph-generator-mps");

	compute_msgid_ = 0;

	bbil_add_data_interface(navgen_if_);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);

	/* For testing
  tf::Quaternion q = tf::create_quaternion_from_yaw(deg2rad(45));
  double tag_pos[3] = {1.0,0.,0.};
  double tag_ori[4] = {q.x(), q.y(), q.z(), q.w()};
  update_station("Test", false, "/base_laser", tag_pos, tag_ori);
  generate_navgraph();
  */

	msg_waker_ = new BlackBoardOnMessageWaker(blackboard, navgen_mps_if_, this);
}

void
NavGraphGeneratorMPSThread::finalize()
{
	delete msg_waker_;

	blackboard->unregister_listener(this);
	bbil_remove_data_interface(navgen_if_);

	blackboard->close(navgen_mps_if_);
	blackboard->close(navgen_if_);

	delete base_graph_;
}

void
NavGraphGeneratorMPSThread::loop()
{
	while (!navgen_mps_if_->msgq_empty()) {
		if (navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::ClearMessage>()) {
			stations_.clear();

		} else if (navgen_mps_if_
		             ->msgq_first_is<NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage>()) {
			NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage *m =
			  navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage>();
			logger->log_warn(name(), "Updating station %s from tag", m->name());

			update_station(m->name(),
			               (m->side() == NavGraphWithMPSGeneratorInterface::INPUT),
			               m->frame(),
			               m->tag_translation(),
			               m->tag_rotation(),
			               Eigen::Vector2i{m->zone_coords(0), m->zone_coords(1)});

		} else if (navgen_mps_if_
		             ->msgq_first_is<NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage>()) {
			NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage *m =
			  navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage>();
			for (size_t i = 0; i < std::min(m->maxlenof_zones(), exp_zones_.size()); ++i) {
				exp_zones_[i] = m->is_zones(i);
			}

		} else if (navgen_mps_if_
		             ->msgq_first_is<NavGraphWithMPSGeneratorInterface::GenerateWaitZonesMessage>()) {
			wait_zones_.clear();
			std::vector<Eigen::Vector2i> blocked_zones;
			for (auto &entry : stations_) {
				blocked_zones.push_back(entry.second.zone);
				blocked_zones.insert(blocked_zones.end(),
				                     entry.second.blocked_zones.begin(),
				                     entry.second.blocked_zones.end());
			}

			std::vector<Eigen::Vector2i> free_zones, free_zones_left, free_zones_right;

			int x_min, x_max, y_min, y_max;
			if (cfg_bounding_box_p1_.x() < cfg_bounding_box_p2_.x()) {
				x_min = int(cfg_bounding_box_p1_.x());
				x_max = int(cfg_bounding_box_p2_.x());
			} else {
				x_min = int(cfg_bounding_box_p2_.x());
				x_max = int(cfg_bounding_box_p1_.x());
			}
			if (cfg_bounding_box_p1_.y() < cfg_bounding_box_p2_.y()) {
				y_min = int(cfg_bounding_box_p1_.y());
				y_max = int(cfg_bounding_box_p2_.y());
			} else {
				y_min = int(cfg_bounding_box_p2_.y());
				y_max = int(cfg_bounding_box_p1_.y());
			}

			for (int x = x_min; x <= x_max; ++x) {
				for (int y = y_min; y <= y_max; ++y) {
					if (x != 0 && y != 0) {
						Eigen::Vector2i zn = {x, y};
						if (std::find(reserved_zones_.begin(), reserved_zones_.end(), zn)
						      == reserved_zones_.end()
						    && std::find(blocked_zones.begin(), blocked_zones.end(), zn)
						         == blocked_zones.end()) {
							// Zone is not reserved or blocked
							free_zones.push_back(zn);
							if (x < 0)
								free_zones_left.push_back(zn);
							else
								free_zones_right.push_back(zn);
						}
					}
				}
			}

			generate_wait_zones(cfg_num_wait_zones_, free_zones_left);
			generate_wait_zones(cfg_num_wait_zones_, free_zones_right);
			generate_mps_wait_zones(free_zones);
		} else if (navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::ComputeMessage>()) {
			NavGraphWithMPSGeneratorInterface::ComputeMessage *m =
			  navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::ComputeMessage>();
			logger->log_info(name(), "Computation started");
			navgen_mps_if_->set_final(false);
			navgen_mps_if_->set_msgid(m->id());
			navgen_mps_if_->write();
			generate_navgraph();
		} else {
			logger->log_warn(name(), "Unknown message received");
		}

		navgen_mps_if_->msgq_pop();
	}
}

void
NavGraphGeneratorMPSThread::generate_mps_wait_zones(std::vector<Eigen::Vector2i> &free_zones)
{
	for (auto &entry : this->stations_) {
		std::sort(free_zones.begin(),
		          free_zones.end(),
		          [&entry](const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
			          Eigen::Vector2f station_input = entry.second.input_pos.head(2);
			          double          d_lhs         = (lhs.cast<float>() - station_input).norm();
			          double          d_rhs         = (rhs.cast<float>() - station_input).norm();
			          return d_lhs < d_rhs;
		          });
		Eigen::Vector2f ori_vec =
		  (*(free_zones.begin())).cast<float>() - entry.second.input_pos.head(2);
		double wait_ori = std::atan2(ori_vec.x(), ori_vec.y());
		mps_wait_zones_.emplace(entry.first + "-I", std::make_pair(*(free_zones.begin()), wait_ori));
		std::sort(free_zones.begin(),
		          free_zones.end(),
		          [&entry](const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
			          Eigen::Vector2f station_output = entry.second.output_pos.head(2);
			          double          d_lhs          = (lhs.cast<float>() - station_output).norm();
			          double          d_rhs          = (rhs.cast<float>() - station_output).norm();
			          return d_lhs < d_rhs;
		          });
		ori_vec  = (*(free_zones.begin())).cast<float>() - entry.second.output_pos.head(2);
		wait_ori = std::atan2(ori_vec.x(), ori_vec.y());
		mps_wait_zones_.emplace(entry.first + "-O", std::make_pair(*(free_zones.begin()), wait_ori));
	}
}

void
NavGraphGeneratorMPSThread::generate_wait_zones(size_t                        count,
                                                std::vector<Eigen::Vector2i> &free_zones)
{
	std::sort(free_zones.begin(),
	          free_zones.end(),
	          [this](const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
		          double d_lhs_min = std::numeric_limits<double>::max();
		          double d_rhs_min = std::numeric_limits<double>::max();
		          for (auto &entry : this->stations_) {
			          Eigen::Vector2i &station_zn = entry.second.zone;

			          double d_lhs = (lhs - station_zn).norm();
			          if (d_lhs < d_lhs_min)
				          d_lhs_min = d_lhs;

			          double d_rhs = (rhs - station_zn).norm();
			          if (d_rhs < d_rhs_min)
				          d_rhs_min = d_rhs;
		          }
		          return d_lhs_min > d_rhs_min;
	          });

	for (auto it = free_zones.begin(); it < free_zones.begin() + count && it < free_zones.end();
	     ++it) {
		wait_zones_.push_back(*it);
	}
}

/*
 * 2017 rules: Machine rotation is always a multiple of 45°
 */
static inline uint16_t
quantize_yaw(float yaw)
{
	if (yaw < 0)
		yaw = 2.0f * float(M_PI) + yaw;
	uint16_t yaw_discrete = uint16_t(std::round(yaw / float(M_PI) * 4)) * 45;
	return yaw_discrete == 360 ? 0 : yaw_discrete;
}

std::vector<Eigen::Vector2i>
NavGraphGeneratorMPSThread::blocked_zones(Eigen::Vector2i zone, uint16_t discrete_ori)
{
	std::vector<Eigen::Vector2i> rv;
	for (const Eigen::Vector2i &offset : zone_blocking_[discrete_ori % 180]) {
		rv.push_back(zone + offset);
	}
	return rv;
}

void
NavGraphGeneratorMPSThread::update_station(std::string     id,
                                           bool            input,
                                           std::string     frame,
                                           double          tag_pos[3],
                                           double          tag_ori[4],
                                           Eigen::Vector2i zone)
{
	// **** 1. convert to map frame
	tf::Stamped<tf::Pose> pose;

	try {
		// Note that we add a correction offset to the centroid X value.
		// This offset is determined empirically and just turns out to be helpful
		// in certain situations.

		tf::Stamped<tf::Pose> spose(
		  tf::Pose(tf::Quaternion(tag_ori[0], tag_ori[1], tag_ori[2], tag_ori[3]),
		           tf::Vector3(tag_pos[0], tag_pos[1], tag_pos[2])),
		  fawkes::Time(0, 0),
		  frame);
		if (cfg_global_frame_ == frame) {
			pose = spose;
		} else {
			tf_listener->transform_pose(cfg_global_frame_, spose, pose);
		}
	} catch (tf::TransformException &e) {
		logger->log_warn(name(), "Transform exception:");
		logger->log_warn(name(), e);
		logger->log_error(name(), "Failed to add station %s: transform failed", id.c_str());
		return;
	}
	tf::Vector3     tf_pose_pos(pose.getOrigin());
	tf::Quaternion  tf_pose_ori(pose.getRotation());
	Eigen::Vector3f tag_pose_pos(tf_pose_pos.x(), tf_pose_pos.y(), tf_pose_pos.z());

	// **** 2. project to ground plane
	Eigen::Vector3f proj_pos;

	{
		Eigen::Vector3f plane_normal = Eigen::Vector3f::UnitZ();
		Eigen::Vector3f plane_origin(0, 0, 0);
		Eigen::Vector3f po     = tag_pose_pos - plane_origin;
		float           lambda = plane_normal.dot(po);
		proj_pos               = tag_pose_pos - (lambda * plane_normal);
	}

	Eigen::Quaternionf proj_ori(
	  Eigen::AngleAxisf(tf::get_yaw(tf_pose_ori), Eigen::Vector3f::UnitZ()));

	// **** 3. calculate approach points
	Eigen::Vector3f                   dir_vec = proj_ori * Eigen::Vector3f::UnitX();
	Eigen::ParametrizedLine<float, 3> mline(proj_pos, dir_vec);

	Eigen::Vector3f    input_pos, output_pos, pose_pos;
	Eigen::Quaternionf input_ori, output_ori, pose_ori;
	if (input) {
		input_pos  = workaround::pointAt(mline, cfg_mps_approach_dist_);
		input_ori  = proj_ori * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
		output_pos = workaround::pointAt(mline, -(cfg_mps_approach_dist_ + cfg_mps_width_));
		output_ori = proj_ori;
		pose_pos   = workaround::pointAt(mline, -(.5 * cfg_mps_width_));
		pose_ori   = output_ori;
	} else {
		output_pos = workaround::pointAt(mline, cfg_mps_approach_dist_);
		output_ori = proj_ori * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
		input_pos  = workaround::pointAt(mline, -(cfg_mps_approach_dist_ + cfg_mps_width_));
		input_ori  = proj_ori;
		pose_pos   = workaround::pointAt(mline, -(.5 * cfg_mps_width_));
		pose_ori   = input_ori;
	}

	// Problem: range of first value is only [0..pi], which might result
	// in "funny" and unexpected angles. If we want only yaw, we can work
	// around this using rpy order... Better use utils.
	// Eigen::Vector3f input_euler  = input_ori.toRotationMatrix().eulerAngles(0,
	// 1, 2);

	/*
  logger->log_info(name(), "P(%f,%f,%f) Q(%f,%f,%f,%f)  D(%f,%f,%f)",
                   proj_pos[0], proj_pos[1], proj_pos[2],
                   proj_ori.x(), proj_ori.y(), proj_ori.z(), proj_ori.w(),
                   dir_vec[0], dir_vec[1], dir_vec[2]);
  logger->log_info(name(), "I(%f,%f,%f)  O(%f,%f,%f)  DI %f  DO %f",
                   input_pos[0], input_pos[1], input_pos[2],
                   output_pos[0], output_pos[1], output_pos[2],
                   (input_pos - proj_pos).norm(), (output_pos -
  proj_pos).norm());
  */

	// **** 4. (optional) calculate corner points
	std::vector<Eigen::Vector2f> corners;
	if (cfg_mps_corner_obst_) {
		corners.resize(4);

		const float mps_width_2  = cfg_mps_width_ / 2.;
		const float mps_length_2 = cfg_mps_length_ / 2.;

		Eigen::Rotation2Df rot(quat_yaw(pose_ori));
		Eigen::Vector2f    center;
		center[0]  = pose_pos[0];
		center[1]  = pose_pos[1];
		corners[0] = (rot * Eigen::Vector2f(mps_width_2, -mps_length_2)) + center;
		corners[1] = (rot * Eigen::Vector2f(-mps_width_2, -mps_length_2)) + center;
		corners[2] = (rot * Eigen::Vector2f(-mps_width_2, mps_length_2)) + center;
		corners[3] = (rot * Eigen::Vector2f(mps_width_2, mps_length_2)) + center;
	}

	// **** 5. add to stations
	MPSStation s;
	s.tag_frame     = frame;
	s.tag_pose_pos  = pose_pos;
	s.tag_pose_ori  = proj_ori;
	s.pose_pos      = pose_pos;
	s.pose_ori      = pose_ori;
	s.pose_yaw      = quat_yaw(pose_ori);
	s.input_pos     = input_pos;
	s.input_ori     = input_ori;
	s.input_yaw     = quat_yaw(input_ori);
	s.output_pos    = output_pos;
	s.output_ori    = output_ori;
	s.output_yaw    = quat_yaw(output_ori);
	s.corners       = corners;
	s.zone          = zone;
	s.blocked_zones = blocked_zones(zone, quantize_yaw(quat_yaw(pose_ori)));

	stations_[id] = s;
}

fawkes::NavGraphGeneratorInterface::ConnectionMode
NavGraphGeneratorMPSThread::mps_node_insmode(std::string name)
{
	if (base_graph_ && base_graph_->node_exists(name)) {
		NavGraphNode n(base_graph_->node(name));
		if (n.has_property("generator-insert-mode")) {
			std::string prop = n.property("generator-insert-mode");
			if (prop == "closest-node") {
				return NavGraphGeneratorInterface::CLOSEST_NODE;
			} else if (prop == "closest-edge") {
				return NavGraphGeneratorInterface::CLOSEST_EDGE;
			} else if (prop == "closest-edge-or-node") {
				return NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE;
			}
		}
	}

	return NavGraphGeneratorInterface::CLOSEST_EDGE;
}

void
NavGraphGeneratorMPSThread::generate_navgraph()
{
	navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::ClearMessage());

	navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::SetAlgorithmMessage(cfg_algorithm_));
	for (const auto &p : cfg_algo_params_) {
		navgen_if_->msgq_enqueue(
		  new NavGraphGeneratorInterface::SetAlgorithmParameterMessage(p.first.c_str(),
		                                                               p.second.c_str()));
	}

	navgen_if_->msgq_enqueue(
	  new NavGraphGeneratorInterface::SetBoundingBoxMessage(cfg_bounding_box_p1_[0],
	                                                        cfg_bounding_box_p1_[1],
	                                                        cfg_bounding_box_p2_[0],
	                                                        cfg_bounding_box_p2_[1]));

	navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::SetFilterMessage(
	  NavGraphGeneratorInterface::FILTER_EDGES_BY_MAP, true));

	navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::SetFilterMessage(
	  NavGraphGeneratorInterface::FILTER_ORPHAN_NODES, true));

	navgen_if_->msgq_enqueue(
	  new NavGraphGeneratorInterface::SetFilterMessage(NavGraphGeneratorInterface::FILTER_MULTI_GRAPH,
	                                                   true));

	navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::SetFilterParamFloatMessage(
	  NavGraphGeneratorInterface::FILTER_EDGES_BY_MAP, "distance", cfg_map_min_dist_));

	navgen_if_->msgq_enqueue(
	  new NavGraphGeneratorInterface::AddMapObstaclesMessage(cfg_map_point_max_dist_));

	navgen_if_->msgq_enqueue(
	  new NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage(true));

	for (const auto &s : stations_) {
		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage(
		  (s.first + "-I").c_str(),
		  s.second.input_pos[0],
		  s.second.input_pos[1],
		  s.second.input_yaw,
		  mps_node_insmode(s.first + "-I")));

		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage(
		  (s.first + "-O").c_str(),
		  s.second.output_pos[0],
		  s.second.output_pos[1],
		  s.second.output_yaw,
		  mps_node_insmode(s.first + "-O")));

		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage(
		  s.first.c_str(),
		  s.second.pose_pos[0],
		  s.second.pose_pos[1],
		  s.second.output_yaw,
		  NavGraphGeneratorInterface::UNCONNECTED));
		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage(
		  s.first.c_str(), "mps", "true"));

		if (cfg_mps_corner_obst_) {
			for (size_t i = 0; i < s.second.corners.size(); ++i) {
				const Eigen::Vector2f &c = s.second.corners[i];
				navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddObstacleMessage(
				  (s.first + "-C" + std::to_string(i)).c_str(), c[0], c[1]));
			}
		}
		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddObstacleMessage(
		  (s.first + "-C").c_str(), s.second.pose_pos[0], s.second.pose_pos[1]));
	}

	for (const Eigen::Vector2i &zn : wait_zones_) {
		std::string z_name = "WAIT-";

		float x = float(zn.x());
		if (x < 0) {
			x += 0.5f;
			z_name += "M-Z";
		} else {
			x -= 0.5f;
			z_name += "C-Z";
		}

		float y = float(zn.y());
		if (y < 0)
			y += 0.5f;
		else
			y -= 0.5f;

		z_name += std::to_string(std::abs(zn.x())) + std::to_string(zn.y());

		logger->log_info(name(), "Adding wait zone %s at %f %f.", z_name.c_str(), x, y);

		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddPointOfInterestMessage(
		  z_name.c_str(), x, y, NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE));
	}

	for (const auto &zn : mps_wait_zones_) {
		std::string z_name = "WAIT-";
		z_name += zn.first;

		float x = float(zn.second.first.x());
		if (x < 0) {
			x += 0.5f;
		} else {
			x -= 0.5f;
		}

		float y = float(zn.second.first.y());
		if (y < 0)
			y += 0.5f;
		else
			y -= 0.5f;
		logger->log_info(name(), "Adding wait zone %s at %f %f.", z_name.c_str(), x, y);
		std::string wait_ori = "orientation";
		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddPointOfInterestMessage(
		  z_name.c_str(), x, y, NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE));
		navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage(
		  z_name.c_str(), wait_ori.c_str(), std::to_string(zn.second.second).c_str()));
	}

	if (base_graph_) {
		// add base graph nodes and edges
		std::list<std::string>           cn; // copied nodes
		const std::vector<NavGraphNode> &nodes = base_graph_->nodes();
		const std::vector<NavGraphEdge> &edges = base_graph_->edges();
		for (const NavGraphNode &n : nodes) {
			bool do_copy = n.has_property("always-copy") && n.property_as_bool("always-copy");
			if (!do_copy) {
				// check for other reasons
				for (size_t i = 0; i < exp_zones_.size(); ++i) {
					if (exp_zones_[i]) {
						std::string prop_name = "orientation_Z" + std::to_string(i + 1);
						if (n.has_property(prop_name)) {
							do_copy = true;
							break;
						}
					}
				}
			}
			if (do_copy) {
				cn.push_back(n.name());

				NavGraphGeneratorInterface::ConnectionMode conn_mode =
				  NavGraphGeneratorInterface::NOT_CONNECTED;

				if (n.unconnected()) {
					conn_mode = NavGraphGeneratorInterface::UNCONNECTED;
				}

				// insert-mode may still override unconnected status, which it
				// may only have to satisfy initial loading criteria
				if (n.has_property("insert-mode")) {
					std::string ins_mode = n.property("insert-mode");
					if (ins_mode == "closest-node" || ins_mode == "CLOSEST_NODE") {
						conn_mode = NavGraphGeneratorInterface::CLOSEST_NODE;
					} else if (ins_mode == "closest-edge" || ins_mode == "CLOSEST_EDGE") {
						conn_mode = NavGraphGeneratorInterface::CLOSEST_EDGE;
					} else if (ins_mode == "closest-edge-or-node" || ins_mode == "CLOSEST_EDGE_OR_NODE") {
						conn_mode = NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE;
					} else if (ins_mode == "unconnected" || ins_mode == "UNCONNECTED") {
						conn_mode = NavGraphGeneratorInterface::UNCONNECTED;
					} else if (ins_mode == "not-connected" || ins_mode == "NOT_CONNECTED") {
						conn_mode = NavGraphGeneratorInterface::NOT_CONNECTED;
					} else {
						ins_mode = "NOT_CONNECTED";
						logger->log_warn(name(),
						                 "Unknown insertion mode '%s' for node '%s', "
						                 "defaulting to NOT_CONNECTED",
						                 ins_mode.c_str(),
						                 n.name().c_str());
					}
				}

				logger->log_debug(name(),
				                  "Copying node %s (insertion mode %s)",
				                  n.name().c_str(),
				                  navgen_if_->tostring_ConnectionMode(conn_mode));
				navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddPointOfInterestMessage(
				  n.name().c_str(), n.x(), n.y(), conn_mode));
			} else {
				logger->log_debug(name(), "Ignoring irrelevant node %s", n.name().c_str());
			}

			if (do_copy
			    || (n.has_property("only-copy-properties")
			        && n.property_as_bool("only-copy-properties"))) {
				// copy node properties
				for (const auto &p : n.properties()) {
					if (p.first != "only-copy-properties") {
						logger->log_debug(name(),
						                  "    Copying property %s=%s",
						                  p.first.c_str(),
						                  p.second.c_str());
						navgen_if_->msgq_enqueue(
						  new NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage(n.name().c_str(),
						                                                                    p.first.c_str(),
						                                                                    p.second.c_str()));
					}
				}
			}
		}
		for (const NavGraphEdge &e : edges) {
			if (e.has_property("generated"))
				continue;

			std::list<std::string> en = {e.from(), e.to()};
			if (std::all_of(en.begin(), en.end(), [&cn](const std::string &enn) {
				    return std::find(cn.begin(), cn.end(), enn) != cn.end();
			    })) {
				logger->log_debug(name(),
				                  "Copying edge %s-%s%s",
				                  e.from().c_str(),
				                  e.is_directed() ? ">" : "-",
				                  e.to().c_str());
				navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddEdgeMessage(
				  e.from().c_str(),
				  e.to().c_str(),
				  e.is_directed(),
				  NavGraphGeneratorInterface::SPLIT_INTERSECTION));
			} else {
				logger->log_debug(name(),
				                  "Ignoring irrelevant edge %s-%s%s",
				                  e.from().c_str(),
				                  e.is_directed() ? ">" : "-",
				                  e.to().c_str());
			}
		}
	}
	NavGraphGeneratorInterface::ComputeMessage *compute_msg =
	  new NavGraphGeneratorInterface::ComputeMessage();
	compute_msg->ref();
	navgen_if_->msgq_enqueue(compute_msg);
	compute_msgid_ = compute_msg->id();
	compute_msg->unref();
}

void
NavGraphGeneratorMPSThread::bb_interface_data_refreshed(Interface *interface) throw()
{
	navgen_if_->read();
	if (navgen_if_->msgid() == compute_msgid_ && navgen_if_->is_final()) {
		logger->log_info(name(), "Computation finished");
		navgen_mps_if_->set_final(true);
		navgen_mps_if_->write();
	}
}
