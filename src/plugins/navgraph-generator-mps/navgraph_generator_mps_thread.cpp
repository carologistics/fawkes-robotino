
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

#include <navgraph/yaml_navgraph.h>
#include <core/threading/mutex_locker.h>
#include <navgraph/navgraph.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
#include <blackboard/utils/on_message_waker.h>
#include <utils/math/angle.h>
#include <utils/math/eigen.h>
#include <limits>
#include <memory>

using namespace fawkes;

/** @class NavGraphGeneratorMPSThread "navgraph_generator_mps_thread.h"
 * Block navgraph paths based on laser clusters.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphGeneratorMPSThread::NavGraphGeneratorMPSThread()
  : Thread("NavGraphGeneratorMPSThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("NavGraphGeneratorMPSThread")
{
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
  cfg_mps_approach_dist_ = config->get_float("/navgraph-generator-mps/approach-distance");

  std::string base_graph_file;
  try {
    base_graph_file  = config->get_string("/navgraph-generator-mps/base-graph");
  } catch (Exception &e) {} // ignored, no base graph

  base_graph_ = NULL;
  if (! base_graph_file.empty()) {
    if (base_graph_file[0] != '/') {
      base_graph_file = std::string(CONFDIR) + "/" + base_graph_file;
    }
    base_graph_ = load_yaml_navgraph(base_graph_file);
  } else {
    logger->log_warn(name(), "No base graph configured");
  }

  exp_zones_.resize(24, true);
  wait_zones_.resize(24, false);

  navgen_if_ =
    blackboard->open_for_reading<NavGraphGeneratorInterface>("/navgraph-generator");

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
  while ( ! navgen_mps_if_->msgq_empty() ) {
    if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::ClearMessage>() ) {
      stations_.clear();

    } else if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage>() ) {
      NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage *m =
	navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage>();
      logger->log_warn(name(), "Updating station %s from tag", m->id());

      update_station(m->id(), (m->side() == NavGraphWithMPSGeneratorInterface::INPUT),
		     m->frame(), m->tag_translation(), m->tag_rotation());

    } else if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage>() ) {
      NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage *m =
	navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage>();
      for (size_t i = 0; i < std::min(m->maxlenof_zones(), exp_zones_.size()); ++i) {
	exp_zones_[i] = m->is_zones(i);
      }

    } else if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::SetWaitZonesMessage>() ) {
      NavGraphWithMPSGeneratorInterface::SetWaitZonesMessage *m =
	      navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::SetWaitZonesMessage>();
      for (size_t i = 0; i < std::min(m->maxlenof_zones(), wait_zones_.size()); ++i) {
	      wait_zones_[i] = m->is_zones(i);
      }

    } else if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::ComputeMessage>() ) {
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
NavGraphGeneratorMPSThread::update_station(std::string id, bool input, std::string frame,
					   double tag_pos[3], double tag_ori[4])
 {
  // **** 1. convert to map frame
  tf::Stamped<tf::Pose> pose;

  try{
    // Note that we add a correction offset to the centroid X value.
    // This offset is determined empirically and just turns out to be helpful
    // in certain situations.

    tf::Stamped<tf::Pose>
      spose(tf::Pose(tf::Quaternion(tag_ori[0], tag_ori[1], tag_ori[2], tag_ori[3]),
                     tf::Vector3(tag_pos[0], tag_pos[1], tag_pos[2])),
            fawkes::Time(0,0), frame);
    tf_listener->transform_pose(cfg_global_frame_, spose, pose);
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
    Eigen::Vector3f plane_origin(0,0,0);
    Eigen::Vector3f po = tag_pose_pos - plane_origin;
    float lambda = plane_normal.dot(po);
    proj_pos = tag_pose_pos - (lambda * plane_normal);
  }

  Eigen::Quaternionf proj_ori(Eigen::AngleAxisf(tf::get_yaw(tf_pose_ori), Eigen::Vector3f::UnitZ()));

  // **** 3. calculate approach points
  Eigen::Vector3f dir_vec = proj_ori * Eigen::Vector3f::UnitX();
  Eigen::ParametrizedLine<float,3> mline(proj_pos, dir_vec);

  Eigen::Vector3f     input_pos, output_pos, pose_pos;
  Eigen::Quaternionf  input_ori, output_ori, pose_ori;
  if (input) {
    input_pos  = mline.pointAt(cfg_mps_approach_dist_);
    input_ori  = proj_ori * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    output_pos = mline.pointAt(-(cfg_mps_approach_dist_ + cfg_mps_width_));
    output_ori = proj_ori;
    pose_pos   = mline.pointAt(-(.5 * cfg_mps_width_));
    pose_ori   = output_ori;
  } else {
    output_pos = mline.pointAt(cfg_mps_approach_dist_);
    output_ori = proj_ori * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    input_pos  = mline.pointAt(-(cfg_mps_approach_dist_ + cfg_mps_width_));
    input_ori  = proj_ori;
    pose_pos   = mline.pointAt(-(.5 * cfg_mps_width_));
    pose_ori   = input_ori;
  }
  
  // Problem: range of first value is only [0..pi], which might result
  // in "funny" and unexpected angles. If we want only yaw, we can work
  // around this using rpy order... Better use utils.
  //Eigen::Vector3f input_euler  = input_ori.toRotationMatrix().eulerAngles(0, 1, 2);

  /*
  logger->log_info(name(), "P(%f,%f,%f) Q(%f,%f,%f,%f)  D(%f,%f,%f)",
		   proj_pos[0], proj_pos[1], proj_pos[2],
		   proj_ori.x(), proj_ori.y(), proj_ori.z(), proj_ori.w(),
		   dir_vec[0], dir_vec[1], dir_vec[2]);
  logger->log_info(name(), "I(%f,%f,%f)  O(%f,%f,%f)  DI %f  DO %f",
		   input_pos[0], input_pos[1], input_pos[2],
		   output_pos[0], output_pos[1], output_pos[2],
		   (input_pos - proj_pos).norm(), (output_pos - proj_pos).norm());
  */

  // **** 4. add to stations
  MPSStation s;
  s.tag_frame    = frame;
  s.tag_pose_pos = pose_pos;
  s.tag_pose_ori = proj_ori;
  s.pose_pos     = pose_pos;
  s.pose_ori     = pose_ori;
  s.pose_yaw     = quat_yaw(pose_ori);
  s.input_pos    = input_pos;
  s.input_ori    = input_ori;
  s.input_yaw    = quat_yaw(input_ori);
  s.output_pos   = output_pos;
  s.output_ori   = output_ori;
  s.output_yaw   = quat_yaw(output_ori);
  stations_[id]  = s;
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
  navgen_if_->msgq_enqueue(
    new NavGraphGeneratorInterface::SetBoundingBoxMessage(-6, 0, 6, 6));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterMessage
     (NavGraphGeneratorInterface::FILTER_EDGES_BY_MAP, true));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterMessage
     (NavGraphGeneratorInterface::FILTER_ORPHAN_NODES, true));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterMessage
     (NavGraphGeneratorInterface::FILTER_MULTI_GRAPH, true));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterParamFloatMessage
     (NavGraphGeneratorInterface::FILTER_EDGES_BY_MAP, "distance", 0.4));

  navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddMapObstaclesMessage(0.5));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage(true));

  for (const auto &s : stations_) {
    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage
       ((s.first + "-I").c_str(),
	s.second.input_pos[0], s.second.input_pos[1], s.second.input_yaw,
	mps_node_insmode(s.first + "-I")));

    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage
       ((s.first + "-O").c_str(),
	s.second.output_pos[0], s.second.output_pos[1], s.second.output_yaw,
	mps_node_insmode(s.first + "-O")));

    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddPointOfInterestMessage
       (s.first.c_str(),
	s.second.pose_pos[0], s.second.pose_pos[1],
	NavGraphGeneratorInterface::UNCONNECTED));

    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddObstacleMessage
       ((s.first + "-C").c_str(), s.second.pose_pos[0], s.second.pose_pos[1]));
  }

  if (base_graph_) {
    // add base graph nodes and edges
    std::list<std::string> cn; // copied nodes
    const std::vector<NavGraphNode> &nodes = base_graph_->nodes();
    const std::vector<NavGraphEdge> &edges = base_graph_->edges();
    for (const NavGraphNode &n : nodes) {
      bool do_copy = n.has_property("always-copy") && n.property_as_bool("always-copy");
      if (! do_copy) {
	// check for other reasons
	for (size_t i = 0; i < exp_zones_.size(); ++i) {
	  if (exp_zones_[i]) {
	    std::string prop_name = "orientation_Z" + std::to_string(i+1);
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
	    logger->log_warn(name(), "Unknown insertion mode '%s' for node '%s', "
			     "defaulting to NOT_CONNECTED", ins_mode.c_str(), n.name().c_str());
	  }
	}

	logger->log_debug(name(), "Copying node %s (insertion mode %s)", n.name().c_str(),
			  navgen_if_->tostring_ConnectionMode(conn_mode));
	navgen_if_->msgq_enqueue
	  (new NavGraphGeneratorInterface::AddPointOfInterestMessage
	   (n.name().c_str(), n.x(), n.y(), conn_mode));
      } else {
	logger->log_debug(name(), "Ignoring irrelevant node %s", n.name().c_str());
      }

      if (do_copy ||
	  (n.has_property("only-copy-properties") &&
	   n.property_as_bool("only-copy-properties")))
      {
	// copy node properties
	for (const auto &p : n.properties()) {
	  if (p.first != "only-copy-properties") {
	    logger->log_debug(name(), "    Copying property %s=%s",
			      p.first.c_str(), p.second.c_str());
	    navgen_if_->msgq_enqueue
	      (new NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage
	       (n.name().c_str(), p.first.c_str(), p.second.c_str()));
	  }
	}
      }

    }
    for (const NavGraphEdge &e : edges) {
      if (e.has_property("generated")) continue;

      std::list<std::string> en = {e.from(), e.to()};     
      if (std::all_of(en.begin(), en.end(),
		      [&cn](const std::string &enn)
		      { return std::find(cn.begin(), cn.end(), enn) != cn.end(); }))
      {
	logger->log_debug(name(), "Copying edge %s-%s%s",
			  e.from().c_str(), e.is_directed() ? ">" : "-", e.to().c_str());
	navgen_if_->msgq_enqueue
	  (new NavGraphGeneratorInterface::AddEdgeMessage
	   (e.from().c_str(), e.to().c_str(), e.is_directed(),
	    NavGraphGeneratorInterface::SPLIT_INTERSECTION));
      } else {
	logger->log_debug(name(), "Ignoring irrelevant edge %s-%s%s",
			  e.from().c_str(), e.is_directed() ? ">" : "-", e.to().c_str());
      }
    }
  }

	for (size_t i = 0; i < wait_zones_.size(); ++i) {
		if (wait_zones_[i]) {
			// calc zone coordinates
			int col = (i / 4); // col is 0-based and is in range [0..2]
			int col_sign = 1;
			if (i > 2)  {
				// magenta side, negative X coordinates
				i = i - 3;
				col_sign = -1;
			}
			int row = i - (i / 4) * 4;

			float from_x = col_sign *  col      * 2.;
			float to_x   = col_sign * (col + 1) * 2.;
			float from_y =  row      * 1.5;
			float to_y   = (row + 1) * 1.5;

			float center_x = from_x + fabsf(from_x - to_x) / 2. * col_sign;
			float center_y = from_y + fabsf(from_y - to_y) / 2.;

			navgen_if_->msgq_enqueue
				(new NavGraphGeneratorInterface::AddPointOfInterestMessage
				 (NavGraph::format_name("WAIT-Z%zu", i).c_str(), center_x, center_y,
				  NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE)); 
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
NavGraphGeneratorMPSThread::bb_interface_data_changed(Interface *interface) throw()
{
  navgen_if_->read();
  if (navgen_if_->msgid() == compute_msgid_ && navgen_if_->is_final()) {
    logger->log_info(name(), "Computation finished");
    navgen_mps_if_->set_final(true);
    navgen_mps_if_->write();
  }
}
