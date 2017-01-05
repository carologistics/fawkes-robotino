
/***************************************************************************
 *  clips_smt_thread.cpp -  Smt feature for CLIPS
 *
 *  Created: Created on Fry Dec 16 14:48 2016 by Igor Nicolai Bongartz
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

#include "clips_smt_thread.h"

#include <navgraph/navgraph.h>
#include <navgraph/constraints/static_list_edge_constraint.h>
#include <navgraph/constraints/constraint_repo.h>

#include <clipsmm.h>

using namespace fawkes;

/** @class ClipsNavGraphThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsSmtThread::ClipsSmtThread()
  : Thread("ClipsSmtThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSFeature("smt"), CLIPSFeatureAspect(this) // CLIPSFeature("navgraph") before
{
}


/** Destructor. */
ClipsSmtThread::~ClipsSmtThread()
{
}


void
ClipsSmtThread::init()
{
}


void
ClipsSmtThread::finalize()
{
  envs_.clear();
}


void
ClipsSmtThread::clips_context_init(const std::string &env_name,
					LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

  clips.lock();
  //clips->batch_evaluate(SRCDIR"/clips/navgraph.clp");
  //clips_smt_load(clips);

  clips->add_function("clips_smt_dummy",
    sigc::slot<void, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_dummy),
  env_name)
    )
  );

 /**
  clips->add_function("navgraph-block-edge",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_block_edge),
	env_name)
    )
  );

  clips->add_function("navgraph-unblock-edge",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_unblock_edge),
	env_name)
    )
  );
  **/

  clips.unlock();
}

void
ClipsSmtThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

/**
void
ClipsSmtThread::clips_smt_load(LockPtr<CLIPS::Environment> &clips)
{
  try {
    const std::vector<NavGraphNode> &nodes = navgraph->nodes();
    const std::vector<NavGraphEdge> &edges = navgraph->edges();

    clips->assert_fact_f("(navgraph (name \"%s\"))", navgraph->name().c_str());

    for (const NavGraphNode &n : nodes) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = n.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-node (name \"%s\") (pos %f %f) (properties %s))",
			   n.name().c_str(), n.x(), n.y(), props_string.c_str());
    }

    for (const NavGraphEdge &e : edges) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = e.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-edge (from \"%s\") (to \"%s\") (directed %s) "
			   "(properties %s))",
			   e.from().c_str(), e.to().c_str(),
			   e.is_directed() ? "TRUE" : "FALSE", props_string.c_str());
    }

  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to assert navgraph, exception follows");
    logger->log_warn(name(), e);
    clips->assert_fact_f("(navgraph-load-fail %s)", *(e.begin()));
  }
}


void
ClipsSmtThread::clips_smt_block_edge(std::string env_name,
					       std::string from, std::string to)
{
  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();

  for (const NavGraphEdge &edge : graph_edges) {
    if (edge.from() == from && edge.to() == to) {
      edge_constraint_->add_edge(edge);
      return;
    }
  }

  logger->log_warn(name(), "Environment %s tried to block edge %s--%s, "
		   "which does not exist in graph", env_name.c_str(),
		   from.c_str(), to.c_str());
}


void
ClipsSmtThread::clips_smt_unblock_edge(std::string env_name,
						 std::string from, std::string to)
{
  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();

  for (const NavGraphEdge &edge : graph_edges) {
    if (edge.from() == from && edge.to() == to) {
      edge_constraint_->remove_edge(edge);
      return;
    }
  }

  logger->log_warn(name(), "Environment %s tried to unblock edge %s--%s, "
		   "which does not exist in graph", env_name.c_str(),
		   from.c_str(), to.c_str());
}
**/

void
ClipsSmtThread::clips_smt_dummy(std::string foo, std::string bar)
{
  std::string cfg_base_frame_      = config->get_string("/frames/base");
  std::string cfg_global_frame_    = config->get_string("/frames/fixed");
  std::string cfg_nav_if_id_       = config->get_string("/navgraph/navigator_interface_id");
  float cfg_resend_interval_ = config->get_float("/navgraph/resend_interval");
  float cfg_replan_interval_ = config->get_float("/navgraph/replan_interval");
  float cfg_replan_factor_   = config->get_float("/navgraph/replan_cost_factor");
  float cfg_target_time_     = config->get_float("/navgraph/target_time");
  float cfg_target_ori_time_ = config->get_float("/navgraph/target_ori_time");
  bool cfg_log_graph_       = config->get_bool("/navgraph/log_graph");
  bool cfg_abort_on_error_  = config->get_bool("/navgraph/abort_on_error");
  std::cout << "This is a smt test message: " << foo << " and " << bar << std::endl;
  std::cout << "Moreover we test to get config values "<< std::endl;
  std::cout << "cfg_base_frame_ " << cfg_base_frame_ << std::endl;
  std::cout << "cfg_global_frame_ " << cfg_global_frame_ << std::endl;
  std::cout << "cfg_nav_if_id_ " << cfg_nav_if_id_ << std::endl;
  std::cout << "cfg_resend_interval_ " << cfg_resend_interval_ << std::endl;
  std::cout << "cfg_replan_interval_ " << cfg_replan_interval_ << std::endl;
  std::cout << "cfg_replan_factor_ " << cfg_replan_factor_ << std::endl;
  std::cout << "cfg_target_time_ " << cfg_target_time_ << std::endl;
  std::cout << "cfg_target_ori_time_ " << cfg_target_ori_time_ << std::endl;
  std::cout << "cfg_log_graph_ " << cfg_log_graph_ << std::endl;
  std::cout << "cfg_abort_on_error_ " << cfg_abort_on_error_ << std::endl;
}

void
ClipsSmtThread::loop()
{
}
