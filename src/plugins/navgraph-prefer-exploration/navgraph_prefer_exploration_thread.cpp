
/***************************************************************************
 *  navgraph_prefer_exploration_thread.h - prefer exploration sub-graph
 *
 *  Created: Thu Apr 09 16:55:05 2015
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

#include "navgraph_prefer_exploration_thread.h"

#include <core/threading/mutex_locker.h>
#include <navgraph/constraints/constraint_repo.h>
#include <navgraph/constraints/static_list_edge_cost_constraint.h>
#include <navgraph/navgraph.h>
#include <navgraph/yaml_navgraph.h>

using namespace fawkes;

/** @class NavGraphPreferExplorationThread
 * "navgraph_prefer_exploration_thread.h" Increase costs on edges not in
 * exploration base graph.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphPreferExplorationThread::NavGraphPreferExplorationThread()
: Thread("NavGraphPreferExplorationThread", Thread::OPMODE_WAITFORWAKEUP)
{
	set_coalesce_wakeups(true);
}

/** Destructor. */
NavGraphPreferExplorationThread::~NavGraphPreferExplorationThread()
{
}

void
NavGraphPreferExplorationThread::init()
{
	cfg_cost_factor_ = config->get_float("/navgraph-prefer-exploration/cost-factor");

	edge_cost_constraint_ = new NavGraphStaticListEdgeCostConstraint("prefer-expl");
	navgraph->constraint_repo()->register_constraint(edge_cost_constraint_);

	navgraph->add_change_listener(this);

	// wakeup();
}

void
NavGraphPreferExplorationThread::finalize()
{
	navgraph->remove_change_listener(this);
	navgraph->constraint_repo()->unregister_constraint(edge_cost_constraint_->name());
	delete edge_cost_constraint_;
}

void
NavGraphPreferExplorationThread::loop()
{
	MutexLocker lock(navgraph.objmutex_ptr());

	bool has_exploration_nodes = false;

	std::vector<std::pair<fawkes::NavGraphEdge, float>> cedges;

	const std::vector<NavGraphEdge> &edges = navgraph->edges();
	for (const NavGraphEdge &e : edges) {
		if (!e.has_property("created-for")) {
			cedges.push_back(std::make_pair(e, cfg_cost_factor_));
		} else {
			std::string            created_for = e.property("created-for");
			std::string            n1, n2;
			std::string::size_type pos;
			if ((pos = created_for.find("--")) != std::string::npos) {
				n1 = created_for.substr(0, pos);
				n2 = created_for.substr(pos + 2);
			}

			if (!n1.empty() && !n2.empty()) {
				if (n1.find("Exp-") == 0 || n2.find("Exp-") == 0) {
					has_exploration_nodes = true;
				} else {
					cedges.push_back(std::make_pair(e, cfg_cost_factor_));
				}
			}
		}
	}

	if (has_exploration_nodes) {
		edge_cost_constraint_->add_edges(cedges);
	} else {
		edge_cost_constraint_->clear_edges();
	}
}

void
NavGraphPreferExplorationThread::graph_changed() throw()
{
	wakeup();
}
