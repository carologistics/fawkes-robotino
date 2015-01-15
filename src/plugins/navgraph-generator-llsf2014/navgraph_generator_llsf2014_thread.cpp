
/***************************************************************************
 *  navgraph_generator_llsf2014_thread.cpp - Generate navgraph for LLSF2014
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

#include "navgraph_generator_llsf2014_thread.h"

#include <navgraph/generators/voronoi.h>
#include <navgraph/yaml_navgraph.h>
#include <core/threading/mutex_locker.h>
#include <limits>
#include <memory>

using namespace fawkes;

/** @class NavGraphGenerator2014Thread "navgraph_clusters_thread.h"
 * Block navgraph paths based on laser clusters.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphGenerator2014Thread::NavGraphGenerator2014Thread()
  : Thread("NavGraphGenerator2014Thread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavGraphGenerator2014Thread::~NavGraphGenerator2014Thread()
{
}

void
NavGraphGenerator2014Thread::init()
{
  NavGraphGeneratorVoronoi nggv(-5.6, 0, 5.6, 5.6);

  nggv.add_obstacle(0.56, 1.68);
  nggv.add_obstacle(0.56, 2.80);
  nggv.add_obstacle(1.68, 1.68);
  nggv.add_obstacle(1.68, 2.80);
  nggv.add_obstacle(1.68, 3.92);
  nggv.add_obstacle(2.80, 1.68);
  nggv.add_obstacle(2.80, 3.92);
  nggv.add_obstacle(2.80, 5.04);
  nggv.add_obstacle(3.92, 1.68);
  nggv.add_obstacle(3.92, 2.80);
  nggv.add_obstacle(3.92, 5.04);
  nggv.add_obstacle(5.04, 5.04);
  nggv.add_obstacle(0.56, 5.04);
  nggv.add_obstacle(5.34, 2.45);
  nggv.add_obstacle(5.34, 2.80);
  nggv.add_obstacle(5.34, 3.15);

  nggv.add_obstacle(-0.56, 1.68);
  nggv.add_obstacle(-0.56, 2.80);
  nggv.add_obstacle(-1.68, 1.68);
  nggv.add_obstacle(-1.68, 2.80);
  nggv.add_obstacle(-1.68, 3.92);
  nggv.add_obstacle(-2.80, 1.68);
  nggv.add_obstacle(-2.80, 3.92);
  nggv.add_obstacle(-2.80, 5.04);
  nggv.add_obstacle(-3.92, 1.68);
  nggv.add_obstacle(-3.92, 2.80);
  nggv.add_obstacle(-3.92, 5.04);
  nggv.add_obstacle(-5.04, 5.04);
  nggv.add_obstacle(-0.56, 5.04);
  nggv.add_obstacle(-5.34, 2.45);
  nggv.add_obstacle(-5.34, 2.80);
  nggv.add_obstacle(-5.34, 3.15);

  // outer boundaries
  nggv.add_obstacle(0, 0);
  nggv.add_obstacle(0, 5.6);
  nggv.add_obstacle(-5.6, 0.0);
  nggv.add_obstacle(-5.6, 5.6);
  nggv.add_obstacle(5.6, 0.0);
  nggv.add_obstacle(5.6, 5.6);

  // along machine positions on edges
  // along bottom edge, cyan
  nggv.add_obstacle(0.56, 0.56);
  nggv.add_obstacle(1.68, 0.56);
  nggv.add_obstacle(2.80, 0.56);
  nggv.add_obstacle(3.92, 0.56);
  nggv.add_obstacle(5.04, 0.56);
  // along bottom edge, magenta
  nggv.add_obstacle(-0.56, 0.56);
  nggv.add_obstacle(-1.68, 0.56);
  nggv.add_obstacle(-2.80, 0.56);
  nggv.add_obstacle(-3.92, 0.56);
  nggv.add_obstacle(-5.04, 0.56);

  // along top edge, cyan
  nggv.add_obstacle(0.56, 5.6);
  //nggv.add_obstacle(1.68, 5.6);
  nggv.add_obstacle(2.80, 5.6);
  nggv.add_obstacle(3.92, 5.6);
  nggv.add_obstacle(5.04, 5.6);

  // along top edge, magenta
  nggv.add_obstacle(-0.56, 5.6);
  //nggv.add_obstacle(-1.68, 5.6);
  nggv.add_obstacle(-2.80, 5.6);
  nggv.add_obstacle(-3.92, 5.6);
  nggv.add_obstacle(-5.04, 5.6);

  // along right edge, only where there is machine
  //nggv.add_obstacle(5.6, 1.68);
  //nggv.add_obstacle(5.6, 2.80);
  //nggv.add_obstacle(5.6, 3.92);
  nggv.add_obstacle(5.6, 5.04);

  // along left edge, only where there is machine
  //nggv.add_obstacle(-5.6, 1.68);
  //nggv.add_obstacle(-5.6, 2.80);
  //nggv.add_obstacle(-5.6, 3.92);
  nggv.add_obstacle(-5.6, 5.04);

  MutexLocker lock(navgraph.objmutex_ptr());

  logger->log_info(name(), "Clearing");
  navgraph->clear();
  logger->log_info(name(), "Computing");
  nggv.compute(navgraph);

  logger->log_info(name(), "Copying");
  std::string cfg_graph_file = config->get_string("/navgraph/graph_file");
  if (cfg_graph_file[0] != '/') {
    cfg_graph_file = std::string(CONFDIR) + "/" + cfg_graph_file;
  }
  std::shared_ptr<NavGraph> file_graph(load_yaml_navgraph(cfg_graph_file));

  for (unsigned int i = 1; i <= 24; ++i) {
    NavGraphNode n = file_graph->node(NavGraph::format_name("M%u", i));
    if (n) add_node_edge(n);
  }

  for (unsigned int i = 1; i <= 24; ++i) {
    NavGraphNode n = file_graph->node(NavGraph::format_name("ExpM%u", i));
    if (n) add_node_edge(n);
  }

  for (unsigned int i = 1; i <= 2; ++i) {
    for (unsigned int j = 1; i <= 2; ++i) {
      NavGraphNode n = file_graph->node(NavGraph::format_name("D%u_PUCK_STORAGE_%u", i, j));
      if (n) add_node_edge(n);
    }
  }

  for (unsigned int i = 1; i <= 2; ++i) {
    for (unsigned int j = 1; i <= 3; ++i) {
      NavGraphNode n = file_graph->node(NavGraph::format_name("WAIT_FOR_INS_%u_ROBOTINO_%u", i, j));
      if (n) add_node_edge(n);
    }
  }

  for (unsigned int i = 1; i <= 2; ++i) {
    NavGraphNode n = file_graph->node(NavGraph::format_name("deliver%u", i));
    if (n) add_node_node(n);
    NavGraphNode n2 = file_graph->node(NavGraph::format_name("deliver%ua", i));
    if (n2) add_node_node(n2);
    NavGraphNode n3 = file_graph->node(NavGraph::format_name("WAIT_FOR_DELIVER_%u", i));
    if (n3) add_node_edge(n3);
  }
  for (unsigned int i = 1; i <= 2; ++i) {
    NavGraphNode n = file_graph->node(NavGraph::format_name("Ins%u", i));
    if (n) add_node_edge(n);
    NavGraphNode n2 = file_graph->node(NavGraph::format_name("Ins%uSec", i));
    if (n2) add_node_edge(n2);
  }

  navgraph->set_default_properties(file_graph->default_properties());

  try {
    logger->log_info(name(), "Calc Reachability");
    navgraph->calc_reachability();
  } catch (Exception &e) {
    logger->log_error(name(), "Graph failed");
    logger->log_error(name(), e);
  }
  lock.unlock();
  logger->log_info(name(), "Done");
  navgraph->notify_of_change();
}


/** Add node, connect to closest edge.
 * @param n node to add
 */
void
NavGraphGenerator2014Thread::add_node_edge(const NavGraphNode &n)
{
  NavGraphEdge closest = navgraph->closest_edge(n.x(), n.y());
  cart_coord_2d_t p = closest.closest_point_on_edge(n.x(), n.y());

  NavGraphNode closest_conn = navgraph->closest_node(p.x, p.y);
  NavGraphNode cn;
  if (closest_conn.distance(p.x, p.y) < std::numeric_limits<float>::epsilon()) {
    cn = closest_conn;
    printf("Re-using node %s\n", cn.name().c_str());
  } else {
    cn = NavGraphNode(NavGraph::format_name("A_%s", n.name().c_str()), p.x, p.y);
    cn.set_property("highway_exit", true);
    printf("Adding node %s\n", cn.name().c_str());
  }

  navgraph->add_node(n);

  if (closest.from() == cn.name() || closest.to() == cn.name()) {
    // we actually want to connect to one of the end nodes of the edge,
    // simply add the new edge and we are done
    printf("Connecting to endpoint of existing edge %s--%s: %s--%s\n",
	   closest.from().c_str(), closest.to().c_str(),
	   cn.name().c_str(), n.name().c_str());
    NavGraphEdge new_edge(cn.name(), n.name());
    new_edge.set_property("generated", true);
    navgraph->add_edge(new_edge);
  } else {
    // we are inserting a new point into the edge

    navgraph->remove_edge(closest);
    NavGraphEdge new_edge_1(closest.from(), cn.name());
    NavGraphEdge new_edge_2(closest.to(), cn.name());
    NavGraphEdge new_edge_3(cn.name(), n.name());
    new_edge_1.set_property("generated", true);
    new_edge_2.set_property("generated", true);
    new_edge_3.set_property("generated", true);

    printf("Splitting %s--%s\n", closest.from().c_str(), closest.to().c_str());
    printf("Adding    %s--%s\n", new_edge_1.from().c_str(), new_edge_1.to().c_str());
    printf("Adding    %s--%s\n", new_edge_2.from().c_str(), new_edge_2.to().c_str());
    printf("Adding    %s--%s\n", new_edge_3.from().c_str(), new_edge_3.to().c_str());

    if (! navgraph->node_exists(cn))  navgraph->add_node(cn);
    navgraph->add_edge(new_edge_1);
    navgraph->add_edge(new_edge_2);
    navgraph->add_edge(new_edge_3);
  }

}

/** Add node, connect to closest node.
 * @param n node to add
 */
void
NavGraphGenerator2014Thread::add_node_node(const NavGraphNode &n)
{
  NavGraphNode closest = navgraph->closest_node(n.x(), n.y());
  closest.set_property("highway_exit", true);
  navgraph->update_node(closest);
  navgraph->add_node(n);
  navgraph->add_edge(NavGraphEdge(n.name(), closest.name()));
}

void
NavGraphGenerator2014Thread::finalize()
{
}

void
NavGraphGenerator2014Thread::loop()
{
}


