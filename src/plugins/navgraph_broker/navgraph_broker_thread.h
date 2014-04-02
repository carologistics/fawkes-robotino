
/***************************************************************************
 *  navgraph_broker_thread.h
 *
 *  Created on Sat March 29th 12:24:01 2014
 *  Copyright (C) 2014 by Sebastian reuter, Carologistics RoboCup Team]
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

#ifndef __PLUGINS_NAVGRAPH_BROKER_THREAD_H_
#define __PLUGINS_NAVGRAPH_BROKER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <plugins/navgraph/aspect/navgraph.h>
#include <plugins/navgraph/constraints/constraint_repo.h>
#include <utils/graph/topological_map_graph.h>
#include <string>

#include <interfaces/NavPathInterface.h>

namespace fawkes{
	class Time;
	class TopologicalMapGraph;
	class ConstraintRepo;
	class NavPathInterface;
}

class NavgraphBrokerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::NavGraphAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  NavgraphBrokerThread();
  virtual ~NavgraphBrokerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // For BlackBoardInterfaceListener
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

  //
  void reserve_nodes(std::string robot_name, std::vector<fawkes::TopologicalMapNode> path);

 private:
  std::vector<fawkes::TopologicalMapNode> get_nodes_from_string(std::string path);
  std::string get_string_from_nodes(std::vector<fawkes::TopologicalMapNode> path);
  std::string get_path_from_interface_as_string();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
 	 std::string robot_name_;
     fawkes::NavPathInterface *path_if_;

 private: // methods


};

#endif
