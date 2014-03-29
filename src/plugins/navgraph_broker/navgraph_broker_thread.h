
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
#include <plugins/navgraph/aspect/navgraph.h>
#include <plugins/navgraph/constraints/constraint_repo.h>
#include <utils/graph/topological_map_graph.h>
#include <string>

namespace fawkes{
	class Time;
	class TopologicalMapGraph;
	class ConstraintRepo;
}

class NavgraphBrokerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect
{
 public:
  NavgraphBrokerThread();
  virtual ~NavgraphBrokerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
 	 std::string robot_name_;

 private: // methods


};

#endif
