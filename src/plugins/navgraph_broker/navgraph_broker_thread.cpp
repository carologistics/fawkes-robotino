
/***************************************************************************
 *  clips_ros_thread.cpp -  ROS integration for CLIPS
 *
 *  Created: Tue Oct 22 18:14:41 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_broker_thread.h"
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

using namespace fawkes;

/** @class NavgraphBrokerThread "navgraph_broker_thread.h"
 *
 * @author Sebastian Reuter
 */

/** Constructor. */
NavgraphBrokerThread::NavgraphBrokerThread()
  : Thread("NavgraphBrokerThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavgraphBrokerThread::~NavgraphBrokerThread()
{
}

void
NavgraphBrokerThread::init()
{

	robot_name_ = config->get_string("/plugins/navgraph-broker/robot_name");

	// load reserved_nodes from config for testing issues
	std::string snodes = config->get_string("/plugins/navgraph-broker/reserved_nodes");
	std::vector<std::string> snodes_list;
	boost::split(snodes_list, snodes, boost::is_any_of(","));

	// create Constraint
	constraint_repo->register_constraint( (AbstractNodeConstraint *) new ReservedNodeConstraint(logger, "ReservedNodes") );

	// fill Constraint with Nodes from Config
	for(unsigned int i=0; i<snodes_list.size(); i++){
		logger->log_info(name(), "Add Node %s", navgraph->node( snodes_list[i] ).name().c_str() );
		constraint_repo->get_constraint("ReservedNodes")->add_node( navgraph->node( snodes_list[i]) );
	}

}

void
NavgraphBrokerThread::finalize()
{
}

void
NavgraphBrokerThread::loop()
{
}
