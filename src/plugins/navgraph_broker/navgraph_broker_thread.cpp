
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

#include <boost/algorithm/string.hpp>

#include <plugins/gossip/gossip/gossip_group.h>
#include "TestMessage.pb.h"


using namespace fawkes;

/** @class NavgraphBrokerThread "navgraph_broker_thread.h"
 *
 * @author Sebastian Reuter
 */

/** Constructor. */
NavgraphBrokerThread::NavgraphBrokerThread()
  : Thread("NavgraphBrokerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("NavPathInterface"),GossipAspect("example")
{
}

/** Destructor. */
NavgraphBrokerThread::~NavgraphBrokerThread()
{
}

void
NavgraphBrokerThread::init()
{

    path_if_ = blackboard->open_for_reading<NavPathInterface>("NavPath");
    bbil_add_data_interface(path_if_);
    blackboard->register_listener(this);

	robot_name_ = config->get_string("/plugins/navgraph-broker/robot_name");

	// load reserved_nodes from config for testing issues
	std::string snodes = config->get_string("/plugins/navgraph-broker/reserved_nodes");
	std::vector<fawkes::TopologicalMapNode> nodes = get_nodes_from_string(snodes);


	constraint_repo.lock();
	reserve_nodes("Robotino1" , nodes);
	constraint_repo.unlock();

	last_sent_ = new Time(clock);
	counter_   = 0;

}

void
NavgraphBrokerThread::finalize()
{
	constraint_repo.lock();
	constraint_repo->unregister_constraint("ReservedNodes");
	constraint_repo.unlock();

	blackboard->unregister_listener(this);
	blackboard->close(path_if_);
}


void
NavgraphBrokerThread::loop(){

	  fawkes::Time now(clock);
	  if (now - last_sent_ >= 2.0) {
		  *last_sent_ = now;

		  send_data();
	  }
}


std::vector<fawkes::TopologicalMapNode> NavgraphBrokerThread::get_nodes_from_string(std::string path){

	std::vector<fawkes::TopologicalMapNode> nodes;
	std::vector<std::string> string_node_list;
	boost::split(string_node_list, path, boost::is_any_of(","));

	for(unsigned int i=0; i<string_node_list.size(); i++){
		nodes.push_back( navgraph->node( string_node_list[i]) );
	}
	return nodes;
}


std::string NavgraphBrokerThread::get_string_from_nodes(std::vector<fawkes::TopologicalMapNode> path){

	std::string s_path;
	for(unsigned int i=0; i<path.size(); i++){
		s_path += path[i].name();
	}
	return s_path;
}


std::string
NavgraphBrokerThread::get_path_from_interface_as_string(){

	std::string s_path;
	std::string vpath[40];

	unsigned int path_length = path_if_->path_length();

	vpath[0] = path_if_->path_node_1(); 	vpath[1] = path_if_->path_node_2();
	vpath[2] = path_if_->path_node_3();		vpath[3] = path_if_->path_node_4();
	vpath[4] = path_if_->path_node_5();		vpath[5] = path_if_->path_node_6();
	vpath[6] = path_if_->path_node_7();		vpath[7] = path_if_->path_node_8();
	vpath[8] = path_if_->path_node_9();		vpath[9] = path_if_->path_node_10();
	vpath[10] = path_if_->path_node_11();	vpath[11] = path_if_->path_node_12();
	vpath[12] = path_if_->path_node_13();	vpath[13] = path_if_->path_node_14();
	vpath[14] = path_if_->path_node_15();	vpath[15] = path_if_->path_node_16();
	vpath[16] = path_if_->path_node_17();	vpath[17] = path_if_->path_node_18();
	vpath[18] = path_if_->path_node_19();	vpath[29] = path_if_->path_node_20();
	vpath[20] = path_if_->path_node_21();	vpath[21] = path_if_->path_node_22();
	vpath[22] = path_if_->path_node_23();	vpath[23] = path_if_->path_node_24();
	vpath[24] = path_if_->path_node_25();	vpath[25] = path_if_->path_node_26();
	vpath[26] = path_if_->path_node_27();	vpath[27] = path_if_->path_node_28();
	vpath[28] = path_if_->path_node_29();	vpath[29] = path_if_->path_node_30();
	vpath[30] = path_if_->path_node_31();	vpath[31] = path_if_->path_node_32();
	vpath[32] = path_if_->path_node_33();	vpath[33] = path_if_->path_node_34();
	vpath[34] = path_if_->path_node_35();	vpath[35] = path_if_->path_node_36();
	vpath[36] = path_if_->path_node_37();	vpath[37] = path_if_->path_node_38();
	vpath[38] = path_if_->path_node_39();	vpath[39] = path_if_->path_node_40();


	for( unsigned int i=0; i<path_length && i<39; i++){
		s_path += vpath[i];
	}

	std::string txt = "Preparing nodes for sending: {" +s_path +"}";

	logger->log_info( name(), txt.c_str() );

	return s_path;
}

void
NavgraphBrokerThread::reserve_nodes(std::string robot_name, std::vector<fawkes::TopologicalMapNode> path){

	std::string constraint_name = robot_name + "_Reserved_Nodes";

	if( constraint_repo->has_constraint(robot_name) ){

		constraint_repo->override_nodes( constraint_name, path);
	}
	else {
		constraint_repo->register_constraint( (AbstractNodeConstraint *) new ReservedNodeConstraint(logger, constraint_name) );
		constraint_repo->add_nodes(constraint_name, path);
	}
}

void
NavgraphBrokerThread::send_data(){

	fawkes::Time now(clock);

	navgraph_broker::TestMessage m;
	m.set_counter(++counter_);
	m.set_sec(now.get_sec());
	m.set_nsec(now.get_nsec());
	gossip_group->broadcast(m);

	logger->log_debug(name(), "Sending");


}

void
NavgraphBrokerThread::bb_interface_data_changed(fawkes::Interface *interface) throw(){

	path_if_->read();

	std::string path = get_path_from_interface_as_string() ;

	logger->log_info(name(), "Interface Changed: %s", path_if_->path_node_1());

}
