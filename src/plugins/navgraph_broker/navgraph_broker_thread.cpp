
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

#include <core/threading/mutex_locker.h>

#include <boost/algorithm/string.hpp>

using namespace fawkes;
using namespace boost::placeholders;

/** @class NavgraphBrokerThread "navgraph_broker_thread.h"
 *
 * @author Sebastian Reuter
 */

/** Constructor. */
NavgraphBrokerThread::NavgraphBrokerThread()
: Thread("NavgraphBrokerThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
  BlackBoardInterfaceListener("NavPathInterface"),
  GossipAspect("example")
{
}

/** Destructor. */
NavgraphBrokerThread::~NavgraphBrokerThread()
{
}

void
NavgraphBrokerThread::init()
{
	// initialize interfaces
	path_if_ = blackboard->open_for_reading<NavPathInterface>("NavPath");
	bbil_add_data_interface(path_if_);
	blackboard->register_listener(this);

	robotname_ = config->get_string("/plugins/navgraph-broker/robotname");
	// get robot-name from host.yaml
	try {
		robotname_ = config->get_string("/robotname");
	} catch (Exception &e) {
		logger->log_error(name(),
		                  "Can't read the robot name. Is 'robot-name' "
		                  "specified in cfg/host.yaml ?");
	}
	repeat_send_duration_ =
	  (double)config->get_float("/plugins/navgraph-broker/repeat-send-duration");
	max_reservation_duration_ =
	  (double)config->get_float("/plugins/navgraph-broker/max-reservation-duration");
	use_node_constraints_ = config->get_bool("/plugins/navgraph-broker/use-node-constraint");

	m_mutex_ = new Mutex();
	m_       = new navgraph_broker::NavigationMessage();
	m_->set_robotname(robotname_.c_str());
	last_message_time_sec_  = 0;
	last_message_time_nsec_ = 0;

	/******************************************************************************************
   * ***************************************************************************************
   * ***************************************************************************************
   * **************************** Test Nodes for reservation
   *****************************************
   * ***************************************************************************************
   ****************************************************************************************/
	// load reserved_nodes from config for testing issues

	if (use_node_constraints_) {
		try {
			std::string snodes = config->get_string("/plugins/navgraph-broker/reserved_nodes");

			std::vector<fawkes::NavGraphNode> nodes = get_nodes_from_string(snodes);

			if (nodes.size() > 0) {
				std::string txt = "{";
				for (uint16_t i = 0; i < nodes.size(); i++) {
					txt += nodes[i].name();
					txt += ",";
				}
				txt.erase(txt.length() - 1, 1);
				txt += "}";
				logger->log_info(name(), "Reserving test nodes  %s", txt.c_str());
			}

			fawkes::Time                                               now(clock);
			std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> timed_nodes;
			double       max_reservation_duration = (double)max_reservation_duration_;
			fawkes::Time valid_duration           = now + max_reservation_duration;
			for (fawkes::NavGraphNode &node : nodes) {
				timed_nodes.push_back(std::pair<fawkes::NavGraphNode, fawkes::Time>(node, valid_duration));
			}

			reserve_nodes("test", timed_nodes);

		} catch (Exception &e) {
			logger->log_error(name(),
			                  "No static nodes reserved. Did you miss to specify "
			                  "'reserved_nodes' in navgraph.broker.yaml ?");
		}
	}

	/******************************************************************************************
   * ***************************************************************************************
   * ***************************************************************************************
   * ***************************************************************************************
   * ***************************************************************************************
   ****************************************************************************************/

	// init gossip receiver
	try {
		gossip_group->message_register().add_message_type<navgraph_broker::NavigationMessage>();
	} catch (std::runtime_error &e) {
	} // ignore, probably already added

	sig_rcvd_conn_ = gossip_group->signal_received().connect(
	  boost::bind(&NavgraphBrokerThread::handle_peer_msg, this, _1, _2, _3, _4));

	sig_recv_error_conn_ = gossip_group->signal_recv_error().connect(
	  boost::bind(&NavgraphBrokerThread::handle_peer_recv_error, this, _1, _2));

	sig_send_error_conn_ = gossip_group->signal_send_error().connect(
	  boost::bind(&NavgraphBrokerThread::handle_peer_send_error, this, _1));
}

void
NavgraphBrokerThread::finalize()
{
	/*
   * ToDo: Keep registered constraints in mind and unregister them on finalize
   */
	// constraint_repo.lock();
	// constraint_repo->unregister_constraint(robotname_.c_str());
	// constraint_repo.unlock();

	blackboard->unregister_listener(this);
	blackboard->close(path_if_);

	sig_rcvd_conn_.disconnect();
	sig_recv_error_conn_.disconnect();
	sig_send_error_conn_.disconnect();

	delete m_mutex_;
	delete m_;
}

void
NavgraphBrokerThread::loop()
{
	if (!reservation_messages_.empty()) {
		std::shared_ptr<navgraph_broker::NavigationMessage> msg = reservation_messages_.front();
		reservation_messages_.pop();

		if (true /*msg->robotname() != robotname_*/) {
			std::vector<fawkes::NavGraphNode> nodes;

			// get nodes out of message
			for (int i = 0; i < (msg->nodelist_size()); i++) {
				nodes.push_back(navgraph->node(msg->nodelist(i)));
			}

			// put nodes pairwise with time into vector
			fawkes::Time now(clock);
			double       max_reservation_duration = (double)max_reservation_duration_;
			fawkes::Time valid_duration           = now + max_reservation_duration;
			std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> timed_nodes;
			for (fawkes::NavGraphNode &node : nodes) {
				timed_nodes.push_back(std::pair<fawkes::NavGraphNode, fawkes::Time>(node, valid_duration));
			}

			std::string constraint_name;

			// reserve node / edge
			if (use_node_constraints_) {
				constraint_name = msg->robotname() + "_Reserved_Nodes";
				reserve_nodes(constraint_name, timed_nodes);
			} else {
				constraint_name = msg->robotname() + "_Reserved_Edges";
				reserve_edges(constraint_name, timed_nodes);
			}
		} else {
			logger->log_warn(name(),
			                 "Message with proper component_id and msg_type, but no conversion. "
			                 " Wrong component ID/message type to C++ type mapping?");
		}
	}

	fawkes::Time now(clock);
	if ((now.in_sec() - time_of_plan_chg.in_sec()) <= repeat_send_duration_) {
		send_msg();
	}
}

std::vector<fawkes::NavGraphNode>
NavgraphBrokerThread::get_nodes_from_string(std::string path)
{
	std::vector<fawkes::NavGraphNode> nodes;
	std::vector<std::string>          string_node_time_list;
	boost::split(string_node_time_list, path, boost::is_any_of(","));

	for (unsigned int i = 0; i < string_node_time_list.size(); i++) {
		nodes.push_back(navgraph->node(string_node_time_list[i]));
	}
	return nodes;
}

std::vector<std::string>
NavgraphBrokerThread::get_path_from_interface_as_vector()
{
	std::vector<std::string> path;
	std::string              vpath[40];

	unsigned int path_length = path_if_->path_length();

	vpath[0]  = path_if_->path_node_1();
	vpath[1]  = path_if_->path_node_2();
	vpath[2]  = path_if_->path_node_3();
	vpath[3]  = path_if_->path_node_4();
	vpath[4]  = path_if_->path_node_5();
	vpath[5]  = path_if_->path_node_6();
	vpath[6]  = path_if_->path_node_7();
	vpath[7]  = path_if_->path_node_8();
	vpath[8]  = path_if_->path_node_9();
	vpath[9]  = path_if_->path_node_10();
	vpath[10] = path_if_->path_node_11();
	vpath[11] = path_if_->path_node_12();
	vpath[12] = path_if_->path_node_13();
	vpath[13] = path_if_->path_node_14();
	vpath[14] = path_if_->path_node_15();
	vpath[15] = path_if_->path_node_16();
	vpath[16] = path_if_->path_node_17();
	vpath[17] = path_if_->path_node_18();
	vpath[18] = path_if_->path_node_19();
	vpath[29] = path_if_->path_node_20();
	vpath[20] = path_if_->path_node_21();
	vpath[21] = path_if_->path_node_22();
	vpath[22] = path_if_->path_node_23();
	vpath[23] = path_if_->path_node_24();
	vpath[24] = path_if_->path_node_25();
	vpath[25] = path_if_->path_node_26();
	vpath[26] = path_if_->path_node_27();
	vpath[27] = path_if_->path_node_28();
	vpath[28] = path_if_->path_node_29();
	vpath[29] = path_if_->path_node_30();
	vpath[30] = path_if_->path_node_31();
	vpath[31] = path_if_->path_node_32();
	vpath[32] = path_if_->path_node_33();
	vpath[33] = path_if_->path_node_34();
	vpath[34] = path_if_->path_node_35();
	vpath[35] = path_if_->path_node_36();
	vpath[36] = path_if_->path_node_37();
	vpath[37] = path_if_->path_node_38();
	vpath[38] = path_if_->path_node_39();
	vpath[39] = path_if_->path_node_40();

	path_.clear();

	for (unsigned int i = 0; i < path_length && i < 39; i++) {
		path.push_back(vpath[i]);
	}

	return path;
}

void
NavgraphBrokerThread::reserve_nodes(
  std::string                                                constraint_name,
  std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> timed_path)
{
	LockPtr<NavGraphConstraintRepo> constraint_repo = navgraph->constraint_repo();

	constraint_repo.lock();

	fawkes::NavGraphTimedReservationListNodeConstraint *timed_node_constraint;

	if (constraint_repo->has_constraint(constraint_name)) {
		timed_node_constraint =
		  (NavGraphTimedReservationListNodeConstraint *)constraint_repo->get_node_constraint(
		    constraint_name);
		timed_node_constraint->clear_nodes();
		timed_node_constraint->add_nodes(timed_path);
	} else {
		timed_node_constraint =
		  new NavGraphTimedReservationListNodeConstraint(logger, constraint_name, clock);
		timed_node_constraint->add_nodes(timed_path);
		constraint_repo->register_constraint((NavGraphNodeConstraint *)timed_node_constraint);
	}

	constraint_repo.unlock();
}

void
NavgraphBrokerThread::reserve_edges(
  std::string                                                constraint_name,
  std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> timed_path)
{
	LockPtr<NavGraphConstraintRepo> constraint_repo = navgraph->constraint_repo();

	constraint_repo.lock();

	fawkes::NavGraphTimedReservationListEdgeConstraint *timed_edge_constraint;
	const std::vector<fawkes::NavGraphEdge> &           graph_edges = navgraph->edges();

	if (constraint_repo->has_constraint(constraint_name)) {
		timed_edge_constraint =
		  (NavGraphTimedReservationListEdgeConstraint *)constraint_repo->get_edge_constraint(
		    constraint_name);
		timed_edge_constraint->clear_edges();

		for (uint16_t i = 1; i < timed_path.size(); i++) {
			for (const NavGraphEdge &gedge : graph_edges) {
				if ((timed_path[i - 1].first.name() == gedge.from()
				     && timed_path[i].first.name() == gedge.to())
				    || (timed_path[i - 1].first.name() == gedge.to()
				        && timed_path[i].first.name() == gedge.from())) {
					timed_edge_constraint->add_edge(gedge, timed_path[i].second);
					break;
				}
			}
		}
	} else {
		timed_edge_constraint =
		  new NavGraphTimedReservationListEdgeConstraint(logger, constraint_name, clock);

		for (uint16_t i = 1; i < timed_path.size(); i++) {
			for (const NavGraphEdge &gedge : graph_edges) {
				if ((timed_path[i - 1].first.name() == gedge.from()
				     && timed_path[i].first.name() == gedge.to())
				    || (timed_path[i - 1].first.name() == gedge.to()
				        && timed_path[i].first.name() == gedge.from())) {
					timed_edge_constraint->add_edge(gedge, timed_path[i].second);
					break;
				}
			}
		}

		constraint_repo->register_constraint(timed_edge_constraint);
	}

	constraint_repo.unlock();
}

void
NavgraphBrokerThread::send_msg()
{
	/*
  std::string txt ="{";
  for(int i=0; i < (m_->nodelist_size()); i++ ){
          txt += m_->nodelist(i);
          txt += ",";
  }
  */

	MutexLocker lock(m_mutex_);
	try {
		gossip_group->broadcast(*m_);
	} catch (std::exception &e) {
		logger->log_warn(name(), "Failed to send message: %s", e.what());
	}
}

void
NavgraphBrokerThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
	path_if_->read();
	std::vector<std::string> path = get_path_from_interface_as_vector();

	fawkes::Time now(clock);
	time_of_plan_chg = now;

	MutexLocker lock(m_mutex_);
	m_->set_sec(now.get_sec());
	m_->set_nsec(now.get_nsec());
	m_->clear_nodelist();

	for (unsigned i = 0; i < path.size(); i++) {
		m_->add_nodelist(path[i].c_str());
	}
}

void
NavgraphBrokerThread::handle_peer_msg(boost::asio::ip::udp::endpoint &           endpoint,
                                      uint16_t                                   component_id,
                                      uint16_t                                   msg_type,
                                      std::shared_ptr<google::protobuf::Message> msg)
{
	MutexLocker lock(loop_mutex);

	if (component_id == navgraph_broker::NavigationMessage::COMP_ID
	    && msg_type == navgraph_broker::NavigationMessage::MSG_TYPE) {
		std::shared_ptr<navgraph_broker::NavigationMessage> tm =
		  std::dynamic_pointer_cast<navgraph_broker::NavigationMessage>(msg);

		int msg_sec  = tm->sec();
		int msg_nsec = tm->nsec();
		if ((msg_sec != last_message_time_sec_) || (msg_nsec != last_message_time_nsec_)) {
			reservation_messages_.push(tm);
			last_message_time_sec_  = tm->sec();
			last_message_time_nsec_ = tm->nsec();

			// print
			std::string txt = "{";
			for (int i = 0; i < (tm->nodelist_size()); i++) {
				txt += tm->nodelist(i);
				txt += ",";
			}
			txt.erase(txt.length() - 1, 1);
			txt += "}";
			logger->log_info(name(),
			                 "Received reservation msg from '%s'time: { sec=%ld nsec=%ld }",
			                 tm->robotname().c_str(),
			                 tm->sec(),
			                 tm->nsec());
		}
	}
}

/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void
NavgraphBrokerThread::handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                                             std::string                     msg)
{
	logger->log_warn(name(),
	                 "Failed to receive peer message from %s:%u: %s",
	                 endpoint.address().to_string().c_str(),
	                 endpoint.port(),
	                 msg.c_str());
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void
NavgraphBrokerThread::handle_peer_send_error(std::string msg)
{
	logger->log_warn(name(), "Failed to send peer message: %s", msg.c_str());
}
