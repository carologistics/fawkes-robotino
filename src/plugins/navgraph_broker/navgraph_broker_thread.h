
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

#include "NavigationMessage.pb.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/NavPathInterface.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/constraints/constraint_repo.h>
#include <navgraph/constraints/timed_reservation_list_edge_constraint.h>
#include <navgraph/constraints/timed_reservation_list_node_constraint.h>
#include <navgraph/navgraph.h>
#include <plugins/gossip/aspect/gossip.h>
#include <plugins/gossip/gossip/gossip_group.h>

#include <queue>
#include <string>
#include <vector>

namespace fawkes {
class Time;
class NavGraph;
class NavGraphConstraintRepo;
class NavPathInterface;
class NavGraphTimedReservationListNodeConstraint;
class NavGraphTimedReservationListEdgeConstraint;
class Mutex;
} // namespace fawkes

class NavgraphBrokerThread : public fawkes::Thread,
                             public fawkes::ClockAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::NavGraphAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::BlackBoardInterfaceListener,
                             public fawkes::GossipAspect
{
public:
	NavgraphBrokerThread();
	virtual ~NavgraphBrokerThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// For BlackBoardInterfaceListener
	virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private: // methods
	void reserve_nodes(std::string                                                constraint_name,
	                   std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> timed_path);
	void reserve_edges(std::string                                                constraint_name,
	                   std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> timed_path);
	void
	                                  add_edges_to_edge_constraint(fawkes::NavGraphTimedReservationListEdgeConstraint *edge_constraint,
	                                                               std::vector<fawkes::NavGraphNode>                   path);
	std::vector<fawkes::NavGraphNode> get_nodes_from_string(std::string path);
	std::vector<std::string>          get_path_from_interface_as_vector();
	void                              send_msg();

	void handle_peer_msg(boost::asio::ip::udp::endpoint &           endpoint,
	                     uint16_t                                   component_id,
	                     uint16_t                                   msg_type,
	                     std::shared_ptr<google::protobuf::Message> msg);
	void handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
	void handle_peer_send_error(std::string msg);

	void log_node_constraint(fawkes::NavGraphTimedReservationListNodeConstraint *node_constraint);
	void log_edge_constraint(fawkes::NavGraphTimedReservationListEdgeConstraint *edge_constraint);

private:
	boost::signals2::connection sig_rcvd_conn_;
	boost::signals2::connection sig_recv_error_conn_;
	boost::signals2::connection sig_send_error_conn_;

private:
	fawkes::NavPathInterface *                                      path_if_;
	std::vector<std::string>                                        path_;
	std::queue<std::shared_ptr<navgraph_broker::NavigationMessage>> reservation_messages_;
	std::string                                                     robotname_;

	navgraph_broker::NavigationMessage *m_;
	fawkes::Mutex *                     m_mutex_;
	int                                 last_message_time_sec_;
	int                                 last_message_time_nsec_;

	fawkes::Time  time_of_plan_chg;
	fawkes::Time *time_;
	double        repeat_send_duration_;
	bool          use_node_constraints_;
	long int      max_reservation_duration_;
};

#endif
