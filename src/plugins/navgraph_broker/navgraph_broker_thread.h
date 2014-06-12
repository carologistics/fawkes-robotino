
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
#include <aspect/clock.h>

#include <plugins/gossip/aspect/gossip.h>
#include <plugins/gossip/gossip/gossip_group.h>

#include <plugins/navgraph/aspect/navgraph.h>
#include <plugins/navgraph/constraints/constraint_repo.h>
#include <utils/graph/topological_map_graph.h>

#include <interfaces/NavPathInterface.h>
#include <blackboard/interface_listener.h>

#include <string>
#include <vector>

namespace fawkes{
	class Time;
	class TopologicalMapGraph;
	class ConstraintRepo;
	class NavPathInterface;
}

class NavgraphBrokerThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::NavGraphAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
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
 protected: virtual void run() { Thread::run(); }

 private: // members
 	 std::string robot_name_;
     fawkes::NavPathInterface *path_if_;

     fawkes::Time *last_sent_;
     unsigned int  counter_;

 private: // methods
     void reserve_nodes(std::string robot_name, std::vector<fawkes::TopologicalMapNode> path);
     std::vector<fawkes::TopologicalMapNode> get_nodes_from_string(std::string path);
     std::string get_string_from_nodes(std::vector<fawkes::TopologicalMapNode> path);
     std::string get_path_from_interface_as_string();
     void send_data();


 private:
   void handle_peer_msg(boost::asio::ip::udp::endpoint &endpoint,
 		       uint16_t component_id, uint16_t msg_type,
 		       std::shared_ptr<google::protobuf::Message> msg);
   void handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
   void handle_peer_send_error(std::string msg);

  private:
   boost::signals2::connection sig_rcvd_conn_;
   boost::signals2::connection sig_recv_error_conn_;
   boost::signals2::connection sig_send_error_conn_;

  private:
   std::vector<std::string> path_;
   std::string robotname_;

};

#endif
