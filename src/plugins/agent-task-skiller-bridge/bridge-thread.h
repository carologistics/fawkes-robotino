
/***************************************************************************
 *  bridge-plugin.cpp - bridge between skiller and agent task proto msgs
 *
 *  Created: Mon May 20 2024
 *  Copyright  2024  Tarik Viehmann
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

#ifndef __PLGUINS_AGENT_TASK_SKILLER_BRIDGE_THREAD_H_
#define __PLGUINS_AGENT_TASK_SKILLER_BRIDGE_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <google/protobuf/message.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SkillerInterface.h>
#include <llsf_msgs/AgentTask.pb.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/peer.h>

#include <boost/asio.hpp>
#include <string>
#include <vector>

namespace fawkes {

class AgentTaskSkillerBridgeThread : public fawkes::Thread,
                                     public fawkes::LoggingAspect,
                                     public fawkes::ConfigurableAspect,
                                     public fawkes::BlackBoardAspect,
                                     public fawkes::BlackBoardInterfaceListener
{
public:
	AgentTaskSkillerBridgeThread();
	virtual ~AgentTaskSkillerBridgeThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for BlackBoardInterfaceListener
	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) noexcept;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::string                  cfg_skiller_iface_id_;
	fawkes::SkillerInterface    *skiller_if_;
	fawkes::Position3DInterface *pos_if_;

	std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> private_peer_;
	std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> public_peer_;
	std::shared_ptr<protobuf_comm::MessageRegister>       message_register_;

	void        handle_peer_msg(boost::asio::ip::udp::endpoint            &endpoint,
	                            uint16_t                                   component_id,
	                            uint16_t                                   msg_type,
	                            std::shared_ptr<google::protobuf::Message> msg);
	void        handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
	void        send_response();
	void        send_pose();
	std::string construct_task_string(const llsf_msgs::AgentTask &agent_task_msg);

	std::string team_name_;
	std::string team_color_;
	std::string crypto_key_;
	int         robot_id_;

	std::string next_skill_;
	std::string curr_skill_;
	bool        running_     = false;
	bool        successful_  = false;
	bool        stop_thread_ = false;
	int         error_code_  = 0;

	std::thread response_thread_;

	llsf_msgs::AgentTask next_agent_task_msg_;
	llsf_msgs::AgentTask curr_agent_task_msg_;

	unsigned short recv_port_magenta_;
	unsigned short recv_port_cyan_;
	unsigned short recv_port_public_;
	std::string    peer_address_;

	double *rotation_;
	double *translation_;

	std::mutex task_mtx;
};
} // namespace fawkes

#endif
