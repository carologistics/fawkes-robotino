
/***************************************************************************
 *  llsfrbcomm_plugin.cpp - Plugin is a adapter between
 *       Protobuf Refbox communication and
 *       Protobuf communication over the gazebo node
 *
 *  Created: Wed Aug 21 15:18:27 2013
 *  Copyright  2013  Frederik Zwilling
 *                   Tim Niemueller [www.niemueller.de]
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

#include "gazsim_llsfrbcomm_thread.h"

#include <aspect/blocked_timing.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/PuckInfo.pb.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/message_register.h>

using namespace fawkes;
using namespace protobuf_comm;

/** @class GazsimLLSFRbCommThread "clips_thread.h"
 * Plugin is a adapter between
 *       Protobuf Refbox communication and
 *       Protobuf communication over the gazebo node
 *
 * @author Frederik Zwilling, Tim Niemueller
 */

/** Constructor. */
GazsimLLSFRbCommThread::GazsimLLSFRbCommThread()
: Thread("GazsimLLSFRbCommThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

/** Destructor. */
GazsimLLSFRbCommThread::~GazsimLLSFRbCommThread()
{
}

void
GazsimLLSFRbCommThread::init()
{
	// logger->log_info(name(), "GazsimLLSFRbComm initialized");

	// read config values
	refbox_host_ = config->get_string("/gazsim/llsf-rb-comm/refbox-host");
	refbox_port_ = config->get_uint("/gazsim/llsf-rb-comm/refbox-port");
	proto_dirs_  = config->get_strings("/gazsim/proto-dirs");
	// resolve proto paths
	try {
		proto_dirs_ = config->get_strings("/clips-protobuf/proto-dirs");
		for (size_t i = 0; i < proto_dirs_.size(); ++i) {
			std::string::size_type pos;
			if ((pos = proto_dirs_[i].find("@BASEDIR@")) != std::string::npos) {
				proto_dirs_[i].replace(pos, 9, BASEDIR);
			}
			if ((pos = proto_dirs_[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
				proto_dirs_[i].replace(pos, 16, FAWKES_BASEDIR);
			}
			if ((pos = proto_dirs_[i].find("@RESDIR@")) != std::string::npos) {
				proto_dirs_[i].replace(pos, 8, RESDIR);
			}
			if ((pos = proto_dirs_[i].find("@CONFDIR@")) != std::string::npos) {
				proto_dirs_[i].replace(pos, 9, CONFDIR);
			}
			if (proto_dirs_[i][proto_dirs_.size() - 1] != '/') {
				proto_dirs_[i] += "/";
			}
		}
	} catch (Exception &e) {
		logger->log_warn(name(), "Failed to load proto paths from config, exception follows");
		logger->log_warn(name(), e);
	}

	// prepare client
	create_client();
	client_->async_connect(refbox_host_.c_str(), refbox_port_);
	// this invokes the connect in the loop
	disconnected_recently_ = true;

	// create publisher and subscriber for connection with gazebo node
	machine_info_pub_ = gazebo_world_node->Advertise<llsf_msgs::MachineInfo>(
	  config->get_string("/gazsim/topics/machine-info"));
	game_state_pub_ = gazebo_world_node->Advertise<llsf_msgs::GameState>(
	  config->get_string("/gazsim/topics/game-state"));
	puck_info_pub_ = gazebo_world_node->Advertise<llsf_msgs::PuckInfo>(
	  config->get_string("/gazsim/topics/puck-info"));
	time_sync_sub_ = gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/time"),
	                                              &GazsimLLSFRbCommThread::on_time_sync_msg,
	                                              this);
	set_game_state_sub_ =
	  gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/set-game-state"),
	                               &GazsimLLSFRbCommThread::on_set_game_state_msg,
	                               this);
	set_game_phase_sub_ =
	  gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/set-game-phase"),
	                               &GazsimLLSFRbCommThread::on_set_game_phase_msg,
	                               this);
	set_team_name_sub_ =
	  gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/set-team-name"),
	                               &GazsimLLSFRbCommThread::on_set_team_name_msg,
	                               this);
}

void
GazsimLLSFRbCommThread::finalize()
{
	delete message_register_;
	delete client_;
}

void
GazsimLLSFRbCommThread::loop()
{
	/*if(disconnected_recently_)
  {
    disconnected_recently_ = false;
    //connect
    logger->log_info(name(), "Dong Try");
    client_->async_connect(refbox_host_.c_str(), refbox_port_);
    }*/
}

/** Handler for successful connection to the client
 */
void
GazsimLLSFRbCommThread::client_connected()
{
	logger->log_info(name(), "Connected to Refbox");
}

/** Handler for loss of connection to client
 * @param error boost error code
 */
void
GazsimLLSFRbCommThread::client_disconnected(const boost::system::error_code &error)
{
	logger->log_info(name(), "Disconnected");
	create_client();
}

/** Handler for incoming msg from client
 * @param comp_id component id of protobuf msg
 * @param msg_type type of protobuf msg
 * @param msg pointer to protobuf msg
 */
void
GazsimLLSFRbCommThread::client_msg(uint16_t                                   comp_id,
                                   uint16_t                                   msg_type,
                                   std::shared_ptr<google::protobuf::Message> msg)
{
	// logger->log_info(name(), "Message");
	// logger->log_info(name(), msg->GetTypeName().c_str());

	// Filter wanted messages
	if (msg->GetTypeName() == "llsf_msgs.MachineInfo") {
		// logger->log_info(name(), "Sending MachineInfo to gazebo");
		machine_info_pub_->Publish(*msg);
		return;
	}

	if (msg->GetTypeName() == "llsf_msgs.GameState") {
		// logger->log_info(name(), "Sending GameState to gazebo");
		game_state_pub_->Publish(*msg);
		return;
	}

	if (msg->GetTypeName() == "llsf_msgs.PuckInfo") {
		// logger->log_info(name(), "Sending PuckInfo to gazebo");
		puck_info_pub_->Publish(*msg);
		return;
	}
}

void
GazsimLLSFRbCommThread::create_client()
{
	// create message register with all messages to listen for
	message_register_ = new protobuf_comm::MessageRegister(proto_dirs_);

	// create client and register handlers
	client_ = new ProtobufStreamClient(message_register_);
	client_->signal_connected().connect(boost::bind(&GazsimLLSFRbCommThread::client_connected, this));
	client_->signal_disconnected().connect(boost::bind(&GazsimLLSFRbCommThread::client_disconnected,
	                                                   this,
	                                                   boost::asio::placeholders::error));
	client_->signal_received().connect(
	  boost::bind(&GazsimLLSFRbCommThread::client_msg, this, _1, _2, _3));

	// this invokes the connect in the loop
	disconnected_recently_ = true;
}

void
GazsimLLSFRbCommThread::on_time_sync_msg(ConstSimTimePtr &msg)
{
	// logger->log_info(name(), "Sending Simulation Time");

	// provide time source with newest message
	if (!client_->connected()) {
		return;
	}
	// fill msg for refbox with info from gazsim_msg
	llsf_msgs::SimTimeSync to_rb;
	llsf_msgs::Time *      time = to_rb.mutable_sim_time();
	time->set_sec(msg->sim_time_sec());
	time->set_nsec(msg->sim_time_nsec());
	to_rb.set_real_time_factor(msg->real_time_factor());
	to_rb.set_paused(msg->paused());

	// send it and make refbox able to handle the msg
	client_->send(to_rb);
}

void
GazsimLLSFRbCommThread::on_set_game_state_msg(ConstSetGameStatePtr &msg)
{
	// logger->log_info(name(), "Sending SetGameState to refbox");
	if (!client_->connected()) {
		return;
	}
	llsf_msgs::SetGameState to_rb = *msg;
	client_->send(to_rb);
}

void
GazsimLLSFRbCommThread::on_set_game_phase_msg(ConstSetGamePhasePtr &msg)
{
	// logger->log_info(name(), "Sending SetGamePhase to refbox");
	if (!client_->connected()) {
		return;
	}
	llsf_msgs::SetGamePhase to_rb = *msg;
	client_->send(to_rb);
}

void
GazsimLLSFRbCommThread::on_set_team_name_msg(ConstSetTeamNamePtr &msg)
{
	// logger->log_info(name(), "Sending SetTeamName to refbox");
	if (!client_->connected()) {
		return;
	}
	llsf_msgs::SetTeamName to_rb = *msg;
	client_->send(to_rb);
}
