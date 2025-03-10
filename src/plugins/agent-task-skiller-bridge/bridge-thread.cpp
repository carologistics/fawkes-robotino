
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

#include "bridge-thread.h"

#include <core/threading/mutex_locker.h>
#include <llsf_msgs/AgentTask.pb.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/RobotInfo.pb.h>

#include <chrono>

using namespace fawkes;

/** @class AgentTaskSkillerBridgeThread "bridge-thread.h"
 * Bridge between the skiller and agent task messages sent via protobuf.
 * @author Tarik Viehmann
 */

/** Constructor. */
AgentTaskSkillerBridgeThread::AgentTaskSkillerBridgeThread()
: Thread("AgentTaskSkillerBridgeThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("AgentTaskSkillerBridgeThread")
{
}

/** Destructor. */
AgentTaskSkillerBridgeThread::~AgentTaskSkillerBridgeThread()
{
}

void
AgentTaskSkillerBridgeThread::init()
{
	peer_address_      = config->get_string("/agent-task-skiller-bridge/peer_address");
	recv_port_magenta_ = config->get_int("/agent-task-skiller-bridge/recv_port_magenta");
	recv_port_cyan_    = config->get_int("/agent-task-skiller-bridge/recv_port_cyan");
	recv_port_public_  = config->get_int("/agent-task-skiller-bridge/recv_port_public");
	team_name_         = config->get_string("/agent-task-skiller-bridge/team_name");
	crypto_key_        = config->get_string("/agent-task-skiller-bridge/crypto_key");
	robot_id_          = config->get_int("/agent-task-skiller-bridge/robot_id");

	std::vector<std::string> proto_dirs;
	try {
		proto_dirs = config->get_strings("/clips-protobuf/proto-dirs");
		for (size_t i = 0; i < proto_dirs.size(); ++i) {
			std::string::size_type pos;
			if ((pos = proto_dirs[i].find("@BASEDIR@")) != std::string::npos) {
				proto_dirs[i].replace(pos, 9, BASEDIR);
			}
			if ((pos = proto_dirs[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
				proto_dirs[i].replace(pos, 16, FAWKES_BASEDIR);
			}
			if ((pos = proto_dirs[i].find("@RESDIR@")) != std::string::npos) {
				proto_dirs[i].replace(pos, 8, RESDIR);
			}
			if ((pos = proto_dirs[i].find("@CONFDIR@")) != std::string::npos) {
				proto_dirs[i].replace(pos, 9, CONFDIR);
			}
			if (proto_dirs[i][proto_dirs.size() - 1] != '/') {
				proto_dirs[i] += "/";
			}
			//logger->log_warn(name(), "DIR: %s", proto_dirs[i].c_str());
		}
	} catch (...) {
		logger->log_error(name(), "could not load stuff, i break");
		return;
	}

	skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");
	pos_if_     = blackboard->open_for_reading<Position3DInterface>("Pose");
	bbil_add_data_interface(skiller_if_);
	bbil_add_data_interface(pos_if_);
	blackboard->register_listener(this);
	//bbil_add_reader_interface(skiller_if_);

	logger->log_info(name(), "Acquire exclusive Skiller Control");
	SkillerInterface::AcquireControlMessage *aqm = new SkillerInterface::AcquireControlMessage();
	skiller_if_->msgq_enqueue(aqm);

	// logger->log_info(name(),
	//                  "Listening to %s on %i (public) %i (magenta) %i (cyan)",
	//                  peer_address_.c_str(),
	//                  recv_port_public_,
	//                  recv_port_magenta_,
	//                  recv_port_public_);
	message_register_ = std::make_shared<protobuf_comm::MessageRegister>(proto_dirs);
	// public_peer_      = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(peer_address_,
	//                                                                       4477,
	//                                                                       recv_port_public_,
	//                                                                       message_register_.get());
	// public_peer_->signal_received().connect(
	//   boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_msg,
	//               this,
	//               boost::placeholders::_1,
	//               boost::placeholders::_2,
	//               boost::placeholders::_3,
	//               boost::placeholders::_4));
	// public_peer_->signal_recv_error().connect(
	//   boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_recv_error,
	//               this,
	//               boost::placeholders::_1,
	//               boost::placeholders::_2));
	logger->log_info(name(),
	                 "Listening to CYAN peer %s:%i for team %s",
	                 peer_address_.c_str(),
	                 recv_port_cyan_,
	                 team_name_.c_str());
	private_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(peer_address_,
	                                                                       5441,
	                                                                       recv_port_cyan_,
	                                                                       message_register_.get());
	private_peer_->signal_received().connect(
	  boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_msg,
	              this,
	              boost::placeholders::_1,
	              boost::placeholders::_2,
	              boost::placeholders::_3,
	              boost::placeholders::_4));
	private_peer_->signal_recv_error().connect(
	  boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_recv_error,
	              this,
	              boost::placeholders::_1,
	              boost::placeholders::_2));
}

void
AgentTaskSkillerBridgeThread::finalize()
{
	SkillerInterface::ReleaseControlMessage *rcm = new SkillerInterface::ReleaseControlMessage();
	skiller_if_->msgq_enqueue(rcm);
	blackboard->close(skiller_if_);
}

void
AgentTaskSkillerBridgeThread::loop()
{
	std::scoped_lock lock(task_mtx);
	if (next_skill_ != "") {
		// other skill is runnung, cancel it frst:
		if (curr_skill_ != "") {
			logger->log_info(name(),
			                 "Cancel previous skill before accepting new commands: %s",
			                 curr_skill_.c_str());
			SkillerInterface::StopExecMessage *sem = new SkillerInterface::StopExecMessage();
			skiller_if_->msgq_enqueue(sem);
			return; // wait for skill to terminate, send response and then continue

		} else {
			curr_skill_          = next_skill_;
			next_skill_          = "";
			curr_agent_task_msg_ = next_agent_task_msg_;
			SkillerInterface::ExecSkillMessage *sem =
			  new SkillerInterface::ExecSkillMessage(curr_skill_.c_str());
			logger->log_info(name(), "Sending skill: %s", curr_skill_.c_str());
			skiller_if_->msgq_enqueue(sem);
			return;
		}
	}
	if (!running_) {
		send_response();
		error_code_ = 0;
		curr_skill_ = "";
	}
}

void
AgentTaskSkillerBridgeThread::handle_peer_msg(boost::asio::ip::udp::endpoint &,
                                              uint16_t,
                                              uint16_t,
                                              std::shared_ptr<google::protobuf::Message> msg_ptr)
{
	if (!msg_ptr) {
		logger->log_debug(name(), "Received invalid msg ptr");
		return;
	}
	const google::protobuf::Descriptor *desc = msg_ptr->GetDescriptor();
	logger->log_debug(name(), "Received msg %s", desc->name().c_str());
	if (desc->name() == "GameState") {
		const llsf_msgs::GameState *game_state_msg =
		  dynamic_cast<const llsf_msgs::GameState *>(msg_ptr.get());
		if (!private_peer_) {
			if (game_state_msg->team_cyan() == team_name_) {
				logger->log_info(name(),
				                 "Listening to CYAN peer %s:%i for team %s",
				                 peer_address_.c_str(),
				                 recv_port_cyan_,
				                 team_name_.c_str());
				private_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
				  peer_address_, 5441, recv_port_cyan_, message_register_.get(), crypto_key_);
				private_peer_->signal_received().connect(
				  boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_msg,
				              this,
				              boost::placeholders::_1,
				              boost::placeholders::_2,
				              boost::placeholders::_3,
				              boost::placeholders::_4));
				private_peer_->signal_recv_error().connect(
				  boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_recv_error,
				              this,
				              boost::placeholders::_1,
				              boost::placeholders::_2));
				team_color_ = "CYAN";
			} else if (game_state_msg->team_magenta() == team_name_) {
				logger->log_info(name(),
				                 "Listening to MAGENTA peer %s:%i for team %s",
				                 peer_address_.c_str(),
				                 recv_port_magenta_,
				                 team_name_.c_str());
				private_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
				  peer_address_, 5441, recv_port_magenta_, message_register_.get(), crypto_key_);
				private_peer_->signal_received().connect(
				  boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_msg,
				              this,
				              boost::placeholders::_1,
				              boost::placeholders::_2,
				              boost::placeholders::_3,
				              boost::placeholders::_4));
				private_peer_->signal_recv_error().connect(
				  boost::bind(&AgentTaskSkillerBridgeThread::handle_peer_recv_error,
				              this,
				              boost::placeholders::_1,
				              boost::placeholders::_2));
				team_color_ = "MAGENTA";
			} else {
				logger->log_info(name(),
				                 "Waiting for RefBox to send information on team %s",
				                 team_name_.c_str());
			}
		}
	} else if (desc->name() == "AgentTask") {
		std::scoped_lock            lock(task_mtx);
		const llsf_msgs::AgentTask *agent_task_msg =
		  dynamic_cast<const llsf_msgs::AgentTask *>(msg_ptr.get());
		if (agent_task_msg->robot_id() == robot_id_) {
			next_skill_          = construct_task_string(*agent_task_msg);
			next_agent_task_msg_ = *agent_task_msg;
			if (next_agent_task_msg_.task_id() != curr_agent_task_msg_.task_id()) {
				logger->log_info(name(), "Got new task id %i", next_agent_task_msg_.task_id());
				// If a previous thread is running, stop it
				if (response_thread_.joinable()) {
					stop_thread_ = true;
					response_thread_.join();
					stop_thread_ = false;
				}
				terminated_ = false;
				wakeup();
			} else {
				next_skill_ = "";
				logger->log_info(name(),
				                 "Ignoring same task id %i, currently at %i",
				                 next_agent_task_msg_.task_id(),
				                 curr_agent_task_msg_.task_id());
			}
		}
	} else {
		logger->log_debug(name(), "Received %s", desc->name().c_str());
	}
}

std::string
AgentTaskSkillerBridgeThread::construct_task_string(const llsf_msgs::AgentTask &agent_task_msg)
{
	if (agent_task_msg.has_move()) {
		const llsf_msgs::Move &move     = agent_task_msg.move();
		std::string            waypoint = move.waypoint();
		std::string machine_point       = move.has_machine_point() ? move.machine_point() : "INPUT";
		if (machine_point.find("SHELF") != std::string::npos) {
			machine_point = "INPUT";
		}
		if (machine_point.find("SLIDE") != std::string::npos) {
			machine_point = "INPUT";
		}
		return "moveto{place=\"" + waypoint + "-" + machine_point + "\"}";
	} else if (agent_task_msg.has_retrieve()) {
		const llsf_msgs::Retrieve &retrieve      = agent_task_msg.retrieve();
		std::string                machine_point = retrieve.machine_point();
		if (machine_point.find("SHELF") != std::string::npos) {
			machine_point += shelf_slots[shelf_index];
			shelf_index = (shelf_index + 1) % 3;
		}
		return "manipulate_wp{mps=\"" + retrieve.machine_id() + "\",safe_put=false,map_pos=true,side=\""
		       + machine_point + "\",target=\"WORKPIECE\",c=\"C3\"}";
	} else if (agent_task_msg.has_deliver()) {
		const llsf_msgs::Deliver &deliver = agent_task_msg.deliver();
		if (deliver.machine_point() == "SLIDE") {
			return "manipulate_wp{mps=\"" + deliver.machine_id() + "\",map_pos=true,side=\""
			       + deliver.machine_point() + "\",target=\"SLIDE\",c=\"C3\"}";
		}
		return "manipulate_wp{mps=\"" + deliver.machine_id() + "\",map_pos=true,side=\""
		       + deliver.machine_point() + "\",target=\"CONVEYOR\",c=\"C3\"}";
	} else if (agent_task_msg.has_buffer()) {
		return "unknown";
	} else if (agent_task_msg.has_explore_machine()) {
		return "unknown";
	} else {
		return "unknown";
	}
}
void
AgentTaskSkillerBridgeThread::handle_peer_recv_error(boost::asio::ip::udp::endpoint &, std::string)
{
	curr_agent_task_msg_.set_task_id(-1);
}

void
AgentTaskSkillerBridgeThread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	SkillerInterface *skiller_if = dynamic_cast<SkillerInterface *>(interface);
	if (skiller_if) {
		skiller_if->read();
		if (skiller_if->serial().get_string() != std::string(skiller_if->exclusive_controller())) {
			successful_ = false;
			terminated_ = true;
			error_code_ = 0; // Skiller control lost
			logger->log_info(name(), "Skill control lost, wakeup");
			wakeup();
			return;
		}
		switch (skiller_if->status()) {
		case fawkes::SkillerInterface::SkillStatusEnum::S_INACTIVE: running_ = false; break;
		case fawkes::SkillerInterface::SkillStatusEnum::S_FINAL:
			successful_ = true;
			terminated_ = true;
			running_    = false;
			logger->log_info(name(), "Final, wakeup");
			wakeup();
			break;
		case fawkes::SkillerInterface::SkillStatusEnum::S_RUNNING: running_ = true; break;
		case fawkes::SkillerInterface::SkillStatusEnum::S_FAILED:
			successful_ = false;
			terminated_ = true;
			running_    = false;
			logger->log_info(name(), "Failed, wakeup");
			wakeup();
			break;
		}
	}
	Position3DInterface *pos_if = dynamic_cast<Position3DInterface *>(interface);
	if (pos_if) {
		pos_if->read();
		rotation_    = pos_if->rotation();
		translation_ = pos_if->translation();
		send_pose();
	}
}
void
AgentTaskSkillerBridgeThread::send_pose()
{
	if (private_peer_) {
		// Create and populate the outer message (Robot)
		llsf_msgs::Robot pose;
		pose.set_name(std::to_string(robot_id_));
		pose.set_team(team_name_);
		pose.set_host("robot");
		auto now     = std::chrono::system_clock::now();
		auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		auto nanoseconds =
		  std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()
		  % 1000000000;

		llsf_msgs::Time *robot_time = new llsf_msgs::Time();
		robot_time->set_sec(static_cast<int32_t>(seconds));
		robot_time->set_nsec(static_cast<int32_t>(nanoseconds));
		pose.set_allocated_last_seen(robot_time);

		if (team_color_ == "CYAN") {
			pose.set_team_color(llsf_msgs::CYAN);
		} else {
			pose.set_team_color(llsf_msgs::MAGENTA);
		}

		pose.set_number(robot_id_);

		llsf_msgs::Time *pose_time = new llsf_msgs::Time();
		pose_time->set_sec(static_cast<int32_t>(seconds));
		pose_time->set_nsec(static_cast<int32_t>(nanoseconds));

		llsf_msgs::Pose2D *pose_info = new llsf_msgs::Pose2D();
		pose_info->set_allocated_timestamp(pose_time);
		pose_info->set_x(static_cast<float>(translation_[0]));
		pose_info->set_y(static_cast<float>(translation_[1]));
		pose_info->set_ori(0);

		pose.set_allocated_pose(pose_info);

		// Send the message using the peer
		private_peer_->send(pose);
	}
}

void
AgentTaskSkillerBridgeThread::send_response()
{
	// If a previous thread is running, stop it
	if (response_thread_.joinable()) {
		stop_thread_ = true;
		response_thread_.join();
		stop_thread_ = false;
	}

	// Start a new thread with the updated data
	response_thread_ = std::thread([this]() {
		llsf_msgs::AgentTask response = curr_agent_task_msg_;
		response.set_cancel_task(!successful_);
		response.set_pause_task(false);
		if (terminated_) {
			response.set_successful(successful_);
		}
		response.set_canceled(false);
		response.set_error_code(error_code_);
		while (!stop_thread_) {
			if (terminated_) {
				logger->log_info(name(),
				                 "Send response: Succesful %s (code %i)",
				                 response.successful() ? "true" : "false",
				                 error_code_);
			}
			private_peer_->send(response);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
	});
}
