
/***************************************************************************
 * Protoboard plugin template instantiation for the LLSF
 * - Mappings for sending ProtoBuf messages via blackboard interfaces
 *
 * Copyright 2019 Victor Mataré
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

#include <interfaces/PrepareMachineInterface.h>
#include <interfaces/SendBeaconInterface.h>
#include <libs/llsf_msgs/BeaconSignal.pb.h>
#include <libs/llsf_msgs/MachineInstructions.pb.h>
#include <libs/protoboard/blackboard_manager.h>

namespace protoboard {

using namespace fawkes;
using namespace std;

/****************************************
 *          SendBeaconMessage           *
 ****************************************/

template <>
std::string
iface_id_for_type<SendBeaconInterface>()
{
	return "/protoboard/send_beacon";
}

static const enum_map<llsf_msgs::Team, SendBeaconInterface::TEAM_COLOR> team_enum_beacon{
  {llsf_msgs::CYAN, SendBeaconInterface::TEAM_COLOR::CYAN},
  {llsf_msgs::MAGENTA, SendBeaconInterface::TEAM_COLOR::MAGENTA}};

/** Send a ProtoBuf BeaconSignal
 * @param iface The SendBeaconInterface
 * @param msg The SendBeaconMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(SendBeaconInterface *                   iface,
                                  SendBeaconInterface::SendBeaconMessage *msg)
{
	std::shared_ptr<llsf_msgs::BeaconSignal> m = std::make_shared<llsf_msgs::BeaconSignal>();

	// Set time here to simplify blackboard interface
	llsf_msgs::Time *time = new llsf_msgs::Time();
	time->set_sec(clock->now().get_sec());
	time->set_nsec(clock->now().get_nsec());

	llsf_msgs::Pose2D *pose = new llsf_msgs::Pose2D();
	pose->set_x(msg->translation(0));
	pose->set_y(msg->translation(1));
	pose->set_ori(msg->orientation());
	pose->set_allocated_timestamp(time);

	m->set_allocated_pose(pose);
	m->set_allocated_team_name(new std::string(msg->team_name()));
	m->set_number(msg->number());
	m->set_allocated_peer_name(new std::string(msg->peer_name()));
	m->set_team_color(team_enum_beacon.of(msg->team_color()));

	m->set_allocated_time(new llsf_msgs::Time(*time));

	// Same for sequence number, good idea to count it centrally, anyways
	static google::protobuf::uint64 beacon_seq = 0;
	m->set_seq(beacon_seq++);

	message_handler_->send(iface->peer_id(), m);
}

/** Set the ProtoBuf peer on the SendBeaconInterface
 * @param iface The SendBeaconInterface
 * @param msg The SetPeerMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(SendBeaconInterface *                iface,
                                  SendBeaconInterface::SetPeerMessage *msg)
{
	iface->set_peer_id(msg->peer_id());
}

/****************************************
 *           Prepare Messages           *
 ****************************************/

template <>
std::string
iface_id_for_type<PrepareMachineInterface>()
{
	return "/protoboard/prepare_machine";
}

static const enum_map<llsf_msgs::Team, PrepareMachineInterface::Team> team_enum_prepare{
  {llsf_msgs::CYAN, PrepareMachineInterface::Team::CYAN},
  {llsf_msgs::MAGENTA, PrepareMachineInterface::Team::MAGENTA}};

static const enum_map<llsf_msgs::BaseColor, PrepareMachineInterface::BaseColor> base_color_enum{
  {llsf_msgs::BaseColor::BASE_BLACK, PrepareMachineInterface::BASE_BLACK},
  {llsf_msgs::BaseColor::BASE_RED, PrepareMachineInterface::BASE_RED},
  {llsf_msgs::BaseColor::BASE_SILVER, PrepareMachineInterface::BASE_SILVER}};

static const enum_map<llsf_msgs::MachineSide, PrepareMachineInterface::MachineSide>
  machine_side_enum{{llsf_msgs::MachineSide::INPUT, PrepareMachineInterface::MachineSide::INPUT},
                    {llsf_msgs::MachineSide::OUTPUT, PrepareMachineInterface::MachineSide::OUTPUT}};

static const enum_map<llsf_msgs::CSOp, PrepareMachineInterface::CSOp> csop_enum{
  {llsf_msgs::CSOp::MOUNT_CAP, PrepareMachineInterface::CSOp::MOUNT_CAP},
  {llsf_msgs::CSOp::RETRIEVE_CAP, PrepareMachineInterface::CSOp::RETRIEVE_CAP}};

static const enum_map<llsf_msgs::RingColor, PrepareMachineInterface::RingColor>
  prepare_ring_color_enum{
    {llsf_msgs::RingColor::RING_BLUE, PrepareMachineInterface::RingColor::RING_BLUE},
    {llsf_msgs::RingColor::RING_GREEN, PrepareMachineInterface::RingColor::RING_GREEN},
    {llsf_msgs::RingColor::RING_ORANGE, PrepareMachineInterface::RingColor::RING_ORANGE},
    {llsf_msgs::RingColor::RING_YELLOW, PrepareMachineInterface::RingColor::RING_YELLOW},
  };

static const enum_map<llsf_msgs::SSOp, PrepareMachineInterface::SSOp> ssop_enum{
  {llsf_msgs::SSOp::STORE, PrepareMachineInterface::SSOp::STORE},
  {llsf_msgs::SSOp::RETRIEVE, PrepareMachineInterface::SSOp::RETRIEVE},
};

/** Send a ProtoBuf PrepareInstructionBS
 * @param iface The PrepareMachineInterface
 * @param msg The PrepareBSMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(PrepareMachineInterface *                  iface,
                                  PrepareMachineInterface::PrepareBSMessage *msg)
{
	std::shared_ptr<llsf_msgs::PrepareMachine> m     = std::make_shared<llsf_msgs::PrepareMachine>();
	llsf_msgs::PrepareInstructionBS *          instr = new llsf_msgs::PrepareInstructionBS();
	instr->set_color(base_color_enum.of(msg->color()));
	instr->set_side(machine_side_enum.of(msg->side()));
	m->set_allocated_instruction_bs(instr);
	m->set_team_color(team_enum_prepare.of(msg->team_color()));
	m->set_machine(msg->machine());

	message_handler_->send(iface->peer_id(), m);
}

/** Send a ProtoBuf PrepareInstructionCS
 * @param iface The PrepareMachineInterface
 * @param msg The PrepareCSMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(PrepareMachineInterface *                  iface,
                                  PrepareMachineInterface::PrepareCSMessage *msg)
{
	std::shared_ptr<llsf_msgs::PrepareMachine> m     = std::make_shared<llsf_msgs::PrepareMachine>();
	llsf_msgs::PrepareInstructionCS *          instr = new llsf_msgs::PrepareInstructionCS();
	instr->set_operation(csop_enum.of(msg->operation()));
	m->set_allocated_instruction_cs(instr);
	m->set_team_color(team_enum_prepare.of(msg->team_color()));
	m->set_machine(msg->machine());

	message_handler_->send(iface->peer_id(), m);
}

/** Send a ProtoBuf PrepareInstructionCS
 * @param iface The PrepareMachineInterface
 * @param msg The PrepareCSMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(PrepareMachineInterface *                  iface,
                                  PrepareMachineInterface::PrepareDSMessage *msg)
{
	std::shared_ptr<llsf_msgs::PrepareMachine> m     = std::make_shared<llsf_msgs::PrepareMachine>();
	llsf_msgs::PrepareInstructionDS *          instr = new llsf_msgs::PrepareInstructionDS();
	instr->set_order_id(msg->order_id());
	m->set_allocated_instruction_ds(instr);
	m->set_team_color(team_enum_prepare.of(msg->team_color()));
	m->set_machine(msg->machine());

	message_handler_->send(iface->peer_id(), m);
}

/** Send a ProtoBuf PrepareInstructionRS
 * @param iface The PrepareMachineInterface
 * @param msg The PrepareRSMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(PrepareMachineInterface *                  iface,
                                  PrepareMachineInterface::PrepareRSMessage *msg)
{
	std::shared_ptr<llsf_msgs::PrepareMachine> m     = std::make_shared<llsf_msgs::PrepareMachine>();
	llsf_msgs::PrepareInstructionRS *          instr = new llsf_msgs::PrepareInstructionRS();
	instr->set_ring_color(prepare_ring_color_enum.of(msg->ring_color()));
	m->set_allocated_instruction_rs(instr);
	m->set_team_color(team_enum_prepare.of(msg->team_color()));
	m->set_machine(msg->machine());

	message_handler_->send(iface->peer_id(), m);
}

/** Send a ProtoBuf PrepareInstructionSS (not implemented yet)
 * @param iface The PrepareMachineInterface
 * @param msg The PrepareSSMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(PrepareMachineInterface *                  iface,
                                  PrepareMachineInterface::PrepareSSMessage *msg)
{
	logger->log_error(name(), "Storage Station is not implemented yet");
}

/** Set the ProtoBuf peer on the PrepareMachineInterface
 * @param iface The PrepareMachineInterface
 * @param msg The SetPeerMessage that came in on the interface */
template <>
void
BlackboardManager::handle_message(PrepareMachineInterface *                iface,
                                  PrepareMachineInterface::SetPeerMessage *msg)
{
	iface->set_peer_id(msg->peer_id());
}

} // namespace protoboard
