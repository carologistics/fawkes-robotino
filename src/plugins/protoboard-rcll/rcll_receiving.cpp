
/***************************************************************************
 * Protoboard plugin template instantiation for the LLSF
 * - Mappings for received ProtoBuf messages to blackboard interfaces
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

#include <interfaces/MachineInfoInterface.h>
#include <interfaces/OrderInterface.h>
#include <interfaces/RCLLGameStateInterface.h>
#include <interfaces/RecvBeaconInterface.h>
#include <interfaces/RingInfoInterface.h>
#include <libs/llsf_msgs/BeaconSignal.pb.h>
#include <libs/llsf_msgs/GameState.pb.h>
#include <libs/llsf_msgs/MachineInfo.pb.h>
#include <libs/llsf_msgs/OrderInfo.pb.h>
#include <libs/llsf_msgs/RingInfo.pb.h>
#include <libs/llsf_msgs/RobotInfo.pb.h>
#include <libs/llsf_msgs/VersionInfo.pb.h>
#include <protoboard/protobuf_to_bb.h>

namespace protoboard {

using namespace fawkes;
using namespace std;

std::unordered_map<std::string, std::shared_ptr<pb_convert>>
make_receiving_interfaces_map()
{
	return {
	  {"llsf_msgs.OrderInfo",
	   make_shared<pb_sequence_converter<llsf_msgs::OrderInfo,
	                                     pb_converter<llsf_msgs::Order, OrderInterface>>>()},
	  {"llsf_msgs.BeaconSignal",
	   make_shared<pb_converter<llsf_msgs::BeaconSignal, RecvBeaconInterface>>()},
	  {"llsf_msgs.GameState",
	   make_shared<pb_converter<llsf_msgs::GameState, RCLLGameStateInterface>>()},
	  {"llsf_msgs.MachineInfo",
	   make_shared<pb_sequence_converter<llsf_msgs::MachineInfo,
	                                     pb_converter<llsf_msgs::Machine, MachineInfoInterface>>>()},
	  {"llsf_msgs.RingInfo", make_shared<pb_converter<llsf_msgs::RingInfo, RingInfoInterface>>()},

	  // Dummy handler, i.e. discard message
	  {"llsf_msgs.RobotInfo", make_shared<pb_convert>()},
	  {"llsf_msgs.VersionInfo", make_shared<pb_convert>()},
	};
};

/****************************************
 *           OrderInterface             *
 ****************************************/

template <>
std::string
iface_id_for_type<OrderInterface>()
{
	return "/protoboard/order";
}

static const enum_map<llsf_msgs::BaseColor, OrderInterface::BaseColor> base_color_enum_order{
  {llsf_msgs::BaseColor::BASE_BLACK, OrderInterface::BASE_BLACK},
  {llsf_msgs::BaseColor::BASE_RED, OrderInterface::BASE_RED},
  {llsf_msgs::BaseColor::BASE_SILVER, OrderInterface::BASE_SILVER}};

static const enum_map<llsf_msgs::RingColor, OrderInterface::RingColor> ring_color_enum_order{
  {llsf_msgs::RingColor::RING_BLUE, OrderInterface::RING_BLUE},
  {llsf_msgs::RingColor::RING_GREEN, OrderInterface::RING_GREEN},
  {llsf_msgs::RingColor::RING_ORANGE, OrderInterface::RING_ORANGE},
  {llsf_msgs::RingColor::RING_YELLOW, OrderInterface::RING_YELLOW},
};

static const enum_map<llsf_msgs::CapColor, OrderInterface::CapColor> cap_color_enum_order{
  {llsf_msgs::CapColor::CAP_BLACK, OrderInterface::CAP_BLACK},
  {llsf_msgs::CapColor::CAP_GREY, OrderInterface::CAP_GREY},
};

static const enum_map<llsf_msgs::Order::Complexity, OrderInterface::Complexity>
  complexity_enum_order{{llsf_msgs::Order::Complexity::Order_Complexity_C0, OrderInterface::C0},
                        {llsf_msgs::Order::Complexity::Order_Complexity_C1, OrderInterface::C1},
                        {llsf_msgs::Order::Complexity::Order_Complexity_C2, OrderInterface::C2},
                        {llsf_msgs::Order::Complexity::Order_Complexity_C3, OrderInterface::C3}};

template <>
const google::protobuf::RepeatedPtrField<llsf_msgs::Order> &
pb_sequence_converter<llsf_msgs::OrderInfo, pb_converter<llsf_msgs::Order, OrderInterface>>::
  extract_sequence(const llsf_msgs::OrderInfo &msg)
{
	return msg.orders();
}

template <>
std::string
pb_converter<llsf_msgs::Order, OrderInterface>::get_sequence_id(const llsf_msgs::Order &msg)
{
	return std::to_string(msg.id());
}

template <>
void
pb_converter<llsf_msgs::Order, OrderInterface>::handle(const llsf_msgs::Order &msg,
                                                       OrderInterface *        iface)
{
	if (iface->base_color() != base_color_enum_order.of(msg.base_color()))
		iface->set_base_color(base_color_enum_order.of(msg.base_color()));
	if (iface->cap_color() != cap_color_enum_order.of(msg.cap_color()))
		iface->set_cap_color(cap_color_enum_order.of(msg.cap_color()));
	if (iface->complexity() != complexity_enum_order.of(msg.complexity()))
		iface->set_complexity(complexity_enum_order.of(msg.complexity()));

	for (int i = 0; i < msg.ring_colors_size(); ++i)
		if (iface->ring_colors(static_cast<unsigned int>(i))
		    != ring_color_enum_order.of(msg.ring_colors(i)))
			iface->set_ring_colors(static_cast<unsigned int>(i),
			                       ring_color_enum_order.of(msg.ring_colors(i)));

	if (iface->order_id() != msg.id())
		iface->set_order_id(msg.id());
	if (iface->delivery_period_begin() != msg.delivery_period_begin())
		iface->set_delivery_period_begin(msg.delivery_period_begin());
	if (iface->delivery_period_end() != msg.delivery_period_end())
		iface->set_delivery_period_end(msg.delivery_period_end());
	if (iface->delivery_gate() != msg.delivery_gate())
		iface->set_delivery_gate(msg.delivery_gate());
	if (iface->quantity_delivered_cyan() != msg.quantity_delivered_cyan())
		iface->set_quantity_delivered_cyan(msg.quantity_delivered_cyan());
	if (iface->quantity_delivered_magenta() != msg.quantity_delivered_magenta())
		iface->set_quantity_delivered_magenta(msg.quantity_delivered_magenta());
	if (iface->quantity_requested() != msg.quantity_requested())
		iface->set_quantity_requested(msg.quantity_requested());
};

/****************************************
 *         RecvBeaconInterface          *
 ****************************************/

template <>
std::string
iface_id_for_type<RecvBeaconInterface>()
{
	return "/protoboard/beacon";
}

static const enum_map<llsf_msgs::Team, RecvBeaconInterface::TEAM_COLOR> team_enum_recv_beacon{
  {llsf_msgs::Team::CYAN, RecvBeaconInterface::TEAM_COLOR::CYAN},
  {llsf_msgs::Team::MAGENTA, RecvBeaconInterface::TEAM_COLOR::MAGENTA}};

template <>
void
pb_converter<llsf_msgs::BeaconSignal, RecvBeaconInterface>::handle(
  const llsf_msgs::BeaconSignal &msg,
  RecvBeaconInterface *          iface)
{
	iface->set_number(msg.number());
	iface->set_peer_name(msg.peer_name().substr(0, iface->maxlenof_peer_name()).c_str());
	iface->set_pose(0, msg.pose().x());
	iface->set_pose(1, msg.pose().y());
	iface->set_pose(2, msg.pose().ori());
	iface->set_team_color(team_enum_recv_beacon.of(msg.team_color()));
	iface->set_team_name(msg.team_name().substr(0, iface->maxlenof_team_name()).c_str());
	iface->set_time_nsec(msg.time().nsec());
	iface->set_time_sec(msg.time().sec());
}

/****************************************
 *          GameStateInterface          *
 ****************************************/

template <>
std::string
iface_id_for_type<RCLLGameStateInterface>()
{
	return "/protoboard/game_state";
}

static const enum_map<llsf_msgs::GameState::State, RCLLGameStateInterface::GameState>
  game_state_enum{{llsf_msgs::GameState::State::GameState_State_INIT,
                   RCLLGameStateInterface::GameState::INIT},
                  {llsf_msgs::GameState::State::GameState_State_PAUSED,
                   RCLLGameStateInterface::GameState::PAUSED},
                  {llsf_msgs::GameState::State::GameState_State_RUNNING,
                   RCLLGameStateInterface::GameState::RUNNING},
                  {llsf_msgs::GameState::State::GameState_State_WAIT_START,
                   RCLLGameStateInterface::GameState::WAIT_START}};

static const enum_map<llsf_msgs::GameState::Phase, RCLLGameStateInterface::GamePhase>
  game_phase_enum{{llsf_msgs::GameState::Phase::GameState_Phase_EXPLORATION,
                   RCLLGameStateInterface::GamePhase::EXPLORATION},
                  {llsf_msgs::GameState::Phase::GameState_Phase_POST_GAME,
                   RCLLGameStateInterface::GamePhase::POST_GAME},
                  {llsf_msgs::GameState::Phase::GameState_Phase_PRE_GAME,
                   RCLLGameStateInterface::GamePhase::PRE_GAME},
                  {llsf_msgs::GameState::Phase::GameState_Phase_PRODUCTION,
                   RCLLGameStateInterface::GamePhase::PRODUCTION},
                  {llsf_msgs::GameState::Phase::GameState_Phase_SETUP,
                   RCLLGameStateInterface::GamePhase::SETUP}};

template <>
void
pb_converter<llsf_msgs::GameState, RCLLGameStateInterface>::handle(const llsf_msgs::GameState &msg,
                                                                   RCLLGameStateInterface *iface)
{
	iface->set_game_time_nsec(msg.game_time().nsec());
	iface->set_game_time_sec(msg.game_time().sec());
	iface->set_phase(game_phase_enum.of(msg.phase()));
	iface->set_points_cyan(msg.points_cyan());
	iface->set_points_magenta(msg.points_magenta());
	iface->set_state(game_state_enum.of(msg.state()));
	iface->set_team_cyan(msg.team_cyan().substr(0, iface->maxlenof_team_cyan()).c_str());
	iface->set_team_magenta(msg.team_magenta().substr(0, iface->maxlenof_team_magenta()).c_str());
}

/****************************************
 *          MachineInfoInterface        *
 ****************************************/

template <>
std::string
iface_id_for_type<MachineInfoInterface>()
{
	return "/protoboard/machine_info";
}

static const enum_map<llsf_msgs::RingColor, MachineInfoInterface::RingColor>
  machine_info_ring_color{
    {llsf_msgs::RingColor::RING_BLUE, MachineInfoInterface::RING_BLUE},
    {llsf_msgs::RingColor::RING_GREEN, MachineInfoInterface::RING_GREEN},
    {llsf_msgs::RingColor::RING_ORANGE, MachineInfoInterface::RING_ORANGE},
    {llsf_msgs::RingColor::RING_YELLOW, MachineInfoInterface::RING_YELLOW},
  };

static const enum_map<llsf_msgs::Team, MachineInfoInterface::Team> mi_team_color_enum{
  {llsf_msgs::CYAN, MachineInfoInterface::Team::CYAN},
  {llsf_msgs::MAGENTA, MachineInfoInterface::Team::MAGENTA}};

template <>
std::string
pb_converter<llsf_msgs::Machine, MachineInfoInterface>::get_sequence_id(
  const llsf_msgs::Machine &msg)
{
	return msg.name();
}

template <>
const google::protobuf::RepeatedPtrField<llsf_msgs::Machine> &
pb_sequence_converter<llsf_msgs::MachineInfo,
                      pb_converter<llsf_msgs::Machine, MachineInfoInterface>>::
  extract_sequence(const llsf_msgs::MachineInfo &msg)
{
	return msg.machines();
}

template <>
void
pb_converter<llsf_msgs::Machine, MachineInfoInterface>::handle(const llsf_msgs::Machine &msg,
                                                               MachineInfoInterface *    iface)
{
	float pose[3]{msg.pose().x(), msg.pose().y(), msg.pose().ori()};
	iface->set_pose(pose);
	std::string zone = llsf_msgs::Zone_Name(msg.zone());
	for (std::string::size_type idx = zone.find('_'); idx != std::string::npos; idx = zone.find('_'))
		zone[idx] = '-';
	iface->set_zone(zone.c_str());
	iface->set_state(msg.state().c_str());
	iface->set_rotation(msg.rotation());
	iface->set_team_color(mi_team_color_enum.of(msg.team_color()));
	iface->set_loaded_with(msg.loaded_with());

	if (msg.ring_colors_size() == 2) {
		MachineInfoInterface::RingColor ring_colors[2]{machine_info_ring_color.of(msg.ring_colors(0)),
		                                               machine_info_ring_color.of(msg.ring_colors(1))};
		iface->set_ring_colors(ring_colors);
	} else if (msg.ring_colors_size() == 0) {
		MachineInfoInterface::RingColor ring_colors[2]{MachineInfoInterface::RING_NONE,
		                                               MachineInfoInterface::RING_NONE};
		iface->set_ring_colors(ring_colors);
	} else {
		logger_->log_error(name(),
		                   "Unexpected: MachineInfo message for %s contains %d ring colors",
		                   msg.name().c_str(),
		                   msg.ring_colors_size());
	}

	iface->set_machine_name(msg.name().c_str());
	iface->set_machine_type(msg.type().c_str());
	iface->set_correctly_reported(msg.correctly_reported());
}

/****************************************
 *          RingInfoInterface           *
 ****************************************/

template <>
std::string
iface_id_for_type<RingInfoInterface>()
{
	return "/protoboard/ring_info";
}

static const enum_map<llsf_msgs::RingColor, RingInfoInterface::RingColor> ring_info_color_enum{
  {llsf_msgs::RingColor::RING_BLUE, RingInfoInterface::RING_BLUE},
  {llsf_msgs::RingColor::RING_GREEN, RingInfoInterface::RING_GREEN},
  {llsf_msgs::RingColor::RING_ORANGE, RingInfoInterface::RING_ORANGE},
  {llsf_msgs::RingColor::RING_YELLOW, RingInfoInterface::RING_YELLOW},
};

template <>
void
pb_converter<llsf_msgs::RingInfo, RingInfoInterface>::handle(const llsf_msgs::RingInfo &msg,
                                                             RingInfoInterface *        iface)
{
	if (msg.rings_size() != 4) {
		logger_->log_error(name(),
		                   "Unexpected: RingInfo message with %d entries instead of %d",
		                   msg.rings_size(),
		                   4);
	} else {
		RingInfoInterface::RingColor ring_colors[4]{ring_info_color_enum.of(msg.rings(0).ring_color()),
		                                            ring_info_color_enum.of(msg.rings(1).ring_color()),
		                                            ring_info_color_enum.of(msg.rings(2).ring_color()),
		                                            ring_info_color_enum.of(msg.rings(3).ring_color())};
		uint32_t                     cost[4]{msg.rings(0).raw_material(),
                     msg.rings(1).raw_material(),
                     msg.rings(2).raw_material(),
                     msg.rings(3).raw_material()};
		iface->set_ring_colors(ring_colors);
		iface->set_raw_materials(cost);
	}
}

} // namespace protoboard
