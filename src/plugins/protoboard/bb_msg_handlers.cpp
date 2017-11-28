#include "blackboard_manager.h"

#include <interfaces/OrderInterface.h>
#include <interfaces/RCLLGameStateInterface.h>
#include <interfaces/RecvBeaconInterface.h>

#include <libs/llsf_msgs/BeaconSignal.pb.h>
#include <libs/llsf_msgs/MachineInstructions.pb.h>
#include <libs/llsf_msgs/GameState.pb.h>
#include <libs/llsf_msgs/OrderInfo.pb.h>
#include <libs/llsf_msgs/RobotInfo.pb.h>
#include <libs/llsf_msgs/VersionInfo.pb.h>

namespace protoboard {


using namespace fawkes;
using namespace std;
using namespace boost::fusion;


std::unordered_map<std::string, std::shared_ptr<pb_convert>> make_receiving_interfaces_map() {
  return {
    make_pair(
          "llsf_msgs.OrderInfo",
          make_shared<
            pb_sequence_converter<
              llsf_msgs::OrderInfo,
              pb_converter<llsf_msgs::Order, OrderInterface>
            >
          >()),
    make_pair("llsf_msgs.BeaconSignal", make_shared<pb_converter<llsf_msgs::BeaconSignal, RecvBeaconInterface>>()),
    make_pair("llsf_msgs.GameState", make_shared<pb_converter<llsf_msgs::GameState, RCLLGameStateInterface>>()),

    // Dummy handler, i.e. discard message
    make_pair("llsf_msgs.RobotInfo", make_shared<pb_convert>()),
    make_pair("llsf_msgs.VersionInfo", make_shared<pb_convert>())
  };
};


/****************************************
 *           OrderInterface             *
 ****************************************/

static const enum_map<llsf_msgs::BaseColor, OrderInterface::BaseColor>
base_color_enum_order {
  {llsf_msgs::BaseColor::BASE_BLACK, OrderInterface::BASE_BLACK},
  {llsf_msgs::BaseColor::BASE_RED, OrderInterface::BASE_RED},
  {llsf_msgs::BaseColor::BASE_SILVER, OrderInterface::BASE_SILVER}
};

static const enum_map<llsf_msgs::RingColor, OrderInterface::RingColor>
ring_color_enum_order {
  {llsf_msgs::RingColor::RING_BLUE, OrderInterface::RING_BLUE},
  {llsf_msgs::RingColor::RING_GREEN, OrderInterface::RING_GREEN},
  {llsf_msgs::RingColor::RING_ORANGE, OrderInterface::RING_ORANGE},
  {llsf_msgs::RingColor::RING_YELLOW, OrderInterface::RING_YELLOW},
};

static const enum_map<llsf_msgs::CapColor, OrderInterface::CapColor>
cap_color_enum_order {
  {llsf_msgs::CapColor::CAP_BLACK, OrderInterface::CAP_BLACK},
  {llsf_msgs::CapColor::CAP_GREY, OrderInterface::CAP_GREY},
};

static const enum_map<llsf_msgs::Order::Complexity, OrderInterface::Complexity>
complexity_enum_order {
  {llsf_msgs::Order::Complexity::Order_Complexity_C0, OrderInterface::C0},
  {llsf_msgs::Order::Complexity::Order_Complexity_C1, OrderInterface::C1},
  {llsf_msgs::Order::Complexity::Order_Complexity_C2, OrderInterface::C2},
  {llsf_msgs::Order::Complexity::Order_Complexity_C3, OrderInterface::C3}
};


template<>
const google::protobuf::RepeatedPtrField<llsf_msgs::Order> &
pb_sequence_converter<llsf_msgs::OrderInfo, pb_converter<llsf_msgs::Order, OrderInterface>>::extract_sequence(
    const llsf_msgs::OrderInfo &msg)
{ return msg.orders(); }


template<>
bool
pb_converter<llsf_msgs::Order, OrderInterface>::corresponds(const llsf_msgs::Order &msg, const OrderInterface *iface)
{ return iface->order_id() == msg.id(); }


template<>
void
pb_converter<llsf_msgs::Order, OrderInterface>::handle(
    const llsf_msgs::Order &msg,
    OrderInterface *iface)
{
  if (iface->base_color() != base_color_enum_order.of(msg.base_color()))
    iface->set_base_color(base_color_enum_order.of(msg.base_color()));
  if (iface->cap_color() != cap_color_enum_order.of(msg.cap_color()))
    iface->set_cap_color(cap_color_enum_order.of(msg.cap_color()));
  if (iface->complexity() != complexity_enum_order.of(msg.complexity()))
    iface->set_complexity(complexity_enum_order.of(msg.complexity()));

  for (int i = 0; i < msg.ring_colors_size(); ++i)
    if (iface->ring_colors(static_cast<unsigned int>(i)) != ring_color_enum_order.of(msg.ring_colors(i)))
      iface->set_ring_colors(static_cast<unsigned int>(i), ring_color_enum_order.of(msg.ring_colors(i)));

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

static const enum_map<llsf_msgs::Team, RecvBeaconInterface::TEAM_COLOR>
team_enum_recv_beacon {
  {llsf_msgs::Team::CYAN, RecvBeaconInterface::TEAM_COLOR::CYAN},
  {llsf_msgs::Team::MAGENTA, RecvBeaconInterface::TEAM_COLOR::MAGENTA}
};


template<>
void
pb_converter<llsf_msgs::BeaconSignal, RecvBeaconInterface>::handle(
    const llsf_msgs::BeaconSignal &msg,
    RecvBeaconInterface *iface)
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


template<>
void
BlackboardManager::handle_message(
    SendBeaconInterface *iface,
    SendBeaconInterface::SetPeerMessage *msg)
{
  iface->set_peer_id(msg->peer_id());
}


/****************************************
 *          GameStateInterface          *
 ****************************************/

static const enum_map<llsf_msgs::GameState::State, RCLLGameStateInterface::GameState>
game_state_enum {
  {llsf_msgs::GameState::State::GameState_State_INIT, RCLLGameStateInterface::GameState::INIT},
  {llsf_msgs::GameState::State::GameState_State_PAUSED, RCLLGameStateInterface::GameState::PAUSED},
  {llsf_msgs::GameState::State::GameState_State_RUNNING, RCLLGameStateInterface::GameState::RUNNING},
  {llsf_msgs::GameState::State::GameState_State_WAIT_START, RCLLGameStateInterface::GameState::WAIT_START}
};

static const enum_map<llsf_msgs::GameState::Phase, RCLLGameStateInterface::GamePhase>
game_phase_enum {
  {llsf_msgs::GameState::Phase::GameState_Phase_EXPLORATION, RCLLGameStateInterface::GamePhase::EXPLORATION},
  {llsf_msgs::GameState::Phase::GameState_Phase_POST_GAME, RCLLGameStateInterface::GamePhase::POST_GAME},
  {llsf_msgs::GameState::Phase::GameState_Phase_PRE_GAME, RCLLGameStateInterface::GamePhase::PRE_GAME},
  {llsf_msgs::GameState::Phase::GameState_Phase_PRODUCTION, RCLLGameStateInterface::GamePhase::PRODUCTION},
  {llsf_msgs::GameState::Phase::GameState_Phase_SETUP, RCLLGameStateInterface::GamePhase::SETUP}
};


template<>
void
pb_converter<llsf_msgs::GameState, RCLLGameStateInterface>::handle(
    const llsf_msgs::GameState &msg,
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
 *          SendBeaconMessage           *
 ****************************************/

static const enum_map<llsf_msgs::Team, SendBeaconInterface::TEAM_COLOR>
team_enum_beacon {
  {llsf_msgs::CYAN, SendBeaconInterface::TEAM_COLOR::CYAN},
  {llsf_msgs::MAGENTA, SendBeaconInterface::TEAM_COLOR::MAGENTA}
};


template<>
void
BlackboardManager::handle_message(
    SendBeaconInterface *iface,
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


/****************************************
 *           Prepare Messages           *
 ****************************************/

static const enum_map<llsf_msgs::Team, PrepareMachineInterface::Team>
team_enum_prepare {
  {llsf_msgs::CYAN, PrepareMachineInterface::Team::CYAN},
  {llsf_msgs::MAGENTA, PrepareMachineInterface::Team::MAGENTA}
};

static const enum_map<llsf_msgs::BaseColor, PrepareMachineInterface::BaseColor>
base_color_enum {
  {llsf_msgs::BaseColor::BASE_BLACK, PrepareMachineInterface::BASE_BLACK},
  {llsf_msgs::BaseColor::BASE_RED, PrepareMachineInterface::BASE_RED},
  {llsf_msgs::BaseColor::BASE_SILVER, PrepareMachineInterface::BASE_SILVER}
};

static const enum_map<llsf_msgs::MachineSide, PrepareMachineInterface::MachineSide>
machine_side_enum {
  {llsf_msgs::MachineSide::INPUT, PrepareMachineInterface::MachineSide::INPUT},
  {llsf_msgs::MachineSide::OUTPUT, PrepareMachineInterface::MachineSide::OUTPUT}
};

static const enum_map<llsf_msgs::CSOp, PrepareMachineInterface::CSOp>
csop_enum {
  {llsf_msgs::CSOp::MOUNT_CAP, PrepareMachineInterface::CSOp::MOUNT_CAP},
  {llsf_msgs::CSOp::RETRIEVE_CAP, PrepareMachineInterface::CSOp::RETRIEVE_CAP}
};


template<>
void
BlackboardManager::handle_message(
    PrepareMachineInterface *iface,
    PrepareMachineInterface::PrepareBSMessage *msg)
{
  std::shared_ptr<llsf_msgs::PrepareMachine> m = std::make_shared<llsf_msgs::PrepareMachine>();
  llsf_msgs::PrepareInstructionBS *instr = new llsf_msgs::PrepareInstructionBS();
  instr->set_color(base_color_enum.of(msg->color()));
  instr->set_side(machine_side_enum.of(msg->side()));
  m->set_allocated_instruction_bs(instr);
  m->set_team_color(team_enum_prepare.of(msg->team_color()));
  m->set_machine(msg->machine());

  message_handler_->send(iface->peer_id(), m);
}


template<>
void
BlackboardManager::handle_message(
    fawkes::PrepareMachineInterface *iface,
    PrepareMachineInterface::PrepareCSMessage *msg)
{
  std::shared_ptr<llsf_msgs::PrepareMachine> m = std::make_shared<llsf_msgs::PrepareMachine>();
  llsf_msgs::PrepareInstructionCS *instr = new llsf_msgs::PrepareInstructionCS();
  instr->set_operation(csop_enum.of(msg->operation()));
  m->set_allocated_instruction_cs(instr);
  m->set_team_color(team_enum_prepare.of(msg->team_color()));
  m->set_machine(msg->machine());

  message_handler_->send(iface->peer_id(), m);
}

template<>
void
BlackboardManager::handle_message(
    fawkes::PrepareMachineInterface *iface,
    PrepareMachineInterface::PrepareDSMessage *msg)
{}

template<>
void
BlackboardManager::handle_message(
    fawkes::PrepareMachineInterface *iface,
    PrepareMachineInterface::PrepareRSMessage *msg)
{}

template<>
void
BlackboardManager::handle_message(
    fawkes::PrepareMachineInterface *iface,
    PrepareMachineInterface::PrepareSSMessage *msg)
{}

template<>
void
BlackboardManager::handle_message(
    fawkes::PrepareMachineInterface *iface,
    PrepareMachineInterface::SetPeerMessage *msg)
{}



} // namespace protoboard

