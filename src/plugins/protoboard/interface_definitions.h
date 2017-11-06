#ifndef INTERFACE_DEFINITIONS_H
#define INTERFACE_DEFINITIONS_H

#include <libs/blackboard/utils/on_message_waker.h>

#include <interfaces/SendBeaconInterface.h>
#include <interfaces/ProtobufPeerInterface.h>
#include <interfaces/PrepareMachineInterface.h>

#include <tuple>
#include <vector>


namespace protoboard {


template<class IfaceT> std::string iface_id_for_type();

template<class IfaceT> std::string iface_id_for_type(size_t idx)
{ return iface_id_for_type<IfaceT>() + std::to_string(idx); }


template<class IfaceT, class MessageTypeList>
class bb_iface_manager {
public:
  bb_iface_manager()
    : interface_(nullptr)
    , blackboard_(nullptr)
    , waker_(nullptr)
  {}

  void init(fawkes::BlackBoard *blackboard, fawkes::Thread *thread)
  {
    blackboard_ = blackboard;
    interface_ = blackboard_->open_for_writing<IfaceT>(iface_id_for_type<IfaceT>().c_str());
    waker_ = new fawkes::BlackBoardOnMessageWaker(blackboard, interface_, thread);
  }

  ~bb_iface_manager()
  {
    if (blackboard_ && interface_)
      blackboard_->close(interface_);
    if (waker_)
      delete waker_;
  }

  IfaceT *interface() const { return interface_; }

private:
  IfaceT *interface_;
  fawkes::BlackBoard *blackboard_;
  fawkes::BlackBoardOnMessageWaker *waker_;
};


template<typename... Ts>
struct type_list
{};


typedef std::tuple <
  bb_iface_manager <
    fawkes::SendBeaconInterface,
    type_list <
      fawkes::SendBeaconInterface::SendBeaconMessage,
      fawkes::SendBeaconInterface::SetPeerMessage
    >
  >,
  bb_iface_manager <
    fawkes::PrepareMachineInterface,
    type_list <
      fawkes::PrepareMachineInterface::PrepareBSMessage,
      fawkes::PrepareMachineInterface::PrepareCSMessage,
      fawkes::PrepareMachineInterface::PrepareDSMessage,
      fawkes::PrepareMachineInterface::PrepareRSMessage,
      fawkes::PrepareMachineInterface::PrepareSSMessage,
      fawkes::PrepareMachineInterface::SetPeerMessage
    >
  >
> sending_interfaces;


std::vector<std::string> proto_dirs();


} // namespace protoboard


#endif // INTERFACE_DEFINITIONS_H
