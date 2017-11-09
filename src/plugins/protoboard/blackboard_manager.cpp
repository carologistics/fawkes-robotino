#include "blackboard_manager.h"
#include "bb_msg_handlers.cpp"


namespace protoboard {


using namespace fawkes;


pb_convert::~pb_convert()
{}


BlackboardManager::BlackboardManager(ProtobufThead *msg_handler)
  : Thread("ProtoboardBlackboardManager", Thread::OPMODE_WAITFORWAKEUP)
  , message_handler_(msg_handler)
  , bb_receiving_interfaces_(make_receiving_interfaces_map())
  , on_message_waker_(nullptr)
  , next_peer_idx_(0)
{}


void BlackboardManager::init()
{
  peer_iface_ = blackboard->open_for_writing<ProtobufPeerInterface>("/protoboard/peers");

  for (pb_conversion_map::value_type &c : bb_receiving_interfaces_)
    c.second->init(blackboard, logger);

  on_message_waker_ = new fawkes::BlackBoardOnMessageWaker(blackboard, peer_iface_, this);
  boost::fusion::for_each(bb_sending_interfaces_, init_interface{this});
}


void BlackboardManager::finalize()
{
  delete on_message_waker_;
  blackboard->close(peer_iface_);
}


void BlackboardManager::loop()
{
  // Handle CreatePeer* messages
  bool did_something = on_interface<ProtobufPeerInterface> { peer_iface_, this }
      .handle_msg_types <
        ProtobufPeerInterface::CreatePeerMessage,
        ProtobufPeerInterface::CreatePeerLocalMessage,
        ProtobufPeerInterface::CreatePeerCryptoMessage,
        ProtobufPeerInterface::CreatePeerLocalCryptoMessage
      > ();

  // Handle sending blackboard interfaces
  did_something |= boost::fusion::any(bb_sending_interfaces_, handle_messages { this });

  // Handle receiving blackboard interfaces
  while (message_handler_->pb_queue_incoming()) {
    ProtobufThead::incoming_message inc = message_handler_->pb_queue_pop();
    pb_conversion_map::iterator it;

    if ((it = bb_receiving_interfaces_.find(inc.msg->GetTypeName())) == bb_receiving_interfaces_.end())
      logger->log_error(name(), "Received message of unregistered type `%s'", inc.msg->GetTypeName().c_str());
    else try {
      it->second->handle(inc.msg);
    } catch (std::exception &e) {
      logger->log_error(name(), "Exception while handling %s: %s",
                        inc.msg->GetTypeName().c_str(), e.what());
    }

    did_something = true;
  }

  if (!did_something)
    // Thread woke up, but nothing was handled
    logger->log_warn(name(), "Spurious wakeup. WTF?");
}


void BlackboardManager::add_peer(ProtobufPeerInterface *iface, long peer_id)
{
  // TODO: Properly handle overflow.
  iface->set_peers(next_peer_idx_++ % iface->maxlenof_peers(), peer_id);
  iface->write();
}


template<> void
BlackboardManager::handle_message(
    ProtobufPeerInterface *iface,
    ProtobufPeerInterface::CreatePeerMessage *msg)
{ add_peer(iface, message_handler_->peer_create(msg->address(), msg->port())); }


template<> void
BlackboardManager::handle_message(
    ProtobufPeerInterface *iface,
    ProtobufPeerInterface::CreatePeerLocalMessage *msg)
{
  add_peer(iface,
           message_handler_->peer_create_local(
             msg->address(),
             msg->send_to_port(),
             msg->recv_on_port()));
}


template<> void
BlackboardManager::handle_message(
    ProtobufPeerInterface *iface,
    ProtobufPeerInterface::CreatePeerCryptoMessage *msg)
{
  add_peer(iface,
           message_handler_->peer_create_crypto(
             msg->address(),
             msg->port(),
             msg->crypto_key(),
             msg->cipher()));
}


template<> void
BlackboardManager::handle_message(
    ProtobufPeerInterface *iface,
    ProtobufPeerInterface::CreatePeerLocalCryptoMessage *msg)
{
  add_peer(iface,
           message_handler_->peer_create_local_crypto(
             msg->address(),
             msg->send_to_port(),
             msg->recv_on_port(),
             msg->crypto_key(),
             msg->cipher()));
}


} // namespace protoboard

