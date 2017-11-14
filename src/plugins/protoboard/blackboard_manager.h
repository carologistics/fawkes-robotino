#ifndef BLACKBOARD_MANAGER_H
#define BLACKBOARD_MANAGER_H

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <blackboard/utils/on_message_waker.h>

#include <boost/fusion/include/std_tuple.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/any.hpp>

#include <boost/bimap.hpp>

#include <type_traits>
#include <vector>
#include <unordered_map>

#include "interface_definitions.h"
#include "protobuf_thread.h"



namespace protoboard {


template<class IfaceT>
struct InterfaceMessageHandler;


class BlackboardManager;


template<class pbEnumT, class bbEnumT>
struct enum_map {
  typedef boost::bimap<pbEnumT, bbEnumT> bimapT;
  const std::vector<typename bimapT::value_type> list;
  const boost::bimap<pbEnumT, bbEnumT> map;

  constexpr enum_map(std::initializer_list<typename bimapT::value_type> init)
    : list(init),
      map(list.begin(), list.end())
  {}

  constexpr bbEnumT of(pbEnumT v) const
  { return map.left.at(v); }

  constexpr pbEnumT of(bbEnumT v) const
  { return map.right.at(v); }
};


class pb_convert : public std::enable_shared_from_this<pb_convert> {
public:
  pb_convert()
    : blackboard_(nullptr)
    , logger_(nullptr)
  {}

  virtual ~pb_convert();

  virtual void
  init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger)
  {
    blackboard_ = blackboard;
    logger_ = logger;
  }

  virtual void handle(std::shared_ptr<google::protobuf::Message> msg)
  { handle(*msg); }

  virtual void handle(const google::protobuf::Message &)
  {}

protected:
  fawkes::BlackBoard *blackboard_;
  fawkes::Logger *logger_;
};


template<class ProtoT, class IfaceT>
class pb_converter : public pb_convert {
public:
  typedef ProtoT input_type;
  typedef IfaceT output_type;

  pb_converter()
      : interface_(nullptr)
  {}

  virtual ~pb_converter()
  { blackboard_->close(interface_); }

  virtual void
  init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger) override
  {
    pb_convert::init(blackboard, logger);
    interface_ = blackboard_->open_for_writing<IfaceT>(
          iface_id_for_type<IfaceT>(0).c_str());
  }

  virtual void
  handle(const google::protobuf::Message &msg) override
  {
    handle(dynamic_cast<const ProtoT &>(msg), interface_);
    interface_->write();
  }

  virtual void handle(const ProtoT &msg)
  { handle(msg, interface_); }

  virtual void handle(const ProtoT &msg, IfaceT *iface);

  virtual bool is_open()
  { return interface_; }

  virtual void close()
  {
    if (is_open()) {
      blackboard_->close(interface_);
      interface_ = nullptr;
    }
  }

private:
  IfaceT *interface_;
};


template<class ProtoT, class OutputT, size_t seq_length>
class pb_sequence_converter : public pb_convert {
public:
  typedef google::protobuf::RepeatedPtrField<typename OutputT::input_type> sequence_type;

  pb_sequence_converter()
      : sub_converters_(seq_length)
  {}

  virtual void
  handle(const google::protobuf::Message &msg) override
  {
    typename std::vector<OutputT>::iterator out_it = sub_converters_.begin();
    sequence_type fields = extract_sequence(dynamic_cast<const ProtoT &>(msg));
    typename sequence_type::const_iterator field_it = fields.begin();

    while (out_it != sub_converters_.end() && field_it != fields.end()) {
      if (!out_it->is_open())
        out_it->init(blackboard_, logger_);
      out_it->handle(*field_it);

      ++out_it;
      ++field_it;
    }
    for ( ; out_it != sub_converters_.end(); ++out_it) {
      if (out_it->is_open())
        out_it->close();
    }
  }

  virtual const sequence_type &extract_sequence(const ProtoT &msg);

private:
  std::vector<OutputT> sub_converters_;
};


typedef std::unordered_map<std::string, std::shared_ptr<pb_convert>> pb_conversion_map;


pb_conversion_map make_receiving_interfaces_map();


class BlackboardManager :
    public fawkes::Thread,
    public fawkes::LoggingAspect,
    public fawkes::ConfigurableAspect,
    public fawkes::BlackBoardAspect,
    public fawkes::ClockAspect
{
public:
  BlackboardManager(ProtobufThead *msg_handler);

protected:
  virtual void init() override;
  virtual void finalize() override;
  virtual void loop() override;

private:
  void add_peer(fawkes::ProtobufPeerInterface *iface, long peer_id);

  template<class MessageT, class InterfaceT>
  bool handle_message_type(InterfaceT *iface)
  {
    if (!iface->msgq_empty()) {
      bool rv = false;
      while (MessageT *msg = iface->msgq_first_safe(msg)) {
        try {
          handle_message(iface, msg);
        } catch (std::exception &e) {
          logger->log_error(name(), "Exception handling %s on %s: %s",
                            msg->type(), iface->uid(), e.what());
        }
        iface->msgq_pop();
        rv = true;
      }
      iface->write();
      return rv;
    } else
      return false;
  }

  template<class InterfaceT, class MessageT>
  void handle_message(InterfaceT *iface, MessageT *msg);


  template<class InterfaceT>
  struct on_interface {
    InterfaceT *iface;
    BlackboardManager *manager;

    on_interface(InterfaceT *iface, BlackboardManager *manager)
      : iface(iface), manager(manager)
    {}

    template<class MessageT>
    bool handle_msg_types ()
    {
      return manager->handle_message_type<MessageT>(iface);
    }

    // This template is disabled if MessageTs is {} to resolve ambiguity
    template<class MessageT1, class... MessageTs>
    typename std::enable_if<(sizeof...(MessageTs) > 0), bool>::type
    handle_msg_types ()
    { return handle_msg_types<MessageTs...>() || handle_msg_types<MessageT1>(); }
  };


  struct init_interface {
    BlackboardManager *manager;
    template<class IfaceT, class... MessageTs>
    void operator() (bb_iface_manager<IfaceT, type_list<MessageTs...>> &iface_mgr) const {
      iface_mgr.init(manager->blackboard, manager);
    }
  };


  struct handle_messages {
    BlackboardManager *manager;

    template<class IfaceT, class MessageT>
    bool operator() (const bb_iface_manager<IfaceT, type_list<MessageT>> &pair) const
    { return manager->handle_message_type<MessageT>(pair.interface()); }

    template<class IfaceT, class MessageT1, class... MessageTs>
    bool operator() (const bb_iface_manager<IfaceT, type_list<MessageT1, MessageTs...>> &iface_mgr) const
    {
      return on_interface<IfaceT>{iface_mgr.interface(), manager}.template handle_msg_types<MessageTs...>()
          || manager->handle_message_type<MessageT1>(iface_mgr.interface());
    }
  };



  ProtobufThead *message_handler_;
  fawkes::ProtobufPeerInterface *peer_iface_;

  sending_interfaces bb_sending_interfaces_;

  pb_conversion_map bb_receiving_interfaces_;

  fawkes::BlackBoardOnMessageWaker *on_message_waker_;

  unsigned int next_peer_idx_;

};


} // namespace protoboard


#endif // BLACKBOARD_MANAGER_H
