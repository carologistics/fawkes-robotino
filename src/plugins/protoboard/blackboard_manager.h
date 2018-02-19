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

#include <boost/core/demangle.hpp>

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
class enum_map {
public:
  typedef boost::bimap<pbEnumT, bbEnumT> bimapT;
  constexpr enum_map(std::initializer_list<typename bimapT::value_type> init)
    : list(init),
      map(list.begin(), list.end())
  {}

  constexpr bbEnumT of(pbEnumT v) const
  { return map.left.at(v); }

  constexpr pbEnumT of(bbEnumT v) const
  { return map.right.at(v); }

private:
  const std::vector<typename bimapT::value_type> list;
  const bimapT map;
};


class pb_convert : public std::enable_shared_from_this<pb_convert> {
public:
  pb_convert()
    : blackboard_(nullptr)
    , logger_(nullptr)
  {}

  pb_convert(const pb_convert &) = default;
  pb_convert &operator = (const pb_convert &) = default;

  virtual ~pb_convert();

  virtual void
  init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, size_t = 0)
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
    : pb_convert()
    , interface_(nullptr)
  {}

  // Don't copy this
  pb_converter(const pb_converter<ProtoT, IfaceT> &) = delete;
  pb_converter<ProtoT, IfaceT> &operator = (const pb_converter<ProtoT, IfaceT> &) = delete;

  // Only move!
  pb_converter(pb_converter<ProtoT, IfaceT> &&o)
    : pb_convert(o)
    , interface_(std::move(o.interface_))
  { o.interface_ = nullptr; }

  pb_converter<ProtoT, IfaceT> &operator = (pb_converter<ProtoT, IfaceT> &&o)
  {
    pb_convert::operator = (o);
    this->interface_ = o.interface_;
    o.interface_ = nullptr;
    return *this;
  }


  virtual ~pb_converter()
  { close(); }

  virtual void
  init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, size_t id = 0) override
  {
    pb_convert::init(blackboard, logger);
    interface_ = blackboard_->open_for_writing<IfaceT>(
          iface_id_for_type<IfaceT>(id).c_str());
    logger->log_info(
          boost::core::demangle(typeid(*this).name()).c_str(),
          "Initialized %s.",
          iface_id_for_type<IfaceT>(id).c_str());
  }

  virtual void
  handle(const google::protobuf::Message &msg) override
  { handle(dynamic_cast<const ProtoT &>(msg)); }

  virtual void handle(const ProtoT &msg)
  {
    handle(msg, interface_);
    interface_->write();
  }

  virtual bool is_open()
  { return interface_; }

  virtual void close()
  {
    if (is_open()) {
      blackboard_->close(interface_);
      interface_ = nullptr;
    }
  }

  IfaceT *interface()
  { return interface_; }

  virtual bool corresponds_to(const ProtoT &msg)
  {
    if (is_open()) {
      interface()->read();
      return corresponds(msg, interface());
    }
    else
      return true;
  }

protected:
  virtual void handle(const ProtoT &msg, IfaceT *iface);
  static bool corresponds(const ProtoT &, const IfaceT *)
  {
    throw fawkes::Exception(
          boost::core::demangle(typeid(pb_converter<ProtoT, IfaceT>).name()).c_str(),
          "BUG: corresponds(...) must "
          "be overridden for every converter used in a sequence.");
  }


private:
  IfaceT *interface_;
};


template<class ProtoT, class OutputT>
class pb_sequence_converter : public pb_convert {
public:
  typedef google::protobuf::RepeatedPtrField<typename OutputT::input_type> sequence_type;

  pb_sequence_converter()
      : sub_converters_()
      , seq_id_(0)
  {}

  virtual void
  handle(const google::protobuf::Message &msg) override
  {
    typename std::vector<OutputT>::iterator out_it = sub_converters_.begin();
    sequence_type fields = extract_sequence(dynamic_cast<const ProtoT &>(msg));
    typename sequence_type::const_iterator field_it = fields.begin();

    for ( ; field_it != fields.end(); ++field_it) {
      // Try to find a corresponding output converter
      for (out_it = sub_converters_.begin(); out_it != sub_converters_.end(); ++out_it) {
        if (out_it->corresponds_to(*field_it))
          break;
      }

      if (out_it == sub_converters_.end()) {
        // No corresponding converter found, create new
        sub_converters_.emplace_back();
        out_it = --sub_converters_.end();
      }

      if (!out_it->is_open())
        out_it->init(blackboard_, logger_, seq_id_++);
      out_it->handle(*field_it);
    }

    sub_converters_.erase(out_it + 1, sub_converters_.end());
  }

  virtual const sequence_type &extract_sequence(const ProtoT &msg);

private:
  std::vector<OutputT> sub_converters_;
  size_t seq_id_;
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
