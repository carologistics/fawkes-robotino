#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <list>
#include <map>
#include <queue>

#include <protobuf_comm/server.h>
#include <core/threading/mutex.h>

#define CFG_PREFIX "/plugins/protoboard"

namespace protobuf_comm {
  class ProtobufStreamClient;
  class ProtobufBroadcastPeer;
}


namespace protoboard {


class BlackboardManager;


class ProtobufThead :
    public fawkes::Thread,
    public fawkes::LoggingAspect,
    public fawkes::ConfigurableAspect
{
public:
  ProtobufThead();
  virtual ~ProtobufThead() override;

  bool pb_queue_incoming();

  struct incoming_message {
    long int peer_id;
    boost::asio::ip::udp::endpoint endpoint;
    uint16_t component_id;
    uint16_t msg_type;
    std::shared_ptr<google::protobuf::Message> msg;
  };

  incoming_message pb_queue_pop();

  long int peer_create(const std::string &host, int port);
  long int peer_create_local(const std::string &host,
                             int send_to_port, int recv_on_port);
  long int peer_create_crypto(const std::string &host, int port,
                              const std::string &crypto_key = "", const std::string &cipher = "");
  long int peer_create_local_crypto(const std::string &host,
                                    int send_to_port, int recv_on_port,
                                    const std::string &crypto_key = "", const std::string &cipher = "");
  void peer_destroy(long int peer_id);

  void send(long int peer_id, std::shared_ptr<google::protobuf::Message> msg);

  /** Get the communicator's message register.
   * @return message register */
  protobuf_comm::MessageRegister &  message_register()
  { return *message_register_; }

  void set_bb_manager(BlackboardManager *bb_manager)
  { bb_manager_ = bb_manager; }

protected:
  virtual void init() override;

private:
  /** Get protobuf_comm peers.
   * @return protobuf_comm peer */
  const std::map<long int, protobuf_comm::ProtobufBroadcastPeer *> &
  peers() const
  { return peers_; }

  BlackboardManager *bb_manager_;

  /** Signal invoked for a message that has been sent via broadcast.
   * @return signal
   */
  boost::signals2::signal<void (long,
                                std::shared_ptr<google::protobuf::Message>)> &
    signal_peer_sent() { return sig_peer_sent_; }


  void peer_setup_crypto(long int peer_id, const std::string &crypto_key, const std::string &cipher);


  void handle_peer_msg(long int peer_id,
                       boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void wait_for_first_ga(long int peer_id,
                       boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(long int peer_id, boost::asio::ip::udp::endpoint &endpoint, std::string msg);
  void handle_peer_send_error(long int peer_id, std::string msg);

  protobuf_comm::MessageRegister       *message_register_;
  protobuf_comm::ProtobufStreamServer  *server_;

  boost::signals2::signal<void (long int, std::shared_ptr<google::protobuf::Message>)> sig_peer_sent_;

  fawkes::Mutex map_mutex_;
  fawkes::Mutex msgq_mutex_;
  long int next_client_id_;

  std::map<long int, protobuf_comm::ProtobufBroadcastPeer *> peers_;

  std::queue<incoming_message> pb_queue_;

};


} // namespace protoboard


#endif // MESSAGE_HANDLER_H
