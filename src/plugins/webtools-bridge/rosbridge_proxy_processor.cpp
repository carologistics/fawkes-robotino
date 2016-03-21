
#include "rosbridge_proxy_processor.h"
#include "web_session.h"

/*This acts as a proxy to RosBridge. Each session has a uniques proxy inatance where 
the incoming and outgoing intercations are uniquly mapped 
*/

RosBridgeProxyProcessor::RosBridgeProxyProcessor(std::string prefix , fawkes::Logger *logger)
:
{
   rosbridge_endpoint_.clear_access_channels(websocketpp::log::alevel::all);
   rosbridge_endpoint_.clear_error_channels(websocketpp::log::elevel::all);
   rosbridge_endpoint_.init_asio();
   rosbridge_endpoint_.start_perpetual();

}

RosBridgeProxyProcessor::~RosBridgeProxyProcessor()
{
    rosbridge_endpoint_.stop_perpetual();

    websocketpp::lib::error_code ec;

    for (it_proxy_= session_proxy_map_.begin()
        ;it_proxy_!= sessoin_proxy_map_.end();
        it_proxy_++)
    {
      rosbridge_endpoint_.close(it_proxy_->second , websocketpp::close::status::going_away, "", ec);
      if (ec) {
          std::cout << "> Error closing connection " << it_proxy_->second << ": "  
                    << ec.message() << std::endl;
      }
    }
   
    //m_thread->join();

}


//Session Operations
void
RosBridgeProxyProcessor::new_client()
{
    std::string const & uri
    
    websocketpp::lib::error_code ec;

    websocketpp::client<websocketpp::config::asio_client>::connection_ptr con;
    con = rosbridge_endpoint_.get_connection(uri, ec);

    if (ec) {
        logger_->log_info("Webtools-bridge:"," Connect initialization error: %s" , ec.message().c_str());
        return -1;
    }

    // con->set_close_handler(websocketpp::lib::bind(
    //     &connection_metadata::on_close,
    //     metadata_ptr,
    //     &m_endpoint,
    //     websocketpp::lib::placeholders::_1
    // ));

    con->set_message_handler(websocketpp::lib::bind(
        &RosBridgeProxyProcessor::proxy_on_message,
        this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2
    ));

    rosbirdge_endpoint_.connect(con);

}

void
RosBridgeProxyProcessor::remove_client()
{

}

void
RosBridgeProxyProcessor::proxy_send(WebSession)
{
   websocketpp::lib::error_code ec;
    rosbridge_endpoint_.send(metadata_ptr->get_hdl(), message, websocketpp::frame::opcode::text, ec);
    if (ec) {
      std::cout << "> Error sending message to ros: " << ec.message() << std::endl;
      return;
    }


}

void
RosBridgeProxyProcessor::proxy_on_message()
{

}

//handles session termination
void
RosBridgeProxyProcessor::callback  ( EventType event_type , std::shared_ptr <EventEmitter> handler) 
{

}

//Capabilities
std::shared_ptr<Subscription>
RosBridgeProxyProcessor::subscribe   ( std::string topic_name 
                                            , std::string id    
                                            , std::string compression
                                            , unsigned int throttle_rate  
                                            , unsigned int queue_length   
                                            , unsigned int fragment_size  
                                            , std::shared_ptr<WebSession> session)
{

}

void 
RosBridgeProxyProcessor::unsubscribe ( std::string id
                                            , std::shared_ptr<Subscription> 
                                           , std::shared_ptr<WebSession> session ) 
{

}
