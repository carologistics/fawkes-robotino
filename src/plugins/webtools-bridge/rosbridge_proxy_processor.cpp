
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

    for (it_proxy_ = session_proxy_map_.begin();
         it_proxy_ != sessoin_proxy_map_.end() ; it_proxy_++)
    {
        rosbridge_endpoint_.close(it_proxy_->second , websocketpp::close::status::going_away, "", ec);
        it_proxy_->first->register_callback(EventType::TERMINATE , shared_from_this() );
        if (ec) 
        {
          std::cout << "> Error closing connection " << it_proxy_->second << ": "  
                    << ec.message() << std::endl;
        }
    }
   
    m_thread->join();

}


//Session Operations
void
RosBridgeProxyProcessor::create_connection(std::shared_ptr <WebSession> new_session)
{
    //if(was not inited)
    m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &rosbridge_endpoint_);  
    
    //create a proxy client connected to RosBirdge
    std::string const & uri="6060";
    
    websocketpp::lib::error_code ec;

    websocketpp::client<websocketpp::config::asio_client>::connection_ptr con;
    con = rosbridge_endpoint_.get_connection(uri, ec);

    if (ec) {
        logger_->log_info("Webtools-bridge:"," Connect initialization error: %s" , ec.message().c_str());
        return -1;
    }

    con->set_message_handler(websocketpp::lib::bind(
        &RosBridgeProxyProcessor::proxy_on_message,
        this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2
    ));

    rosbirdge_endpoint_.connect(con);

    //register for termination events
    new_session->register_callback(EventType::TERMINATE , shared_from_this() );
    //add the mapping entry from session to proxy_client
    session_proxy_map_[new_session]=con->get_handle();

}


void
RosBridgeProxyProcessor::proxy_send(std::shared_ptr <WebSession> session , std::string message)
{
    if(session_proxy_map_.find(session) == session_proxy_map_.end())
    {
        create_connection(session);
    }

    websocketpp::lib::error_code ec;
    rosbridge_endpoint_.send(session_proxy_map_[session], message, websocketpp::frame::opcode::text, ec);
    if (ec) {
      std::cout << "> Error sending message to ros: " << ec.message() << std::endl;
      return;
    }
}

void
RosBridgeProxyProcessor::proxy_on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg)
{

    MutexLocker ml(mutex_);

    websocketpp::lib::shared_ptr<WebSession>  session;
    bool found =false; 


    std::string jsonString = msg -> get_payload();

    for (it_proxy_= session_proxy_map_.begin()
        ;it_proxy_!= sessoin_proxy_map_.end();
        it_proxy_++)
    {
      if (it_proxy_->second == hdl) 
      {
        session = it_proxy_->first;
        found = true;
        break;
      }
    }

    if(!found)
    {
        // throw excption. The proxy does not have a session
    }
  

    try{
        session -> send(jsonString);        
    }
    catch(fawkes::Exception &e)
    {
        logger_ -> log_error("Webtools-Bridge",e);
    }

}

//handles session termination
void
RosBridgeProxyProcessor::callback  ( EventType event_type , std::shared_ptr <EventEmitter> event_emitter) 
{
    try{
        //check if the event emitter was a session
        std::shared_ptr <WebSession> session;
        session = std::dynamic_pointer_cast<WebSession> (event_emitter);
        if(session != NULL)
        {
            if(event_type == EventType::TERMINATE )
            {
                //make sure the session is still there and was not deleted while waiting for the mutex
                if (session_proxy_map_.find(session) != session_proxy_map_.end())
                {
                    rosbridge_endpoint_.close( session_proxy_map_[session] , websocketpp::close::status::going_away, "", ec);
                    session_proxy_map_.erase(session);
                }

                std::cout<< "Session terminated NICELY :D" << std::endl;
            }
        }
        
    }
    catch(Exception &e){
        //if exception was fired it only means that the casting failed becasue the emitter is not a session
    }
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
    //check if it was a new session
    //serialize msg
    //send it to proxy with the given session
}

void 
RosBridgeProxyProcessor::unsubscribe ( std::string id
                                            , std::shared_ptr<Subscription> 
                                           , std::shared_ptr<WebSession> session ) 
{

}
