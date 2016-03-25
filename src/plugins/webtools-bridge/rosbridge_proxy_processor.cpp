
#include "rosbridge_proxy_processor.h"
#include "web_session.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>
#include <logging/logger.h>

/*This acts as a proxy to RosBridge. Each session has a uniques proxy inatance where 
the incoming and outgoing intercations are uniquly mapped 
*/
using namespace fawkes;
using namespace websocketpp;

RosBridgeProxyProcessor::RosBridgeProxyProcessor(std::string prefix , fawkes::Logger *logger , fawkes::Clock *clock)
:   BridgeProcessor(prefix)
,   logger_(logger)
,   clock_(clock)
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

    for (it_pears_ = pears_map_.begin();
         it_pears_ != pears_map_.end() ; it_pears_++)
    {
        rosbridge_endpoint_.close(it_pears_->first , websocketpp::close::status::going_away, "", ec);
        it_pears_->second->register_callback(EventType::TERMINATE , shared_from_this() );
        if (ec) 
        {
       //   std::cout << "> Error closing connection " << it_pears_->second << ": "  
         //           << ec.message() << std::endl;
        }
    }
   
    proxy_thread_->join();

}

void 
RosBridgeProxyProcessor::init()
{
    proxy_thread_ = websocketpp::lib::make_shared<websocketpp::lib::thread>(&websocketpp::client<websocketpp::config::asio_client> ::run, &rosbridge_endpoint_);  
}


//Creats a new webskocet connection to RosBridge and maps it to the new_session 
void 
RosBridgeProxyProcessor::create_rb_connection(std::shared_ptr <WebSession> new_session)
{
    init();
    
    //create a proxy client connected to RosBirdge
    std::string const & uri="ws://localhost:9090";//store it as a conf
    
    websocketpp::lib::error_code ec;

    websocketpp::client<websocketpp::config::asio_client>::connection_ptr con;
    con = rosbridge_endpoint_.get_connection(uri, ec);
    if (ec)
    {
        logger_->log_info("RosBridgeProxyProcessor"," Connect initialization error: %s" , ec.message().c_str());
        return;
    }

    con->set_message_handler(websocketpp::lib::bind(
        &RosBridgeProxyProcessor::rb_on_message,
        this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2));
    rosbridge_endpoint_.connect(con);

    //register for session's termination events
    new_session->register_callback(EventType::TERMINATE , shared_from_this() );
   
    //store the mapping <session , rosBridge_client>
    pears_map_[con->get_handle()]= new_session ;

    logger_->log_info("RosBridgeProxyProcessor"," new_connection created" , ec.message().c_str());


}



void
RosBridgeProxyProcessor::close_rb_connection(std::shared_ptr <WebSession> session)
{
    //get the RosProxy connections_hdl for this session
    it_pears_ = std::find_if(pears_map_.begin(), pears_map_.end(), 
        [&](const std::pair< connection_hdl , std::shared_ptr <WebSession>>  & v){return v.second.get() == it_pears_->second.get();});

    if( it_pears_ != pears_map_.end())
    {
        websocketpp::lib::error_code ec;
        rosbridge_endpoint_.close(it_pears_->first , websocketpp::close::status::going_away, "", ec);
        if (ec)
        {
            logger_->log_info("RosBridgeProxyProcessor","cloud not close connection to rosbridge: %s" , ec.message().c_str());
            return;
        }
      
        //unregister from termination events
        session->unregister_callback(EventType::TERMINATE , shared_from_this() );
        pears_map_.erase(it_pears_);

        logger_->log_info("RosBridgeProxyProcessor","connection closed" );
    }
}

void
RosBridgeProxyProcessor::rb_send(std::shared_ptr <WebSession> session , std::string message)
{
    it_pears_ = std::find_if(pears_map_.begin(), pears_map_.end(), 
        [&](const std::pair< connection_hdl , std::shared_ptr <WebSession>>  & v){return v.second.get() == it_pears_->second.get();});

     //send over the rosBridge_client corresponding to the session 
    if (it_pears_  == pears_map_.end())
    {
        //try
        create_rb_connection(session);
    }

    websocketpp::lib::error_code ec;
    rosbridge_endpoint_.send(it_pears_->first, message, websocketpp::frame::opcode::text, ec);
    if (ec)
    {
        logger_->log_info("RosBridgeProxyProcessor","cloud not send: %s" , ec.message().c_str());
        // std::cout << "> Error sending message to ros: " << ec.message() << std::endl;
        return;
    }
}

void
RosBridgeProxyProcessor::rb_on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg)
{
    //MutexLocker ml(mutex_);
    std::string jsonString = msg -> get_payload();  
    if(pears_map_.find(hdl) == pears_map_.end())
    {
        // throw excption. The proxy does not have a session
    }

    pears_map_[hdl]-> send(jsonString);        
}

//handles session termination
void
RosBridgeProxyProcessor::callback  ( EventType event_type , std::shared_ptr <EventEmitter> event_emitter) 
{
    //mutext lock
    try{
        //check if the event emitter was a session
        std::shared_ptr <WebSession> session;
        session = std::dynamic_pointer_cast<WebSession> (event_emitter);
        if(session != NULL)
        {
            if(event_type == EventType::TERMINATE )
            {
                //get the RosProxy connections_hdl for this session
                it_pears_ = std::find_if(pears_map_.begin(), pears_map_.end(), 
                        [&](const std::pair< connection_hdl , std::shared_ptr <WebSession>>  & v){return v.second.get() == it_pears_->second.get();});

                //make sure the session is still there and was not deleted while waiting for the mutex
                if (it_pears_ != pears_map_.end())
                {
                    websocketpp::lib::error_code ec;
                    rosbridge_endpoint_.close( it_pears_->first , websocketpp::close::status::going_away, "", ec);
                    if (ec)
                    {
                        // std::cout << "> Error sending message to ros: " << ec.message() << std::endl;
                        return;
                    }

                    pears_map_.erase(it_pears_);
                }

                //std::cout<< "Session terminated NICELY :D" << std::endl;
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
    //Mutex.lock

    std::string jsonMsg="";
    //serialize msg

    rb_send(session , jsonMsg);

    //Make a starndered Subscribtion instace (for bookkeeping and desgin consistancy)
    std::shared_ptr <Subscription> new_subscirption;
      //create a DORMANT subscription instance 
    try{
        new_subscirption = std::make_shared <Subscription>(topic_name , prefix_, clock_);
    }
    catch (fawkes::Exception &e) 
    {
        logger_->log_info("Processor:" , "Failed to subscribe to '%s': %s\n", topic_name.c_str(), e.what());
        throw e;
    }


    new_subscirption->add_request(id , compression , throttle_rate  
                                           , queue_length , fragment_size , session);

    return new_subscirption ;

}

void 
RosBridgeProxyProcessor::unsubscribe ( std::string id
                                            , std::shared_ptr<Subscription> 
                                           , std::shared_ptr<WebSession> session ) 
{
}
