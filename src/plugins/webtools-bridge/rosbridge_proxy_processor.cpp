
#include "rosbridge_proxy_processor.h"
#include "web_session.h"
#include "proxy_session.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>
#include <logging/logger.h>

#include "serializer.H"

/*This acts as a proxy_server to RosBridge. For each web_session, a session to "RosBridge Server" is created to act a proxy
forwarding all incoming and outgoing intercations.*/

using namespace fawkes;
using namespace websocketpp;

RosBridgeProxyProcessor::RosBridgeProxyProcessor(std::string prefix , fawkes::Logger *logger , fawkes::Clock *clock)
:   BridgeProcessor(prefix)
,   logger_(logger)
,   clock_(clock)
{
    rosbridge_endpoint_=websocketpp::lib::make_shared<Client>();
    
    rosbridge_endpoint_->clear_access_channels(websocketpp::log::alevel::all);
    rosbridge_endpoint_->clear_error_channels(websocketpp::log::elevel::all);
    rosbridge_endpoint_->init_asio();
    rosbridge_endpoint_->start_perpetual();

    mutex_=new fawkes::Mutex();
}

RosBridgeProxyProcessor::~RosBridgeProxyProcessor()
{
    rosbridge_endpoint_->stop_perpetual();

    websocketpp::lib::error_code ec;

    for (it_pears_ = pears_.begin();
         it_pears_ != pears_.end() ; it_pears_++)
    {
        (*it_pears_)->unregister_callback(EventType::TERMINATE , shared_from_this() );
        rosbridge_endpoint_->close((*it_pears_)->get_connection_hdl() , websocketpp::close::status::going_away, "", ec);
        if (ec) 
        {
           logger_->log_info("RosBridgeProxyProcessor"," Error closing connection : : %s" , ec.message().c_str());
        }
    }

    proxy_thread_->join();
    delete mutex_;
}

void 
RosBridgeProxyProcessor::init()
{
    proxy_thread_ = websocketpp::lib::make_shared<websocketpp::lib::thread>(&websocketpp::client<websocketpp::config::asio_client> ::run, rosbridge_endpoint_.get());  
    logger_->log_info("RosBridgeProxyProcessor"," Proxy Thread Started" );
}

/*creates new a session to to "Ros Bridge" acting as proxy,
 and wrappes the a unique web_session and mapes its interactions

 forward the msg incoming on the web_session to RosBridge using the corresponding proxy_session */
void
RosBridgeProxyProcessor::forward_to_proxy_session(std::shared_ptr <WebSession> web_session , std::string jsonMsg)
{
    MutexLocker ml(mutex_);
 
    //Is there an existing proxy_session for this web_session
    it_pears_ = std::find_if(pears_.begin(), pears_.end(), 
            [&](const std::shared_ptr <ProxySession>  & v)
            {return v->get_web_session().get() == web_session.get();});

    if (it_pears_  == pears_.end())
    {
        //create a proxy_session to act as a client to the "Ros Bridge Server". 
        std::string const & uri="ws://localhost:9090";//TODO::store it as a conf
        
        websocketpp::lib::error_code ec;

        websocketpp::client<websocketpp::config::asio_client>::connection_ptr con;
        con = rosbridge_endpoint_->get_connection(uri, ec);
        if (ec)
        {
            logger_->log_info("RosBridgeProxyProcessor"," Connect initialization error: %s" , ec.message().c_str());
            return;
            //throw
        }

        //Register all the proxy_session handlers to the proxy_session itslef
        std::shared_ptr  <ProxySession> rosbridge_session = std::make_shared<ProxySession>(web_session);

        con->set_open_handler(websocketpp::lib::bind(
            &ProxySession::on_open,
            rosbridge_session,
            websocketpp::lib::placeholders::_1
            ,rosbridge_endpoint_));
        con->set_close_handler(websocketpp::lib::bind(
            &ProxySession::on_close,
            rosbridge_session,
            websocketpp::lib::placeholders::_1));
        con->set_fail_handler(websocketpp::lib::bind(
            &ProxySession::on_fail,
            rosbridge_session,
            websocketpp::lib::placeholders::_1));
        con->set_message_handler(websocketpp::lib::bind(
            &ProxySession::on_message,
            rosbridge_session,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2));

        rosbridge_endpoint_->connect(con);

        //register for proxy_session's termination events
        rosbridge_session->register_callback(EventType::TERMINATE , shared_from_this() );

        pears_.push_back(rosbridge_session); 

        logger_->log_info("RosBridgeProxyProcessor"," new_connection created" , ec.message().c_str());
    }

    //TODO::replace all this with a better written version
     it_pears_ = std::find_if(pears_.begin(), pears_.end(), 
            [&](const std::shared_ptr <ProxySession>  & v)
            {return v->get_web_session().get() == web_session.get();});

    if (it_pears_  == pears_.end())
    {

        logger_->log_info("RosBridgeProxyProcessor"," still not found" );
        //throw
    }
    else
    {
        //wait for session to be ready
        while((*it_pears_)->get_status() != "open")
        {
            if((*it_pears_)->get_status() == "connecting")
            {
                //wait
            }
            else
            {
                //throw
            }
        }

        (*it_pears_)->send (jsonMsg);
    }
}

//Handles Session termination
void
RosBridgeProxyProcessor::callback  ( EventType event_type , std::shared_ptr <EventEmitter> event_emitter) 
{
    MutexLocker ml(mutex_);
    try{
        //check if the event emitter was a session
        std::shared_ptr <ProxySession> session;
        session = std::dynamic_pointer_cast<ProxySession> (event_emitter);
        if(session != NULL)
        {
            if(event_type == EventType::TERMINATE )
            {
                //get the RosProxy connections_hdl for this session
                it_pears_ = std::find_if(pears_.begin(), pears_.end(), 
                        [&](const std::shared_ptr <ProxySession>  & v){return v.get() == session.get();});

                if (it_pears_ != pears_.end())
                {
                    pears_.erase(it_pears_);
                    logger_->log_info("RosBridgeProxyProcessor"," proxy session deleted" );
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
                                            , std::shared_ptr<WebSession> web_session)
{

    std::string jsonMsg= Serializer::op_subscribe( topic_name , id , compression 
                                                , throttle_rate , queue_length , fragment_size );
    //try catch
    forward_to_proxy_session(web_session , jsonMsg);

    //create a DORMANT  Subscribtion instace (to keep consistancy with other Bridge_processors)
    std::shared_ptr <Subscription> new_subscirption;
    try{
        new_subscirption = std::make_shared <Subscription>(topic_name , prefix_, clock_);
    }
    catch (fawkes::Exception &e) 
    {
        logger_->log_info("Processor:" , "Failed to subscribe to '%s': queue_length%s\n", topic_name.c_str(), e.what());
        throw e;
    }

    new_subscirption->add_request(id , compression , throttle_rate  
                                           , queue_length , fragment_size , web_session);

    return new_subscirption ;

}

void 
RosBridgeProxyProcessor::unsubscribe ( std::string id
                                        , std::shared_ptr<Subscription> subscription
                                        , std::shared_ptr<WebSession> session ) 
{
    std::string jsonMsg= Serializer::op_subscribe( topic_name , id , compression 
                                                , throttle_rate , queue_length , fragment_size );
    //try catch
    forward_to_proxy_session(web_session , jsonMsg);

    subscription->remove_request(id, session);
    
}
