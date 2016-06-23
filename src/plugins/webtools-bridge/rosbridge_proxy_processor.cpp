
#include "rosbridge_proxy_processor.h"
#include "web_session.h"
#include "proxy_session.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>
#include <config/config.h>
#include <logging/logger.h>

#include "serializer.h"

/*This acts as a proxy_server to RosBridge. For each web_session, a session to "RosBridge Server" is created to act a proxy
forwarding all incoming and outgoing intercations.*/

using namespace websocketpp;
using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using namespace fawkes;

RosBridgeProxyProcessor::RosBridgeProxyProcessor(std::string prefix , fawkes::Logger *logger , fawkes::Configuration *config , fawkes::Clock *clock)
:   BridgeProcessor(prefix)
,   logger_(logger)
,   clock_(clock)
{
    config_= config;
    rosbridge_uri_= config_->get_string("/webtools-bridge/rosbridge-uri");

    rosbridge_endpoint_=websocketpp::lib::make_shared<Client>();
    rosbridge_endpoint_->clear_access_channels(websocketpp::log::alevel::all);
    rosbridge_endpoint_->clear_error_channels(websocketpp::log::elevel::all);
    rosbridge_endpoint_->init_asio();
    rosbridge_endpoint_->start_perpetual();

    mutex_=new fawkes::Mutex();
}

RosBridgeProxyProcessor::~RosBridgeProxyProcessor()
{
    if (!finalized_) { finalize(); }

    rosbridge_endpoint_-> stop_perpetual();

    if(!rosbridge_endpoint_ -> stopped())
    {
        rosbridge_endpoint_ -> stop();
        usleep(100000);
        logger_->log_info("RosBridgeProxyProcessor", "Stopping Asio");
    } 
    
    if(proxy_thread_ != NULL ) proxy_thread_->join();

    MutexLocker ml(mutex_);  
    while(!peers_.empty())
    {
        logger_->log_info("ProxyServer", "force deleting sessions");
        ( *peers_.begin() ) -> unregister_web_session();
        peers_.erase(peers_.begin());
    } 

    rosbridge_endpoint_.reset();

    delete mutex_;
}

void 
RosBridgeProxyProcessor::init()
{
    if (!initialized_)
    { 
        proxy_thread_ = websocketpp::lib::make_shared<websocketpp::lib::thread>(&websocketpp::client<websocketpp::config::asio_client> ::run, rosbridge_endpoint_);  
        logger_->log_info("RosBridgeProxyProcessor"," Proxy Thread Started" );

        BridgeProcessor::init();
    }
}

void 
RosBridgeProxyProcessor::finalize()
{
    MutexLocker ml(mutex_);

    if(!finalized_)
    {
        websocketpp::lib::error_code ec;
        logger_->log_info( "RosBridgeProxyProcessor:"," Finalizing" ) ;

        for (  it_peers_ = peers_.begin()
            ; it_peers_ != peers_.end()
            ; it_peers_++ )
        {
            ( *it_peers_ ) -> unregister_web_session();
            rosbridge_endpoint_->close((*it_peers_)->get_connection_hdl() , websocketpp::close::status::going_away, "", ec);
            
            if (ec) 
            {
               logger_->log_info("RosBridgeProxyProcessor"," Error closing connection : : %s" , ec.message().c_str());
            }
        }

        BridgeProcessor::finalize();
    }

}

/*to thing about: What if this asiorun thread wants to terminate before the other webserver is already finished..
is that even possible....
why not move the web_Session terminatoin to the finilization from the ditrctor....
because u need to make sure the session was joined right!
*/

void 
RosBridgeProxyProcessor::on_open( std::shared_ptr<WebSession> web_session , connection_hdl hdl  ) 
{

    MutexLocker ml(mutex_);

    if(!finalized_)
    {
        std::shared_ptr  <ProxySession> rosbridge_session = std::make_shared<ProxySession>();

        //Register the termination call_back to RosBridgeProxyProcessor::call_back and terminate from here 
        
        web_session -> register_callback(EventType::TERMINATE , shared_from_this() ); //Still a big question
         //The call_back should find the corresponding  Proxy and terminates it

        rosbridge_session -> set_status( "open" ) ;
        rosbridge_session -> set_connection_hdl( hdl ) ;
        rosbridge_session -> set_endpoint( rosbridge_endpoint_ ) ;
        rosbridge_session -> register_web_session( web_session ) ;

        Client::connection_ptr con = rosbridge_endpoint_ -> get_con_from_hdl(hdl);
        //Add server statelater: m_server = con->get_response_header("Server");
        con->set_message_handler(websocketpp::lib::bind(
            &ProxySession::on_message,
            rosbridge_session,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2));

        peers_.push_back(rosbridge_session); 
        logger_->log_info("RosBridgeProxyProcessor"," new connection to rosbirdge established" );        
    }
}

void
RosBridgeProxyProcessor::on_fail(connection_hdl hdl) 
{
    MutexLocker ml(mutex_);
    //Just say that the session failed..There is no session created at this point
    //The time out should take care of that if failed..
}

void
RosBridgeProxyProcessor::on_close(connection_hdl hdl) 
{
    MutexLocker ml(mutex_);
    
    it_peers_ = std::find_if(peers_.begin(), peers_.end(), 
            [&]( const std::shared_ptr <ProxySession>  & v)
            {return v -> get_connection_hdl().lock()  == hdl.lock() ;});

    if (it_peers_ != peers_.end())
    {
        ( *it_peers_ ) -> unregister_web_session();
        ( *it_peers_ ) ->  set_status( "closed" ) ;

        //( *it_peers_ ) -> terminate();
        peers_ . erase( it_peers_ );
        logger_->log_info("RosBridgeProxyProcessor"," proxy session deleted" );
    }
    
    std::cout<< "Proxy Session closed NICELY :D" << std::endl;        
}

void
RosBridgeProxyProcessor::connect_to_rosbridge( std::shared_ptr <WebSession> web_session )
{   
   //MutexLocker ml(mutex_); will only be called from a mutexed_ threadh
    //Is there an existing proxy_session for this web_session
    it_peers_ = std::find_if(peers_.begin(), peers_.end(), 
            [&](const std::shared_ptr <ProxySession>  & v)
            {return v->get_web_session().get() == web_session.get();});
     
    if (it_peers_  == peers_.end())
    {
        //create a proxy_session to act as a client to the "Ros Bridge Server". 
        
        websocketpp::lib::error_code ec;

        websocketpp::client<websocketpp::config::asio_client>::connection_ptr con;
        con = rosbridge_endpoint_->get_connection( rosbridge_uri_ , ec);
        if (ec)
        {
            logger_->log_info("RosBridgeProxyProcessor"," Connect initialization error: %s" , ec.message().c_str());
            return;
            //throw
        }

        con->set_open_handler(websocketpp::lib::bind(
                                                        &RosBridgeProxyProcessor::on_open
                                                        , this
                                                        , web_session                        
                                                        , websocketpp::lib::placeholders::_1 ) );
        con->set_close_handler(websocketpp::lib::bind(
                                                        &RosBridgeProxyProcessor::on_close
                                                        , this
                                                        , websocketpp::lib::placeholders::_1 ) );
        con->set_fail_handler(websocketpp::lib::bind(
                                                        &RosBridgeProxyProcessor::on_fail
                                                        , this
                                                        , websocketpp::lib::placeholders::_1 ) );

        rosbridge_endpoint_->connect(con);

    }

}


/*Forward the incoming request comming from the web_session to RosBridge using the corresponding proxy_session
 proxy_session exists only if a connection was established. If not  busy wait till timeout is reached */
void
RosBridgeProxyProcessor::process_request(std::shared_ptr <WebSession> web_session , std::string jsonMsg)
{
 
    MutexLocker ml(mutex_);
 
    const int time_out = 300000; //time out in ms. TODO: Move to config 
    it_peers_ = std::find_if(peers_.begin(), peers_.end(), 
                [&](const std::shared_ptr <ProxySession>  & v)
                {return v->get_web_session().get() == web_session.get();});
    
    if( it_peers_  == peers_.end() )
    {
        connect_to_rosbridge ( web_session );
        

        fawkes::Time now(clock_);
        fawkes::Time time_out_start(clock_);
        bool         time_out_reached = false;
        
        do
        {//Give the connections thread time to try to connect
         
            ml.unlock();
            usleep( (int) time_out / 4 );
            MutexLocker ml(mutex_);
    
            it_peers_ = std::find_if(peers_.begin(), peers_.end(), 
                    [&](const std::shared_ptr <ProxySession>  & v)
                    {return v->get_web_session().get() == web_session.get();});

            now.stamp();
            time_out_reached = (now.in_msec() - time_out_start.in_msec() >= time_out);
            
        } while ( it_peers_  == peers_.end() || time_out_reached );

        if( time_out_reached )
        {
            logger_->log_warn("RosBridgeProxyProcessor"," could not connect to RosBridge. Connection timed out!" );
            return;
        }
    }

    if(it_peers_  != peers_.end() )
    {
        if( (*it_peers_) -> get_status() != "open" )
        {
            logger_->log_warn("RosBridgeProxyProcessor"," connected but session is unknown state" );
            return;
        }

        (*it_peers_)->send (jsonMsg);
    }
    else 
    {
        logger_->log_warn("RosBridgeProxyProcessor"," connected but session is not found!!" );
    }
    
}

//Handles Session termination
void
RosBridgeProxyProcessor::callback  ( EventType event_type , std::shared_ptr <EventEmitter> event_emitter) 
{
    MutexLocker ml(mutex_);

    try{
        //check if the event emitter was a session
        std::shared_ptr <WebSession> web_session;
        web_session = std::dynamic_pointer_cast<WebSession> (event_emitter);
        if(web_session != NULL)
        {
            if(event_type == EventType::TERMINATE )
            {
                it_peers_ = std::find_if(peers_.begin(), peers_.end(), 
                            [&](const std::shared_ptr <ProxySession>  & v)
                            {return v -> get_web_session().get() == web_session.get();});
                   
                if (it_peers_ != peers_.end())
                {
                     std::cout<< "removing web_peer session" << std::endl;        
                    (*it_peers_) -> unregister_web_session();

                    websocketpp::lib::error_code ec;
                    rosbridge_endpoint_ ->close( (*it_peers_) -> get_connection_hdl() , websocketpp::close::status::going_away, "", ec);

                    if (ec) 
                    {
                       logger_->log_info("RosBridgeProxyProcessor"," Error closing connection : : %s" , ec.message().c_str());
                    }
                    // peers_.erase(it_peers_);
                    // logger_->log_info("RosBridgeProxyProcessor"," proxy session deleted" );
                }
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
                                            , std::string type   
                                            , std::string compression
                                            , unsigned int throttle_rate  
                                            , unsigned int queue_length   
                                            , unsigned int fragment_size  
                                            , std::shared_ptr<WebSession> web_session)
{
    std::string jsonMsg= Serializer::op_subscribe( topic_name , id , type , compression 
                                                , throttle_rate , queue_length , fragment_size );
    //try catch
    process_request(web_session , jsonMsg);

    //create a DORMANT  Subscribtion instace (to keep consistancy with other Bridge_processors)
    std::shared_ptr <Subscription> new_subscirption;
    try{
        new_subscirption = std::make_shared <Subscription>(topic_name , prefix_, logger_ , clock_);
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
                                        , std::shared_ptr<WebSession> web_session ) 
{
    std::string jsonMsg= Serializer::op_unsubscribe( subscription->get_topic_name() , id );
    //try catch
     logger_->log_info("Processor:" , jsonMsg.c_str());
    process_request(web_session , jsonMsg);
    subscription->remove_request(id, web_session);   
}

std::shared_ptr<Advertisment> 
RosBridgeProxyProcessor::advertise  ( std::string topic_name 
                                        , std::string id    
                                        , std::string type  
                                        , std::shared_ptr<WebSession> web_session)
{
    std::string jsonMsg= Serializer::op_advertise( topic_name , id , type );
    //try catch
    process_request(web_session , jsonMsg);

    //create a DORMANT  Subscribtion instace (to keep consistancy with other Bridge_processors)
    std::shared_ptr <Advertisment> new_advertisment;
    try{
        new_advertisment = std::make_shared <Advertisment>(topic_name , prefix_);
    }
    catch (fawkes::Exception &e) 
    {
        logger_->log_info("Processor:" , "Failed to advertise to '%s':", topic_name.c_str(), e.what());
        throw e;
    }

    new_advertisment->add_request(id , web_session);

    return new_advertisment;
}
  
void                         
RosBridgeProxyProcessor::unadvertise ( std::string id
                                        , std::shared_ptr<Advertisment> advertisment
                                        , std::shared_ptr<WebSession> web_session )
{
    std::string jsonMsg= Serializer::op_unadvertise( advertisment->get_topic_name() , id );
    process_request(web_session , jsonMsg);
    advertisment->remove_request(id, web_session); 
}

void
RosBridgeProxyProcessor::publish     ( std::string id
                                        , bool latch
                                        , std::string msg_in_json
                                        , std::shared_ptr<Advertisment> advertisment
                                        , std::shared_ptr<WebSession> web_session )
{
    std::string jsonMsg= Serializer::op_publish( advertisment->get_topic_name() , id , latch , msg_in_json);
    process_request(web_session , jsonMsg);
}



void
RosBridgeProxyProcessor::call_service( std::string srv_call_json 
                                            , std::shared_ptr<WebSession> web_session )
{
    logger_->log_info("Processor:" , "calling service'%s':", srv_call_json.c_str());
    process_request(web_session , srv_call_json);   
}

