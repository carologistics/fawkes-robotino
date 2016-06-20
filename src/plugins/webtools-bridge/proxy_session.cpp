#include "proxy_session.h"

#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include "callable.h"
#include <exception>


using namespace fawkes;
using namespace websocketpp;

ProxySession::ProxySession(std::shared_ptr <WebSession> web_session )
: EventEmitter()
, web_session_(web_session)
, status_("connecting")
{
	mutex_=new fawkes::Mutex();
}

ProxySession::~ProxySession()
{
    delete mutex_;
}

void
ProxySession::terminate()
{
    std::shared_ptr <ProxySession> me= shared_from_this();
 //   web_session_->unregister_callback(EventType::TERMINATE , shared_from_this() );

    //emit termination event for all listeners of this session (should delete all ptrs to this instance)
    me->emitt_event( EventType::TERMINATE );    
}


void
ProxySession::on_open( websocketpp::connection_hdl hdl , std::shared_ptr<Client> endpoint_ptr )
{
    MutexLocker ml(mutex_);
    web_session_->register_callback(EventType::TERMINATE , shared_from_this() );
	hdl_ = hdl;
    endpoint_ptr_=endpoint_ptr;
	status_="open";
}

void
ProxySession::on_fail(websocketpp::connection_hdl hdl)
{
	MutexLocker ml(mutex_);
	status_="failed";
	terminate();
	//TODO:do something about the fail
}

/* It Notifies anyone that registered for the session termination by directly calling their registed callback with the session ptr.
 */
void ProxySession::on_close(websocketpp::connection_hdl hdl)
{   
 	MutexLocker ml(mutex_);

    std::cout << " on close " << std::endl;

 	status_= "closed";
 	terminate();
}

void
ProxySession::on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg)
{
    MutexLocker ml(mutex_);
    if(status_!="open")
    {
        return;
        //throw
    }
    std::string jsonString = msg -> get_payload();  

    //maybe check for its validity before
    web_session_-> send(jsonString);        
}

//TODO::catch exceptions and print in the log
bool 
ProxySession::send(std::string const & msg){

	//MutexLocker ml(mutex_);
    if(status_!="open")
    {
        return false;
        //throw
    }

	websocketpp::lib::error_code ec;

    std::cout << ">TO Proxy::sending message: "<<msg << std::endl;

    endpoint_ptr_->send(hdl_, msg, websocketpp::frame::opcode::text, ec);

    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

void
ProxySession::set_id(int id)
{
	session_id_=id;
}

int 
ProxySession::get_id()
{
	return session_id_;
}

std::string 
ProxySession::get_status()
{
    MutexLocker ml(mutex_);
	return status_;
}

websocketpp::connection_hdl
ProxySession::get_connection_hdl()
{
	return hdl_;
}

std::shared_ptr <WebSession>
ProxySession::get_web_session()
{
	return web_session_;
}

void  
ProxySession::callback( EventType event_type , std::shared_ptr <EventEmitter> event_emitter)
{

    MutexLocker ml(mutex_);

    try{
        //check if the event emitter was a web_session
        std::shared_ptr <WebSession> session = std::dynamic_pointer_cast<WebSession> (event_emitter);
        if(session != NULL)
        {
            if(event_type == EventType::TERMINATE )
            {
                websocketpp::lib::error_code ec;
            	endpoint_ptr_->close( hdl_ , websocketpp::close::status::going_away, "", ec);

                if (ec)
                {
                std::cout << "ProxySession:cloud not close connection to rosbridge "<< std::endl;
		            //logger_->log_info("ProxySession","cloud not close connection to rosbridge: %s" , ec.message().c_str());
		            return;
		        }
		     }
        }
    }
    catch(Exception &e){
        //if exception was fired it only means that the casting failed becasue the emitter is not a session
    }
} 

void
ProxySession::emitt_event(EventType event_type)
{
	for(it_callables_  = callbacks_ [event_type].begin();
		it_callables_ != callbacks_ [event_type].end() ; 
		it_callables_++)
	{
		(*it_callables_)->callback(event_type , shared_from_this());

        if(event_type == EventType::TERMINATE)
        {
            it_callables_ = callbacks_ [event_type].erase(it_callables_);   
        }
        else
        {
            it_callables_++;
        }
	}


}
