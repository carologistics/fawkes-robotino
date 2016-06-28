#include "proxy_session.h"

#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include "callable.h"
#include <exception>


using namespace fawkes;
using namespace websocketpp;

ProxySession::ProxySession()
: status_("connecting")
{
    web_session_available = false;
    std::shared_ptr <WebSession> dumy_session = websocketpp::lib::make_shared<WebSession>();
	mutex_ = new fawkes::Mutex();
}

ProxySession::~ProxySession()
{
    delete mutex_;
    web_session_.reset();
}

// void
// ProxySession::terminate()
// {
//     std::shared_ptr <ProxySession> me= shared_from_this();
//  //   web_session_->unregister_callback(EventType::TERMINATE , shared_from_this() );

//     //emit termination event for all listeners of this session (should delete all ptrs to this instance)
//     me->emitt_event( EventType::TERMINATE );    
//}


// void
// ProxySession::on_open( websocketpp::connection_hdl hdl , std::shared_ptr<Client> endpoint_ptr )
// {
//     MutexLocker ml(mutex_);
//     web_session_->register_callback(EventType::TERMINATE , shared_from_this() );
// 	hdl_ = hdl;
//     endpoint_ptr_=endpoint_ptr;
// 	status_="open";
// }

// void
// ProxySession::on_fail(websocketpp::connection_hdl hdl)
// {
// 	MutexLocker ml(mutex_);
// 	status_="failed";
// 	terminate();
// 	//TODO:do something about the fail
// }

// /* It Notifies anyone that registered for the session termination by directly calling their registed callback with the session ptr.
//  */
// void ProxySession::on_close(websocketpp::connection_hdl hdl)
// {   
//  	MutexLocker ml(mutex_);

//     //std::cout << " on close " << std::endl;

//  	status_= "closed";
//  	terminate();
//}

void
ProxySession::on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg)
{
    MutexLocker ml(mutex_);
    if(status_!="open" && !web_session_available)
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

    //std::cout << ">TO Proxy::sending message: "<<msg << std::endl;

    endpoint_ptr_->send(hdl_, msg, websocketpp::frame::opcode::text, ec);

    if (ec) {
        //std::cout << "> Error sending message: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

void
ProxySession::set_id(int id)
{
    MutexLocker ml(mutex_);
	session_id_=id;
}

void
ProxySession::set_connection_hdl(websocketpp::connection_hdl hdl)
{
    MutexLocker ml(mutex_);
    hdl_=hdl;
}

void 
ProxySession::set_endpoint(std::shared_ptr<Client> endpoint_ptr)
{
    MutexLocker ml(mutex_);
    endpoint_ptr_=endpoint_ptr;
}

void
ProxySession::set_status(std::string status)
{
    MutexLocker ml(mutex_);
    status_ = status ;
}

int 
ProxySession::get_id()
{
    MutexLocker ml(mutex_);
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
    MutexLocker ml(mutex_);
	return hdl_;
}

std::shared_ptr <WebSession>
ProxySession::get_web_session()
{
    MutexLocker ml(mutex_);
	return web_session_;
}

void 
ProxySession::register_web_session( std::shared_ptr <WebSession> web_session )
{
    MutexLocker ml(mutex_);
    web_session_ = web_session;
    web_session_available = true;
}

void
ProxySession::unregister_web_session()
{
    MutexLocker ml(mutex_);
    web_session_available = false;
    std::shared_ptr <WebSession> dumy_session = websocketpp::lib::make_shared<WebSession>();
    web_session_ = dumy_session;
}




