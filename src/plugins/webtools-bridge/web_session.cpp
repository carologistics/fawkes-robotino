#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

WebSession::WebSession(fawkes::Mutex *mutex)
: mutex_(mutex)
{
}

WebSession::~WebSession()
{
}

void
WebSession::set_connection_hdl(websocketpp::connection_hdl hdl)
{
	hdl_=hdl;
}

void 
WebSession::set_endpoint(websocketpp::lib::shared_ptr<server> endpoint_ptr)
{
	endpoint_ptr_=endpoint_ptr;
}

void
WebSession::set_id(int id)
{
	session_id_=id;
}

void 
WebSession::set_name(std::string name)
{
	session_name_=name;
}

void
WebSession::set_status(std::string status)
{
	status_=status;
}

int 
WebSession::get_id()
{
	return session_id_;
}

std::string 
WebSession::get_name()
{
	return session_name_;
}

std::string 
WebSession::get_status()
{
	return status_;
}

server::connection_ptr 
WebSession::get_connection_ptr()
{
	return endpoint_ptr_->get_con_from_hdl(hdl_);
}

//TODO::catch exceptions and print in the log
bool 
WebSession::send(std::string msg){

	//MutexLocker ml(mutex_);

	websocketpp::lib::error_code ec;

    std::cout << ">TO WEB::sending message: " << std::endl;

	endpoint_ptr_->send(hdl_, msg, websocketpp::frame::opcode::text, ec);

    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

//Will be called from Web_server if this session was closed.
//It makes sure the event is propagated by calling all the registered terminate callbacks
void WebSession::terminate()
{
	//mutex
	for(std::vector<handler>::iterator 
		it = terminate_callbacks_.begin();
		it != terminate_callbacks_.end() ;
		it++)
	{
		handler terminate_callback = (*it);
		terminate_callback(shared_from_this());
	}
	//unlock mutex
}


//Call from each class that stors sessions with every new session to get notified when this session is close.
//Parameter: the result of a boost::bind of session_terminate_handler ,within the storing class, to be calledback when session terminates
//eg,[boost::bind(Subscription::on_terminate_session,this,::_1)]

void
WebSession::register_terminate_callback(handler terminate_callback)
{
	//lock_mutex
	terminate_callbacks_.push_back(terminate_callback);
	//unlock_mutex
}