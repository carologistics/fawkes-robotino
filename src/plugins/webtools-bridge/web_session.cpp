#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include "subscription_capability.h"

using namespace fawkes;

WebSession::WebSession()
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

/** This session has been closed from the server.
 * This method is called whenever the session is closed by the server. 
 * It Notifies anyone that registered for the session termination by directly calling their registed callback with the session ptr.
 */
void WebSession::terminate()
{

	for(std::vector<Callback>::iterator 
		it = terminate_callbacks_.begin();
		it != terminate_callbacks_.end() ; 
		it++)
	{
		Callback terminate_callback = (*it);
		terminate_callback(shared_from_this());
	}

	terminate_callbacks_.clear();
}


/** An object wants to be notified with termination events.
 * This is called by any object that wants to keep track of the session termination (usually a CapabilityObject).
 * The object send its termination callback to be called when session terminats but WebSession::terminate()
 * @param terminate_callback The Object's method that will be called ,binded as a boost::function
 *  ex[boost::bind(Subscription::on_terminate_session,this,::_1))
 */
void
WebSession::register_terminate_callback(Callback terminate_callback)
{
	//lock_mutex
	terminate_callbacks_.push_back(terminate_callback);
	//unlock_mutex
}

void
WebSession::deregister_terminate_callback( void (Subscription::*callback)(std::shared_ptr<WebSession>) )
{
	terminate_callbacks_.erase(std::find(terminate_callbacks_.begin(), terminate_callbacks_.end(), callback));
}