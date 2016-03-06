#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include "session_listener.h"

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

/** This session has been closed from the server
. * This method is called whenever the session is closed by the server. 
 * It Notifies anyone that registered for the session termination by directly calling their registed callback with the session ptr.
 */
void WebSession::on_terminate()
{

	for(it_handlers_ = callbacks_[TERMINATE].begin();
		it_handlers_ != callbacks_[TERMINATE].end() ; 
		it_handlers_++)
	{
		(*it_handlers_)->session_terminated(shared_from_this());
	}

	//Clear the whole map to be ready for termination.
	//TODO::move to Finalize() and call it
	callbacks_.clear();
}

/** An object wants to be notified with termination events.
 * This is called by any object that wants to keep track of the session termination (usually a CapabilityObject).
 * The object send its termination callback to be called when session terminats but WebSession::terminate()
 * @param terminate_callback The Object's method that will be called ,binded as a boost::function
 *  ex[boost::bind(Subscription::on_terminate_session,this,::_1))
 */
void
WebSession::register_callback(Event event, std::shared_ptr <SessionListener> callback_hdl)
{
	//TODO:: add mutex
	callbacks_[event].push_back(callback_hdl);
}

void
WebSession::unregister_callback(Event event, std::shared_ptr <SessionListener> callback_hdl )
{
	//TODO:: add mutex
	it_handlers_ = std::find_if(callbacks_[event].begin(), callbacks_[event].end(), 
		[&](std::shared_ptr<SessionListener> const& s){return s == callback_hdl;});

	if( it_handlers_ != callbacks_[event].end() ) {
		callbacks_[event].erase(it_handlers_);
	}
}