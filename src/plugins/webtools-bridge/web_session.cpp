#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>


using namespace fawkes;

WebSession::WebSession()
: EventHandler()
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
	call_callbacks( EventType::TERMINATE );
}

//called automatically after events are emitted
void WebSession::do_on_event(EventType event_type){
	//Nothing for now
}