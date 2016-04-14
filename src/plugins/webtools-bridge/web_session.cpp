#include "web_session.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include "callable.h"
#include <exception>


using namespace fawkes;

WebSession::WebSession()
: EventEmitter()
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
WebSession::set_endpoint(std::shared_ptr<server> endpoint_ptr)
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
    std::shared_ptr <WebSession> me= shared_from_this();

	me->emitt_event( EventType::TERMINATE );
}

void
WebSession::emitt_event(EventType event_type)
{
	for(it_callables_  = callbacks_ [event_type].begin();
		it_callables_ != callbacks_ [event_type].end() ; 
		)
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
