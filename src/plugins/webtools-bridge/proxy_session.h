#ifndef _PROXY_SESSION_H
#define _PROXY_SESSION_H

#include <memory>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>

#include "callable.h"
#include "event_emitter.h"
#include "event_type.h"

//TODO::move to commonNameSpace
typedef websocketpp::client<websocketpp::config::asio_client> Client;

namespace fawkes {
  class Logger;
  class Mutex;
}

class WebSession;

class ProxySession 
: public EventEmitter
, public Callable
, public std::enable_shared_from_this<ProxySession>
{

public:
	ProxySession( std::shared_ptr <WebSession> web_session);
	~ProxySession();

	void terminate();

	void on_open(websocketpp::connection_hdl hdl , std::shared_ptr<Client> endpoint_ptr );
	void on_fail(websocketpp::connection_hdl hdl);
	void on_close(websocketpp::connection_hdl hdl);//this will be called when session is closed from server
	void on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg);
	bool send( std::string const & msg);

	void 							set_id(int id);
	int 							get_id();
	std::string 					get_status();
	websocketpp::connection_hdl 	get_connection_hdl();
	std::shared_ptr <WebSession> 	get_web_session();

	void 						callback( EventType event_type , std::shared_ptr <EventEmitter> handler);
	void 						emitt_event(EventType event_type);
	
	
private:
	//The websocket client session where all interactions to be mapped
    std::shared_ptr <WebSession> 						web_session_;

    websocketpp::connection_hdl                			hdl_;
    std::shared_ptr<Client>       						endpoint_ptr_;

    std::string										 	status_;
    int                                        			session_id_;

    fawkes::Mutex 										*mutex_;	

};

#endif