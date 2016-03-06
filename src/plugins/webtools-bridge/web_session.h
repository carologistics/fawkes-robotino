#ifndef _WEB_SESSION_H
#define _WEB_SESSION_H

#include <map>
#include <list>
#include <memory>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>


namespace fawkes{
	class Mutex;
}


class WebSession;
//TODO::move to commonNameSpace
typedef websocketpp::server<websocketpp::config::asio> server;

class SessionListener;

class WebSession 
: public std::enable_shared_from_this<WebSession>
{

public:
	WebSession();
	~WebSession();

	void 						set_connection_hdl(websocketpp::connection_hdl hdl);
	void 						set_endpoint(websocketpp::lib::shared_ptr<server> endpoint_ptr);
	void 						set_id(int id);
	void						set_name(std::string name);
	void 						set_status(std::string status);

	server::connection_ptr 		get_connection_ptr();
	int 						get_id();
	std::string 				get_name();
	std::string 				get_status();
	
	bool 						send(std::string msg);

	

    std::map<std::string,std::string>					http_req;

	//==Session Event handlers
	//TODO::Move to a seperate class
    enum Event { TERMINATE } ;
	void						on_terminate();//this will be called when session is closed from server
	
	void 						register_callback(Event event ,std::shared_ptr <SessionListener> callback_hdl );
	void 						unregister_callback(Event event ,std::shared_ptr <SessionListener> callback_hdl); 
    
private:
    websocketpp::connection_hdl                			hdl_;
    websocketpp::lib::shared_ptr<server>       		 	endpoint_ptr_;

    std::string                              			session_name_;
    std::string										 	status_;
    int                                        			session_id_;

    fawkes::Mutex 										*mutex_;	

    std::map<Event,std::list<std::shared_ptr <SessionListener> >>			callbacks_;
    std::map<Event,std::list<std::shared_ptr <SessionListener> >>::iterator	it_callbacks_;
    std::list<std::shared_ptr <SessionListener> >::iterator					it_handlers_;
};

#endif