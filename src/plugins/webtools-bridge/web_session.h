#ifndef _WEB_SESSION_H
#define _WEB_SESSION_H

#include <map>
#include <memory>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <boost/function.hpp>

class WebSession;
//TODO::move to commonNameSpace
typedef websocketpp::server<websocketpp::config::asio> server;
typedef boost::function < void (std::shared_ptr<WebSession>) > handler; 


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

	void						terminate();//this will be called when session is closed from server
	
	void 						register_terminate_callback(handler terminate_callback);

    std::map<std::string,std::string>					http_req;
    
private:
    websocketpp::connection_hdl                			hdl_;
    websocketpp::lib::shared_ptr<server>       		 	endpoint_ptr_;

    std::string                              			session_name_;
    std::string										 	status_;
    int                                        			session_id_;

    std::vector<handler> 						terminate_callbacks_;
};

#endif