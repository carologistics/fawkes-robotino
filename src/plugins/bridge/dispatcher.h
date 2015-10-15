
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_


#include <string>
#include <ros_endpoint.cpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>


using websocketpp::connection_hdl;

class ros_endpoint;
class Isession;

class Dispatcher
{

private:
	typedef std::map<const int,ros_endpoint::ptr> ros_list;
	bool					rosbridge_started_;

    ros_list m_ros_list;

    ros_endpoint::ptr 		rosbridge_ptr_ ;

     websocketpp::lib::shared_ptr<Isession>	web_session_;
public:

	Dispatcher(websocketpp::lib::shared_ptr<Isession> web_s);
	~Dispatcher();

	void web_configure_session();
	void web_on_message(connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg);
	//todo::replace the big type names in a name space and use it

	bool web_forward_message(std::string msg);

	//void run();
	//void init_rosbridge(ros_endpoint::ptr rosbridge_ptr);
	void register_ros_endpoint(ros_endpoint::ptr rosbridge_ptr);
	//void register_web_endpoint( websocketpp::lib::shared_ptr<Iendpoint> web_endpoint_ptr);
	
	void reply(std::string msg);
	bool bridges_ready();

	void dispatch_msg(std::string msg);

};


#endif


