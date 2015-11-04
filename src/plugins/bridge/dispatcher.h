
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_

#include <map>
#include <string>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <ros_proxy.cpp>

using websocketpp::connection_hdl;

class ros_proxy;
class Isession;

class Dispatcher : public Idispatcher, public std::enable_shared_from_this<Dispatcher>
{

private:
	typedef std::map<bridgeType, websocketpp::lib::shared_ptr<GenericBridge> > bridgesList;

	bool					rosbridge_started_;
	ros_proxy::ptr 				rosbridge_ptr_ ;
	websocketpp::lib::shared_ptr<Isession>	web_session_;// public to be able to get the hdl later
	bridgesList bridges_;



public:


	Dispatcher(websocketpp::lib::shared_ptr<Isession> web_s);
	~Dispatcher();
	void start();
	void init_rosbridge();
	void register_bridge();

	void web_register_handler();
	void web_on_message(connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg);
	//todo::replace the big type names in a name space and use it
	bool web_forward_message(std::string msg);

	bridgeType dispatch(std::string);

};


#endif


