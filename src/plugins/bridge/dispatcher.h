
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_

#include <map>
#include <string>
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

#include <logging/logger.h>

#include "ros_proxy.cpp"

//#include "generic_bridge.h"
#include "generic_bridge_manager.h"
//#include "interfaces/ibridge_manager.h"

using websocketpp::connection_hdl;

class Isession;

class Dispatcher : public Idispatcher, public std::enable_shared_from_this<Dispatcher>
{

private:
	typedef std::map<bridgeType, websocketpp::lib::shared_ptr<Ibridge> > bridgesList;
	bridgesList 			bridges_;
	bool					rosbridge_started_;

	websocketpp::lib::shared_ptr<Isession>		web_session_;// public to be able to get the hdl later
	fawkes::Logger 								*logger_;

	std::shared_ptr<GenericBridgeManager>                                  fawkes_bridge_manager_;

public:


	Dispatcher(fawkes::Logger *logger, websocketpp::lib::shared_ptr<Isession> web_s ,std::shared_ptr<GenericBridgeManager>   fawkes_bridge_manager_);
	~Dispatcher();
	void start();
	void register_bridges();

	void web_register_handler();
	void web_on_message(connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg);
	//todo::replace the big type names in a name space and use it
	bool send_to_web(std::string msg);

	bridgeType dispatch(std::string);

};


#endif


