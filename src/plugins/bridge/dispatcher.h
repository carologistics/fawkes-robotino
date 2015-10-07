
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_


#include <string>
#include "ros_endpoint.cpp"
#include <iendpoint.h>

class ros_endpoint;

class Dispatcher
{

private:
	typedef std::map<const int,ros_endpoint::ptr> ros_list;
	bool					rosbridge_started_;

    ros_list m_ros_list;

    ros_endpoint::ptr 		rosbridge_ptr_ ;

     websocketpp::lib::shared_ptr<Iendpoint>	web_endpoint_ptr_;
public:

	Dispatcher();
	~Dispatcher();

	//void run();
	//void init_rosbridge(ros_endpoint::ptr rosbridge_ptr);
	void register_ros_endpoint(ros_endpoint::ptr rosbridge_ptr);
	void register_web_endpoint( websocketpp::lib::shared_ptr<Iendpoint> web_endpoint_ptr);
	
	void reply(std::string msg);
	bool bridges_ready();

	void dispatch_msg(std::string msg);

};


#endif


