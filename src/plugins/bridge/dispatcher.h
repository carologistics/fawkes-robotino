
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_


#include <string>
#include "ros_endpoint.cpp"

class ros_endpoint;

class Dispatcher
{

private:
	typedef std::map<const int,ros_endpoint::ptr> ros_list;
	bool					rosbridge_up_;

    ros_list m_ros_list;


    ros_endpoint::ptr rosbridge_ptr_ ;

public:

	Dispatcher();
	~Dispatcher();

	void init(ros_endpoint::ptr rosbridge_ptr);
	bool bridges_ready();

	void dispatch_msg();

};

#endif


