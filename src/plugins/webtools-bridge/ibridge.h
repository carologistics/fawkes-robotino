#ifndef INTERFACE_BRIDGE_H
#define INTERFACE_BRIDGE_H

#include <string>

typedef enum {
		ROS_BRIDGE
		, FAWKES_BRIDGE
	  } bridgeType ;

class Ibridge{

public:
	bridgeType type;

	virtual bool init()=0;

	virtual void process_request(std::string json_str)=0;

};



#endif
