#include <string>

typedef enum {
		BLACKBOARD_BRDIGE,
		ROS_BRIDGE
	  } bridge_type ;

class GenericBridge{

public:
	bridge_type type;

	virtual void process_request(std::string msg)=0;
	
};
