#include <string>

typedef enum {
		BLACKBOARD_BRDIGE,
		ROS_BRIDGE
	  } bridgeType ;

class Ibridge{

public:
	bridgeType type;

	virtual bool init()=0;

	virtual void process_request(std::string json_str)=0;

};
