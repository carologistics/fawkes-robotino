#include "capability_manager.h"

class ServiceCapability;

class ServiceCapabilityManager
: public CapabilityManager
{
public:
	ServiceCapabilityManager();
	~ServiceCapabilityManager();

	void init();

	void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>);

	bool register_processor(std::shared_ptr <BridgeProcessor> processor);


};
