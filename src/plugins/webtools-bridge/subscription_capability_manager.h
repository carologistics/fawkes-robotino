#include "capability_manager.h"


class Subscription;

class SubscriptionCapability;

class SubscriptionCapabilityManager
: public CapabilityManager
{
public:
	SubscriptionCapabilityManager();
	~SubscriptionCapabilityManager();

	void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>);

	bool register_processor(std::shared_ptr <BridgeProcessor> processor);


private:
	void subscribe 	( std::string bridge_prefix
					, std::string topic_name 
					, std::string id 		
					, std::string compression
					, unsigned int throttle_rate	
					, unsigned int queue_length 	
					, unsigned int fragment_size 	
					, std::shared_ptr<WebSession> session);
	
	void unsubscribe( std::string bridge_prefix
					, std::string topic_name 
					, std::string id 		
					, std::shared_ptr<WebSession> session);

	std::map <std::string,std::shared_ptr<Subscription> > topic_Subscription_;
};
