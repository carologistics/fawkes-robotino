#include "capability_manager.h"
#include "callable.h"
#include "event_type.h"

class Subscription;
class SubscriptionCapability;
class EventHandter;



class SubscriptionCapabilityManager
: public CapabilityManager
, public Callable
, public std::enable_shared_from_this<SubscriptionCapabilityManager>
{
public:
	SubscriptionCapabilityManager();
	~SubscriptionCapabilityManager();

	void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>);

	bool register_processor(std::shared_ptr <BridgeProcessor> processor);

	void callback(EventType event_type , std::shared_ptr <EventEmitter> event_emitter);

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
