#include "capability_manager.h"

#include "callable.h"
#include "event_type.h"
#include "event_emitter.h"

#include <thread>

class Subscription;
class SubscriptionCapability;
class EventHandter;

namespace fawkes
{
	class Mutex;
}


class SubscriptionCapabilityManager
: public CapabilityManager
, public EventEmitter
, public Callable
, public std::enable_shared_from_this<SubscriptionCapabilityManager>
{
public:
	SubscriptionCapabilityManager();
	~SubscriptionCapabilityManager();

	void init();

	void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>);

	bool register_processor(std::shared_ptr <BridgeProcessor> processor);

	void callback(EventType event_type , std::shared_ptr <EventEmitter> event_emitter);

	void publish_loop();

	void emitt_event(EventType event_type);


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
	std::shared_ptr<std::thread> publisher_thread;
	bool initialized_;

	//fawkes::Mutex 			*mutex_;

};
