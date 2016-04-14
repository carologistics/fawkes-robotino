#include "capability_manager.h"
#include "callable.h"
#include "event_type.h"

class Advertisment;
class AdvertismentCapability;
class EventHandter;



class AdvertismentCapabilityManager
: public CapabilityManager
, public Callable
, public std::enable_shared_from_this<AdvertismentCapabilityManager>
{
public:
	AdvertismentCapabilityManager();
	~AdvertismentCapabilityManager();

	void init();

	void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>);

	bool register_processor(std::shared_ptr <BridgeProcessor> processor);

	void callback(EventType event_type , std::shared_ptr <EventEmitter> event_emitter);

private:
	void advertise	( std::string bridge_prefix
					, std::string topic_name 
					, std::string id 	
					, std::string type		
					, std::shared_ptr<WebSession> session);
	
	void unadvertise( std::string bridge_prefix
					, std::string topic_name 
					, std::string id 		
					, std::shared_ptr<WebSession> session);

	void publish	( std::string bridge_prefix
					, std::string topic_name 
					, std::string id
					, bool latch
					, std::string msg_jsonStr	
					, std::shared_ptr<WebSession> session);

	std::map <std::string,std::shared_ptr<Advertisment> > topic_Advertisment_;
};
