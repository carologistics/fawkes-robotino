#include "capability_manager.h"
#include "subscribtion_capability.h"

using namespace rapidjson;


//DECIDE..will i extend the susbscribtion capability or not...i can even make i a virtual inheritance.
//but then the capability manager also became a capability...it will implement the operations anyways
//and i can even keep the subscribtion list in the base class and be able to access it from the manager 
//and the processor...but Subscribtion then will contain all the list of any ever subscribed processor 
// at the very same place..



SubscribtionCapabilityManager::SubscribtionCapabilityManager()
:	CapabilityManager("subscribtion")
{

}

SubscribtionCapabilityManager::~SubscribetionCapabilityManager()
{
	//TODO: check if something need to be changed
}

void 
SubscriptionCapabilityManager::subscribe(	std::string bridge_prefix
										,	Document &d
										,	std::shared_ptr<WebSession> session)
{
	//TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if(!d.HasMember("topic") || !d.HasMember("id")){
		//throw missing msg feild exception
	}
	
	//TODO:: Make a Protocol Structure in the until
	if(d.HasMember("topic"))		std::string topic_name 		= 	std::string( d["topic"].GetString() );
	if(d.HasMember("id")) 			std::string id 				= 	std::string(d["id"].GetString());
	if(d.HasMember("compression")) 	std::string compression		=	std::string(d["compression"].GetString());	
	if(d.HasMember("throttle_rate"))unsigned int throttle_rate	=	d["throttle_rate"].GetUint();
	if(d.HasMember("queue_length")) unsigned int queue_length 	=	d["queue_length"].GetUint();
	if(d.HasMember("fragment_size"))unsigned int fragment_size 	=	d["fragment_size"].GetUint();

	
	try{
		//always creates a new subscriber for that topic with the given Session and parameters
		//TODO:: pass the pure string arguments or a "protocol" type
		std::shared_ptr <Subscribtion> subscriber = processores_[match_prefix]-> subscribe(Document &d, session);
	}catch(Exception){
		//TODD:: Thorw the appropriate expcetion
	}

	/*push it to the topic_subscribertion_map maintaing only ONE instance per topic
	*containing all the clients maped to thier individuale requests*/

	//Is it a new topic? Just push it to the map and activate the subscriber

	//Mutex.lock()
	if( topic_subscribtion_.find(topic_name) == topic_subscribtion_.end() )
	{
		topic_subscribtion_[topic_name] = subscriber;
		//Activate the listeners or whatever the publishs
		subscriber.activate();
	}else{
		topic_subscribtion_[topic_name].append(subscriber);
	}
	//Mutex.unlock();
}

void
SubscriptionCapabilityManager::unsubscribe(Document &d
										,	std::shared_ptr<WebSession> session)
{
	
}


bool
SubscribtionCapabilityManager::register_processor(std::shared_ptr <BridgeProcessor> processor )
{
	SubscribtionCapability *subscribtion_processor;
	subscribtion_processor = dynamic_cast<SubscribtionCapability *>(processor);
	if(bridge_processor == NULL)
	{
		return false;
	}
	//find if it was used before
	std::string processor_prefix= processor.get_prefix();
	size_t found = processores_.find(processor_prefix);
	if(found != std::string::npos)
	{
		//throw exception this prefix name is invalide coz it was used before

	}

	processores_[processor_prefix]=processor;
	
	return true;
}

void
SubscribtionCapabilityManager::handle_message(Document &d
	,	std::shared_ptr<WebSession> session)
{

	//TODO::Pass the Dom the a Protocol Class that will have the deserialized types
	try
	{
		std::string msg_op = std::string(d["op"].GetString());
	}
	catch(Exception ){
		//	"Wrong msg option"
	}
 
	std::string msg_topic = std::string(d["topic"].GetString());
	std::string match_prefix ="";

	for( ProcessorList::iterator it = processores_.begin();
		it != processores_.end(); it++)
	{
		std::string processor_prefix = it->first;
		std::size_t found_at = msg_topic.find(processor_prefix);
		if(found != std::string::npos)
		{
			//allow freedom of 2 characters before the match to account 
			//for leading "/" ot "//" or and other startting charachters
			if (found_at < 1)
			{
				if(processor_prefix.length() = match_prefix.length())
				{
					//Throw an exception. That there are 2 names with confusion prefixs
					//a conflict like "/bl" and  "bl1" when looking for "bl".
					//this cozed by the fliexibility

				}
				else if(processor_prefix.length() > match_prefix.length())
				{
					match_prefix = processor_prefix.length();
				}
			}
		} 
		
	}

	if(match_prefix.lenght() == 0)
	{
		//throw Exception: this means no processors was regiesterd for this
		return ;
	}


	//Go with the bridge name to the operation
	if(msg_op=="subscribe")
	{	
		subscribe(match_prefix, Document &d, session);
	}else 

	if (msg_op=="unsubscribe")
	}
		unsubscribe(match_prefix, Document &d, session);
	}

}
