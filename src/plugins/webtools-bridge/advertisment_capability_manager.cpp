#include "advertisment_capability_manager.h"
#include "advertisment_capability.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <core/exceptions/software.h>

using namespace fawkes;
using namespace rapidjson;


AdvertismentCapabilityManager::AdvertismentCapabilityManager()	
:	CapabilityManager("Advertisment")
{

}

AdvertismentCapabilityManager::~AdvertismentCapabilityManager()
{
	//TODO: check if something needs to be changed
}

void 
AdvertismentCapabilityManager::init()
{}
	

bool
AdvertismentCapabilityManager::register_processor(std::shared_ptr <BridgeProcessor> processor )
{
	std::shared_ptr <AdvertismentCapability> Advertisment_processor;
	Advertisment_processor = std::dynamic_pointer_cast<AdvertismentCapability> (processor);
	if(Advertisment_processor == NULL)
	{
		return false;
	}
	//find if it was used before
	std::string processor_prefix= processor->get_prefix();
	
	if(processores_.find(processor_prefix) == processores_.end())
	{
		//throw exception this prefix name is invalide coz it was used before

	}

	processores_[processor_prefix]=processor;
	
	return true;
}

void
//TODO::remnam it (DispatchtoCapability)
AdvertismentCapabilityManager::handle_message(Document &d
											, std::shared_ptr<WebSession> session)
{	

	//TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if(!d.HasMember("topic") || !d.HasMember("id")){
		throw fawkes::MissingParameterException("AdvertismentCPM: Wrong Json Msg, topic name or id missing!");
	}

	//TODO::Pass the Dom the a Protocol Class that will have the deserialized types
	std::string msg_op = std::string(d["op"].GetString());

	std::string msg_topic = std::string(d["topic"].GetString());
	std::string match_prefix ="";

	//Find longest matching processor presfix within topic_name 
	//TODO:Check the logic..might be wrong
	//TODO:posible Optimization. if the same topic name exists in the topic_advertisment just get the prefix from there
	for( ProcessorMap::iterator it = processores_.begin();
		it != processores_.end(); it++)
	{
		std::string processor_prefix = it->first;
		std::size_t found_at = msg_topic.find(processor_prefix,0);

		if(found_at != std::string::npos)
		{
			//allows freedom of 2 characters before the match to account 
			//for leading "/" ot "//" or and other startting charachters
			if (found_at <= 1)
			{
				if(processor_prefix.length() == match_prefix.length())
				{
					//Throw an exception. That there are 2 names with confusion prefixs
					//a conflict like "/bl" and  "bl1" when looking for "bl".
					//this cozed by the fliexibility
					throw fawkes::IllegalArgumentException("AdvertismentCPM: Could not select processor, 2 conflicting names!");

				}
				else if(processor_prefix.length() > match_prefix.length())
				{
					match_prefix = processor_prefix;
				}
			}
		}
	}

	if(match_prefix.length() == 0)
	{
		throw fawkes::IllegalArgumentException("AdvertismentCPM: No processor recognized for this topic!") ;
	}

	//Go with To the proper operation with the bridge_prefix and the request Paramters
	if(msg_op=="advertise")
	{	
		std::string  topic_name 	= 	"";
		std::string  id 			= 	"";
		std::string  type			=	"";	
		
		if(d.HasMember("topic"))		 topic_name 	= 	std::string(d["topic"].GetString());
		if(d.HasMember("id")) 			 id 			= 	std::string(d["id"].GetString());
		if(d.HasMember("type")) 		 type			=	std::string(d["type"].GetString());	
		
		advertise( match_prefix, topic_name , id , type , session);
	}else 

	if (msg_op=="unadvertise")
	{
		std::string topic_name 		= 	std::string(d["topic"].GetString());
		std::string id 				= 	std::string(d["id"].GetString());
	
		unadvertise(match_prefix, topic_name, id , session);	
	}else 

	if (msg_op=="publish")
	{
		std::string topic_name 		= 	std::string(d["topic"].GetString());
		std::string id 				= 	std::string(d["id"].GetString());
		bool latch					= 	d["latch"].GetBool();
		//TEMP: for now keep the msg as a json and just forward it to ros
		StringBuffer buffer;
   		Writer<StringBuffer> writer(buffer);
    	d["msg"].Accept(writer);
		std::string msg_jsonStr		= buffer.GetString();
	
		publish(match_prefix, topic_name, id , latch , msg_jsonStr , session);	
	}

}

void
AdvertismentCapabilityManager::callback(EventType event_type , std::shared_ptr <EventEmitter> event_emitter)
{
	try{
		//check if the event emitter was a Advertisment
		std::shared_ptr <Advertisment> advertisment;
		advertisment = std::dynamic_pointer_cast<Advertisment> (event_emitter);
		if(advertisment != NULL)
		{
			if(event_type == EventType::TERMINATE )
			{
				//construct the prefixed_name from info in the advertisment
				//std::string prefixed_topic_name="/"+advertisment->get_processor_prefix()+"/"+advertisment->get_topic_name();
				std::string prefixed_topic_name=advertisment->get_topic_name();
				
				//does the advertisment exist (unique per topic_name)
				if (topic_Advertisment_.find(prefixed_topic_name) != topic_Advertisment_.end())
				{
					topic_Advertisment_.erase(prefixed_topic_name);
				}
			}
		}
	}
	catch(Exception &e){
		//if exception was fired it only means that the casting failed becasue the emitter is not a advertisment
	}
}

void 
AdvertismentCapabilityManager::advertise( std::string bridge_prefix
										, std::string topic_name 
										, std::string id 		
										, std::string type	
									   	, std::shared_ptr<WebSession> session)
{

	std::shared_ptr <AdvertismentCapability> processor;
	processor = std::dynamic_pointer_cast<AdvertismentCapability> (processores_[bridge_prefix]);
	//should be garanteed to work

	std::shared_ptr <Advertisment> advertisment;
	
	try{
		//always creates a new advertisment for that topic with the given Session and parameters
		//TODO:: pass the pure string arguments or a "protocol" type
		advertisment = processor-> advertise(topic_name 
													, id 		
													, type	
													, session);

	}catch(Exception &e){
		throw e;
	}

	/*push it to the topic_advertisment_map maintaing only ONE Advertisment instance per topic.
	*ps.Advertisment contains all the clients maped to thier individuale requests*/

	//Is it a new topic? Just push it to the map and activate the Advertisment

	//Mutex.lock()
	if( topic_Advertisment_.find(topic_name) == topic_Advertisment_.end() )
	{
		topic_Advertisment_[topic_name] = advertisment;
		//Activate the listeners or whatever that publishs
		//advertisment->register_callback( TERMINATE , shared_from_this() );
		advertisment->activate();
		//Advertisments should norify me if it was terminated (by calling my callback)
		advertisment->register_callback( EventType::TERMINATE , shared_from_this() );
	}else{
		topic_Advertisment_[topic_name]->subsume(advertisment);
		//advertisment->finalize();
	}

	//To be moved back to the subscrib() if the processor
	//..Or does it! u want to keep track of the advertisment all the time..leaving that as a choice to the processor does not seem right.
	// topic_Advertisment_[topic_name]->add_request(id , compression , throttle_rate , queue_length , fragment_size , session);
	

	//Mutex.unlock();
}

void
AdvertismentCapabilityManager::unadvertise	( std::string bridge_prefix
											, std::string topic_name 
											, std::string id 		
											, std::shared_ptr<WebSession> session)
{
	//select thre right processor
	std::shared_ptr <AdvertismentCapability> processor;
	processor = std::dynamic_pointer_cast<AdvertismentCapability> (processores_[bridge_prefix]);
	
	std::shared_ptr <Advertisment> advertisment;
	
	if(topic_Advertisment_.find(topic_name) != topic_Advertisment_.end()){

		advertisment = topic_Advertisment_[topic_name];

		try{
			processor->unadvertise(id, advertisment ,session );
		}catch (Exception &e){
			throw e;
		}

		if(advertisment-> empty()){
			topic_Advertisment_[topic_name] -> finalize();
			topic_Advertisment_.erase(topic_name);
		}

		return;
	}
	//throw exception that topic was not found

}

void
AdvertismentCapabilityManager::publish	( std::string bridge_prefix
											, std::string topic_name 
											, std::string id
											, bool latch
											, std::string msg_jsonStr	
											, std::shared_ptr<WebSession> session)
{
	//select thre right processor
	std::shared_ptr <AdvertismentCapability> processor;
	processor = std::dynamic_pointer_cast<AdvertismentCapability> (processores_[bridge_prefix]);
	
	std::shared_ptr <Advertisment> advertisment;
	
	if(topic_Advertisment_.find(topic_name) != topic_Advertisment_.end()){

		advertisment = topic_Advertisment_[topic_name];

		try{
			processor->publish(id , latch , msg_jsonStr , advertisment , session );
		}catch (Exception &e){
			throw e;
		}

		if(advertisment-> empty()){
			topic_Advertisment_[topic_name] -> finalize();
			topic_Advertisment_.erase(topic_name);
		}
		return;
	}
	//throw exception that topic was not found

}

