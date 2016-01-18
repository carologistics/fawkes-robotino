#include "capability_manager.h"
#include "subscribtion_capability.h"

using namespace rapidjson;


//DECIDE..will i extend the susbscribtion capability or not...i can even make i a virtual inheritance.
//but then the capability manager also became a capability...it will implement the operations anyways
//and i can even keep the subscribtion list in the base class and be able to access it from the manager 
//and the processor...but Subscribtion then will contain all the list of any ever subscribed processor 
// at the very same place..

class SubscribtionCapabilityManager
: public CapabilityManger
{
private:

};

SubscribtionCapabilityManager::SubscribtionCapabilityManager()
:	CapabilityManager("subscribtion")
{

}

SubscribtionCapabilityManager::~SubscribetionCapabilityManager()
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
		std::size_t found = msg_topic.find(processor_prefix);
		if(found != std::string::npos)
		{
			//allow freedom of 2 characters before the match to account 
			//for leading "/" ot "//" or and other startting charachters
			if (found < 1)
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
		processores_[match_prefix]->
			subscribe(Document &d, session);
	}else 

	if (msg_op=="unsubscribe")
	}
		processores_[match_prefix]->
			unsubscribe(Document &d);
	}

}
