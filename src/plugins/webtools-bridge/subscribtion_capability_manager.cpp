#include "capability_manager.h"
#include "subscribtion_capability.h"
#include "subscribtion.h"

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
	processores_[processor.get_prefix()]=processor;
	return true;
}

void
SubscribtionCapabilityManager::handle_message(Document &d, std::string client_id)
{

	//first decides which Source to call ..longest match ..
	//Q .. Do i have to look within each processor or is it enough with the map
		std::string source_name= "blackboard";

	//decides which opreation it should call

		std::string op_name= std::string(d["op"].GetString());

		if(op_name=="subscribe")
		 processores_[source_name].subscribe(Document &d, std::string client_id);
		else if (op_name=="unsubscribe")
			processores_[source_name].unsubscribe(Document &d, std::string client_id);

}
