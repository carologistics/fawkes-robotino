#include "service_capability_manager.h"
#include "service_capability.h"
#include <iostream>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <core/exceptions/software.h>

using namespace fawkes;
using namespace rapidjson;

ServiceCapabilityManager::ServiceCapabilityManager()	
:	CapabilityManager("Service")
{

}

ServiceCapabilityManager::~ServiceCapabilityManager()
{
	//TODO: check if something needs to be changed
}

bool
ServiceCapabilityManager::register_processor(std::shared_ptr <BridgeProcessor> processor )
{
	std::shared_ptr <ServiceCapability> service_processor;
	service_processor = std::dynamic_pointer_cast<ServiceCapability> (processor);
	if(service_processor == NULL)
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
ServiceCapabilityManager::handle_message(Document &d
											, std::shared_ptr<WebSession> session)
{	
			std::cout<< "in manager"<< std::endl;

	//TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if(!d.HasMember("op")){
		throw fawkes::MissingParameterException("ServiceCPM: Wrong Json Msg, topic name or id missing!");
	}

	//TODO::Pass the Dom the a Protocol Class that will have the deserialized types
	std::string msg_op = std::string(d["op"].GetString());

	//set the processor to ros_processor. (the only processor that implements Services for now)
	std::string processor_prefix="/";
	if(processores_.find(processor_prefix) != processores_.end())
	{
		std::cout<< "found processor"<< std::endl;

		std::shared_ptr <ServiceCapability> service_processor;
		service_processor = std::dynamic_pointer_cast<ServiceCapability> (processores_[processor_prefix]);
		if(service_processor == NULL)
		{
			throw fawkes::MissingParameterException("ServiceCPM: processor type issue!");

		}
		//Just serilize the Dom back to a json and forward it to the processor as it is.
		if (msg_op=="call_service")
		{

		std::cout<< "op is call service"<< std::endl;
			//TEMP: for now keep the msg as a json and just forward it to ros
			StringBuffer buffer;
	   		Writer<StringBuffer> writer(buffer);
	    	d.Accept(writer);
			std::string srv_call_json	= buffer.GetString();

			service_processor -> call_service( srv_call_json , session);	
		}
	}
			std::cout<< "end manager"<< std::endl;

}