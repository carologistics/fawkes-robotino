#include "bridge_manager.h"

#include <core/exceptions/software.h>

using namespace rapidjson;

BridgeManager::BridgeManager()
{
}

BridgeManager::~BridgeManager()
{
}

void
BridgeManager::incoming(std::string json,std::shared_ptr<WebSession> session)
{
	Document  d;
	deserialize(json,d);

	if (!d.HasMember("op"))
	{
		throw fawkes::MissingParameterException("BridgeManager: wrong json!, Op field missing");
	}

    std::string op_name=std::string(d["op"].GetString());
	if(operation_cpm_map_.find(op_name)==operation_cpm_map_.end())
	{
		throw fawkes::UnknownTypeException("BridgeManager: There is no handler registered for the given operation ");
	}
	try{
		operation_cpm_map_[op_name]->handle_message(d,session);
	}catch(fawkes::Exception &e)
	{
		throw e;
	}
}

/**TODO:May be here and may be just pass the session..decide later. 
 *If here keep trake of different sessions*/
void
BridgeManager::outgoing(std::string jsonMsg, std::string client_id)
{
	//ex.clients_p[client_id]->send(jsonMsg)
}

//TODO:move into Util
bool 
BridgeManager::deserialize(std::string jsonStr,Document &d)
{
	const char* json = jsonStr.c_str();

	d.Parse(json);

	if (d.Parse(json).HasParseError())
	{
		std::cout<< GetParseError_En(d.GetParseError());
		return false;
	}

	return true;
}

	
bool
BridgeManager::register_operation_handler(std::string op_name,std::shared_ptr <CapabilityManager> cpm)
{
	if(operation_cpm_map_.find(op_name)==operation_cpm_map_.end())
	{
		operation_cpm_map_[op_name]=cpm;
		return true;
	}
	
	//throw fawkes::IllegalArgumentException("BridgeManager: Operation '" + op_name.c_str()+ "' was already registered");	
	return false;
}

//DECEIDE:maybe keep track of a list of processors with thier prefix
bool
BridgeManager::register_processor(std::shared_ptr<BridgeProcessor> processor)
{
	for (std::map<std::string, std::shared_ptr <CapabilityManager>>::iterator it= operation_cpm_map_.begin()
			;it!=operation_cpm_map_.end(); ++it )
	{
		it->second->register_processor(processor);
	}

	return true;
}



