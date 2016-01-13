
#ifndef INTERFACE_CAPABILTY_MANAGER_H
#define INTERFACE_CAPABILTY_MANAGER_H


#include "interfaces/ibridge_processor.h"
#include <string>
#include <list>
#include <memory>

using namespace rapidjson;

class ICapabilityManager{

public :

	ICapabilityManager(std::sting cap_name):capability_name(capability_name)
	{}
	~ICapabilityManager()
	{delete processores_list_;}


	//check if the processor has this capability and either register it or returen false
	virtual bool register_processor(std::shared_ptr <IBridgeProcessor> )=0;

	virtual void handle_message(Document &d,std::string client_id)=0;

	std::string get_name(){
		return capability_name_;
	}

protected:
	std::string capability_name_;
	std::map<std::string,IBridgeProcessor> processores_;


};

#endif