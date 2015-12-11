#ifndef INTERFACE_CAPABILITY_H
#define INTERFACE_CAPABILITY_H

#include <rapidjson/document.h>
#include "ibridge_manager.h"
#include <memory>

//The capability will be the access point for the operations..
//any operation belongs to a class that extends capability
using namespace rapidjson;

class Icapability{
public:
	
//will be called by incoming() in the from the bridge. 
//It is responsible in finding the right operation within this capability

	Icapability(std::shared_ptr<IbridgeManager> manager)
	:manager_(manager)
	{}

	virtual void handle_message(Document &d)=0;


protected:

	std::shared_ptr <IbridgeManager> manager_; 

};

#endif