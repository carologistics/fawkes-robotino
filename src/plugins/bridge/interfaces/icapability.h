#ifndef INTERFACE_CAPABILITY_H
#define INTERFACE_CAPABILITY_H

#include <rapidjson/document.h>


//The capability will be the access point for the operations..
//any operation belongs to a class that extends capability
using namespace rapidjson;

class Icapability{
public:
	
//will be called by incoming() in the from the bridge. 
//It is responsible in finding the right operation within this capability
	virtual void handle_message(Document &d)=0;

};

#endif