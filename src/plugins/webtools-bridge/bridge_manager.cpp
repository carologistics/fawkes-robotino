/***************************************************************************
 *  bridge_manager.cpp - Single access point for sessions to the bridge's capabilities.
 *  Created: 2016 
 *  Copyright  2016 Mostafa Gomaa 
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */


#include "bridge_manager.h"

#include <core/exceptions/software.h>

using namespace rapidjson;


/** @class BridgeManager 
 * Player plugin for Fawkes.
 * Keeps track of what capabilities the bridge provides, Dispatching the incoming requests to the proper capability manger.
 * @author Mostafa Gomaa
 */

BridgeManager::BridgeManager()
{
}

BridgeManager::~BridgeManager()
{
	
}

void BridgeManager::finalize()
{
	while (!operation_cpm_map_.empty())
	{
		operation_cpm_map_.begin()-> second-> finalize();
		operation_cpm_map_.erase(operation_cpm_map_.begin());
	}
}

void
BridgeManager::incoming(std::string json,std::shared_ptr<WebSession> session)
{
	Document  d;
	deserialize(json,d);

	if (!d.HasMember("op"))
	{
		throw fawkes::MissingParameterException("BridgeManager: wrong json!, 'Op' field is missing!");
	}

    std::string op_name=std::string(d["op"].GetString());
	if(operation_cpm_map_.find(op_name)==operation_cpm_map_.end())
	{
		throw fawkes::UnknownTypeException("BridgeManager: There is no handler registered for the given operation ");
	}
	try{
		operation_cpm_map_[op_name]->handle_message(d,session);
	}

	catch(fawkes::Exception &e)
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
		//std::cout<< GetParseError_En(d.GetParseError());
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
		operation_cpm_map_[op_name]->init();
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
	processor->init();
	
	return true;
}



