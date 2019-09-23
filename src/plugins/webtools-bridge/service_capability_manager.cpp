/***************************************************************************
 *  service_capability_manager.cpp- Service Requests Dispatcher
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

#include "service_capability_manager.h"

#include "service_capability.h"

#include <core/exceptions/software.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <iostream>

using namespace fawkes;
using namespace rapidjson;

/** @class ServiceCapabilityManager "service_capability_manager.h"
        This class provides handlers for Operations provided by the
   ServiceCapability (Service calls). Messages with an "op:service_call" are
   handled here and dispatched to the targeted BridgeProcessor from its list.
   BridgeProcessor's registered here all provide ServiceCapability
*/

ServiceCapabilityManager::ServiceCapabilityManager() : CapabilityManager("Service")
{
}

ServiceCapabilityManager::~ServiceCapabilityManager()
{
	// TODO: check if something needs to be changed
}

bool
ServiceCapabilityManager::register_processor(std::shared_ptr<BridgeProcessor> processor)
{
	std::shared_ptr<ServiceCapability> service_processor;
	service_processor = std::dynamic_pointer_cast<ServiceCapability>(processor);
	if (service_processor == NULL) {
		return false;
	}
	// find if it was used before
	std::string processor_prefix = processor->get_prefix();

	if (processores_.find(processor_prefix) == processores_.end()) {
		// throw exception this prefix name is invalide coz it was used before
	}

	processores_[processor_prefix] = processor;

	return true;
}

/** Handles the Service Call Requests.
 * Dispatches the service call requests to the requested BridgeProcessor
 * providing the ServiceCapabiliy. For now this is hard coded to rout all
 * service_call requests to the RosBridgeProxyProcessor since it the only one
 * that provides a ServiceCapabiliy. This is wrong and this code should be
 * replaced with a dynamic selection like what is done in the handler of
 * SubscriptionCapabilityManager.
 * @param d the Dom object containg the JSON message
 * @param session the session issuing the request.
 */
// TODO::remnam it (DispatchtoCapability)
void
ServiceCapabilityManager::handle_message(Document &d, std::shared_ptr<WebSession> session)
{
	// std::cout<< "in manager"<< std::endl;

	// TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if (!d.HasMember("op")) {
		throw fawkes::MissingParameterException(
		  "ServiceCPM: Wrong Json Msg, topic name or id missing!");
	}

	// TODO::Pass the Dom the a Protocol Class that will have the deserialized
	// types
	std::string msg_op = std::string(d["op"].GetString());

	// set the processor to ros_processor. (the only processor that implements
	// Services for now)
	std::string processor_prefix = "/";
	if (processores_.find(processor_prefix) != processores_.end()) {
		// std::cout<< "found processor"<< std::endl;

		std::shared_ptr<ServiceCapability> service_processor;
		service_processor =
		  std::dynamic_pointer_cast<ServiceCapability>(processores_[processor_prefix]);
		if (service_processor == NULL) {
			throw fawkes::MissingParameterException("ServiceCPM: processor type issue!");
		}
		// Just serilize the Dom back to a json and forward it to the processor as
		// it is.
		if (msg_op == "call_service") {
			// std::cout<< "op is call service"<< std::endl;
			// TEMP: for now keep the msg as a json and just forward it to ros
			StringBuffer         buffer;
			Writer<StringBuffer> writer(buffer);
			d.Accept(writer);
			std::string srv_call_json = buffer.GetString();

			service_processor->call_service(srv_call_json, session);
		}
	}
	// std::cout<< "end manager"<< std::endl;
}