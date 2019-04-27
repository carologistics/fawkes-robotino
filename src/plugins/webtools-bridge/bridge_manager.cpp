/***************************************************************************
 *  bridge_manager.cpp - Single access point for sessions to the bridge's
 *capabilities. Created: 2016 Copyright  2016 Mostafa Gomaa
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
 * An instance of this class, is the single access point for requests made by
 Sessions via Websocket.
 * The "Bridge" is an abstract concept representing group of Capabilities (like,
 Subscribtion , Advertising and ServiceCalls capabilites)
 * provided to remote clients, and executed on demand, giving access to
 different components connected to Fawkes (ex, Blackboard, CLIPS, ROS).
 * The requests are made via JSON following the Rosbridge Protocol, and received
 over Websockets from a remote client.

 * A so called "Bridge" can provide different capabilities to different
 components. Its up to each component's BridgeProcessor to choose what
 capabilities it wants to
 * provide by simply implementing each Capability's abstract class, prescribing
 what does it mean for that component to have this capability. for example, If a
 "Bridge"
 * should provide subscription capabilities to CLIPS facts. There has to be a
 CLIPS_Processor that implements SubscriptionCapability  class (and its
 * respected operations, Subscribe & Unsubscribe) providing access to clips
 facts and implementing a mechanism to when to publish those facts that has been
 updated.
 *
 * BridgeManager is the the single access point to the "Bridge".It where the
 first request processing and dispatching steps takes place.
 * BridgeManager keeps track of different CapabilitiyManagers that act as
 handlers to requests made to those capabilities.
 * Each CapabilityManager performs any book keeping or procedures necessary for
 the operations they manage.

 * @author Mostafa Gomaa
 */

/** Constructor. */
BridgeManager::BridgeManager()
{
}

/** Destructor. */
BridgeManager::~BridgeManager()
{
}

/** Finlize the instance */
void
BridgeManager::finalize()
{
	while (!operation_cpm_map_.empty()) {
		operation_cpm_map_.begin()->second->finalize();
		operation_cpm_map_.erase(operation_cpm_map_.begin());
	}
}

/** Processes incoming requests .
 * This is the first processing done on the request, dispatching it to the
 * proper handler.
 * @param json The request in JSON, structured according to RosBridge protocol
 * @param session The websocket session that made that request, and the
 * destination for the replies
 */
void
BridgeManager::incoming(std::string json, std::shared_ptr<WebSession> session)
{
	Document d;
	deserialize(json, d);

	if (!d.HasMember("op")) {
		throw fawkes::MissingParameterException("BridgeManager: wrong json!, 'Op' field is missing!");
	}

	std::string op_name = std::string(d["op"].GetString());
	if (operation_cpm_map_.find(op_name) == operation_cpm_map_.end()) {
		throw fawkes::UnknownTypeException("BridgeManager: There is no handler "
		                                   "registered for the given operation ");
	}
	try {
		operation_cpm_map_[op_name]->handle_message(d, session);
	}

	catch (fawkes::Exception &e) {
		throw e;
	}
}

/** Parse the json strinf into a dom
 * @param jsonStr to parse
 * @param d Dom that the message date will be stored into
 * @return true if parsing succeeded
 */
bool
BridgeManager::deserialize(std::string jsonStr, Document &d)
{
	const char *json = jsonStr.c_str();

	d.Parse(json);

	if (d.Parse(json).HasParseError()) {
		// std::cout<< GetParseError_En(d.GetParseError());
		return false;
	}

	return true;
}

/** Register the Operations Your Bridge Will Provide and Their Handlers.
 * To let your bridge provide certain capabilities, first they have to be
 * registered to the operations they will provide. This method keeps the mapping
 * that indicates which CapabilityManager handles a certain operation (op ex,
 * subscribe, call_service). one CapabilityManager can handle more than one
 * operation by being registered to each of them.
 * @todo : The name of the operations provided by a capability should be stored
 * and queried from in its CapabilityManager.
 * @param op_name The name of the operation to be matched with the "op" field of
 * the JSON request (ex, subscribe, publish).
 * @param cpm The CapabiliyManager instance that will handle the requests with
 * this "op" field
 * @return true on sucess
 */

// Should only register the CapabilyManager with no Info about what they
// provide.
bool
BridgeManager::register_operation_handler(std::string                        op_name,
                                          std::shared_ptr<CapabilityManager> cpm)
{
	if (operation_cpm_map_.find(op_name) == operation_cpm_map_.end()) {
		operation_cpm_map_[op_name] = cpm;
		operation_cpm_map_[op_name]->init();
		return true;
	}

	// throw fawkes::IllegalArgumentException("BridgeManager: Operation '" +
	// op_name.c_str()+ "' was already registered");
	return false;
}

/** Register a processor that implements a Capability or more for a Fawkes
 * component. This will try to register a Processor at all stored
 * CapabilityManagers this BridgeManager has, and only succeeds for those CPMs
 * that handle a Capability the Processor implements. Its important to only
 * start registering Processors, after all CapabilityManagers has been
 * registered, otherwise a some CapabilityManagers will not know about a
 * Processor that they should.
 * @param processor An instance of BridgeProcessor implementing one  Capability
 * or more for a certain Fawkes component.
 * @return true on success
 */
bool
BridgeManager::register_processor(std::shared_ptr<BridgeProcessor> processor)
{
	for (std::map<std::string, std::shared_ptr<CapabilityManager>>::iterator it =
	       operation_cpm_map_.begin();
	     it != operation_cpm_map_.end();
	     ++it) {
		it->second->register_processor(processor);
	}
	processor->init();

	return true;
}