/***************************************************************************
 *  advertisment_capability_manager.cpp - Handler and Dispatcher of
 *Advertisement and Publish Requests and Objects. Created: 2016 Copyright  2016
 *Mostafa Gomaa
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

#include "advertisment_capability_manager.h"

#include "advertisment_capability.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

using namespace fawkes;
using namespace rapidjson;

/** @class AdvertismentCapabilityManager "advertisment_capability_manager.h"
 * Handles requests with rosbridge protocol opcodes ["advertise" , "unadvertise"
 * , "publish"] and dispatches them to the targeted BridgeProcessor (ex,
 * BlackBoard, clips, Ros) p
 */
AdvertismentCapabilityManager::AdvertismentCapabilityManager() : CapabilityManager("Advertisment")
{
	__mutex = new fawkes::Mutex();
}

AdvertismentCapabilityManager::~AdvertismentCapabilityManager()
{
	topic_Advertisment_.clear();
}

/** Initialize the Capability Manager */
void
AdvertismentCapabilityManager::init()
{
	initialized_ = true;
}

/** Finalize the Capability Manager */
void
AdvertismentCapabilityManager::finalize()
{
	MutexLocker ml(__mutex);

	if (!finalized_) {
		for (std::map<std::string, std::shared_ptr<Advertisment>>::iterator it =
		       topic_Advertisment_.begin();
		     it != topic_Advertisment_.end();
		     it++) {
			it->second->finalize();
		}

		CapabilityManager::finalize();
	}
}

bool
AdvertismentCapabilityManager::register_processor(std::shared_ptr<BridgeProcessor> processor)
{
	std::shared_ptr<AdvertismentCapability> Advertisment_processor;
	Advertisment_processor = std::dynamic_pointer_cast<AdvertismentCapability>(processor);
	if (Advertisment_processor == NULL) {
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

void
// TODO::remnam it (DispatchtoCapability)
AdvertismentCapabilityManager::handle_message(Document &d, std::shared_ptr<WebSession> session)
{
	// TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if (!d.HasMember("topic") || !d.HasMember("id")) {
		throw fawkes::MissingParameterException(
		  "AdvertismentCPM: Wrong Json Msg, topic name or id missing!");
	}

	// TODO::Pass the Dom the a Protocol Class that will have the deserialized
	// types
	std::string msg_op = std::string(d["op"].GetString());

	std::string msg_topic    = std::string(d["topic"].GetString());
	std::string match_prefix = "";

	// Find longest matching processor presfix within topic_name
	// TODO:Check the logic..might be wrong
	// TODO:posible Optimization. if the same topic name exists in the
	// topic_advertisment just get the prefix from there
	for (ProcessorMap::iterator it = processores_.begin(); it != processores_.end(); it++) {
		std::string processor_prefix = it->first;
		std::size_t found_at         = msg_topic.find(processor_prefix, 0);

		if (found_at != std::string::npos) {
			// allows freedom of 2 characters before the match to account
			// for leading "/" ot "//" or and other startting charachters
			if (found_at <= 1) {
				if (processor_prefix.length() == match_prefix.length()) {
					// Throw an exception. That there are 2 names with confusion prefixs
					// a conflict like "/bl" and  "bl1" when looking for "bl".
					// this cozed by the fliexibility
					throw fawkes::IllegalArgumentException(
					  "AdvertismentCPM: Could not select processor, 2 conflicting "
					  "names!");

				} else if (processor_prefix.length() > match_prefix.length()) {
					match_prefix = processor_prefix;
				}
			}
		}
	}

	if (match_prefix.length() == 0) {
		throw fawkes::IllegalArgumentException(
		  "AdvertismentCPM: No processor recognized for this topic!");
	}

	// Go with To the proper operation with the bridge_prefix and the request
	// Paramters
	if (msg_op == "advertise") {
		std::string topic_name = "";
		std::string id         = "";
		std::string type       = "";
		// bool 		 latch			=	false;
		// int 		 queue_size 	=	100;

		if (d.HasMember("topic"))
			topic_name = std::string(d["topic"].GetString());
		if (d.HasMember("id"))
			id = std::string(d["id"].GetString());
		if (d.HasMember("type"))
			type = std::string(d["type"].GetString());
		// if(d.HasMember("latch")) 		 latch			=
		// d["latch"].GetBool();
		// if(d.HasMember("queue_size")) 	 queue_size		=
		// d["type"].GetInt();

		advertise(match_prefix, topic_name, id, type, session);
	} else

	  if (msg_op == "unadvertise") {
		std::string topic_name = std::string(d["topic"].GetString());
		std::string id         = std::string(d["id"].GetString());

		unadvertise(match_prefix, topic_name, id, session);
	} else

	  if (msg_op == "publish") {
		std::string topic_name = std::string(d["topic"].GetString());
		std::string id         = std::string(d["id"].GetString());
		bool        latch      = d["latch"].GetBool();
		// TEMP: for now keep the msg as a json and just forward it to ros
		StringBuffer         buffer;
		Writer<StringBuffer> writer(buffer);
		d["msg"].Accept(writer);
		std::string msg_jsonStr = buffer.GetString();

		publish(match_prefix, topic_name, id, latch, msg_jsonStr, session);
	}
}

/** Callback To Be Called On Events This Instance Is Registered To
 * note that, AdvertisementCM implements Callable interface is notified when an
 * Advertisement instance is terminated. In turn, it removes all entries of that
 * instance from its registry.
 * @param event_type of event responsible for this call
 * @param event_emitter is a ptr to the instance that emitted that event
 */
void
AdvertismentCapabilityManager::callback(EventType                     event_type,
                                        std::shared_ptr<EventEmitter> event_emitter)
{
	MutexLocker ml(__mutex);

	try {
		// check if the event emitter was a Advertisment
		std::shared_ptr<Advertisment> advertisment;
		advertisment = std::dynamic_pointer_cast<Advertisment>(event_emitter);
		if (advertisment != NULL) {
			if (event_type == EventType::TERMINATE) {
				// construct the prefixed_name from info in the advertisment
				// std::string
				// prefixed_topic_name="/"+advertisment->get_processor_prefix()+"/"+advertisment->get_topic_name();
				std::string prefixed_topic_name = advertisment->get_topic_name();

				// does the advertisment exist (unique per topic_name)
				if (topic_Advertisment_.find(prefixed_topic_name) != topic_Advertisment_.end()) {
					topic_Advertisment_[prefixed_topic_name]->finalize();
					topic_Advertisment_.erase(prefixed_topic_name);
				}
			}
		}
	} catch (Exception &e) {
		// if exception was fired it only means that the casting failed becasue the
		// emitter is not a advertisment
	}
}

void
AdvertismentCapabilityManager::advertise(std::string                 bridge_prefix,
                                         std::string                 topic_name,
                                         std::string                 id,
                                         std::string                 type,
                                         std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);

	std::shared_ptr<AdvertismentCapability> processor;
	processor = std::dynamic_pointer_cast<AdvertismentCapability>(processores_[bridge_prefix]);
	// should be garanteed to work

	std::shared_ptr<Advertisment> advertisment;

	try {
		// always creates a new advertisment for that topic with the given Session
		// and parameters
		// TODO:: pass the pure string arguments or a "protocol" type
		advertisment = processor->advertise(topic_name, id, type, session);

	} catch (Exception &e) {
		throw e;
	}

	/*push it to the topic_advertisment_map maintain only ONE Advertisement
   *instance per topic. ps.Advertisement contains all the clients mapped to
   *their individual requests*/

	// Is it a new topic? Just push it to the map and activate the Advertisement

	// Mutex.lock()
	if (topic_Advertisment_.find(topic_name) == topic_Advertisment_.end()) {
		topic_Advertisment_[topic_name] = advertisment;
		// Activate the listeners or whatever that publishs
		// advertisment->register_callback( TERMINATE , shared_from_this() );
		advertisment->activate();
		// Advertisments should norify me if it was terminated (by calling my
		// callback)
		advertisment->register_callback(EventType::TERMINATE, shared_from_this());
	} else {
		topic_Advertisment_[topic_name]->subsume(advertisment);
		// advertisment->finalize();
	}

	// Mutex.unlock();
}

void
AdvertismentCapabilityManager::unadvertise(std::string                 bridge_prefix,
                                           std::string                 topic_name,
                                           std::string                 id,
                                           std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);

	// select thre right processor
	std::shared_ptr<AdvertismentCapability> processor;
	processor = std::dynamic_pointer_cast<AdvertismentCapability>(processores_[bridge_prefix]);

	std::shared_ptr<Advertisment> advertisment;

	if (topic_Advertisment_.find(topic_name) != topic_Advertisment_.end()) {
		advertisment = topic_Advertisment_[topic_name];

		try {
			processor->unadvertise(id, advertisment, session);
		} catch (Exception &e) {
			throw e;
		}

		if (advertisment->empty()) {
			topic_Advertisment_[topic_name]->finalize();
			topic_Advertisment_.erase(topic_name);
		}

		return;
	}
	// throw exception that topic was not found
}

void
AdvertismentCapabilityManager::publish(std::string                 bridge_prefix,
                                       std::string                 topic_name,
                                       std::string                 id,
                                       bool                        latch,
                                       std::string                 msg_jsonStr,
                                       std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);

	// select thre right processor
	std::shared_ptr<AdvertismentCapability> processor;
	processor = std::dynamic_pointer_cast<AdvertismentCapability>(processores_[bridge_prefix]);

	std::shared_ptr<Advertisment> advertisment;

	if (topic_Advertisment_.find(topic_name) != topic_Advertisment_.end()) {
		advertisment = topic_Advertisment_[topic_name];

		try {
			processor->publish(id, latch, msg_jsonStr, advertisment, session);
		} catch (Exception &e) {
			throw e;
		}

		if (advertisment->empty()) {
			topic_Advertisment_[topic_name]->finalize();
			topic_Advertisment_.erase(topic_name);
		}
		return;
	}
	// throw exception that topic was not found
}
