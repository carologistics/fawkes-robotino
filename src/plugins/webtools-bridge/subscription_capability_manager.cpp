/***************************************************************************
 *  subscription_capability_manager.cpp - Handler of SubscriptionCapability
 *related Requests (subscribe and unsubscribe). Created: 2016 Copyright  2016
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

#include "subscription_capability_manager.h"

#include "subscription_capability.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/thread.h>
#include <utils/time/time.h>

using namespace fawkes;
using namespace rapidjson;

// DECIDE..will i extend the susbscribtion capability or not...i can even make i
// a virtual inheritance. but then the capability manager also became a
// capability...it will implement the operations anyways and i can even keep the
// Subscription list in the base class and be able to access it from the manager
// and the processor...but Subscription then will contain all the list of any
// ever subscribed processor
// at the very same place..

/** @class SubscriptionCapabilityManager "subscription_capability_manager.h"
 * Provides handlers to process the JSON messages with opcodes ["subscribe" ,
 * "unsubscribe"] and dispatches them to the corresponding operation provided by
 * SubscriptionCapability [subscribe(), unsubscribe()] More importantly, it
 * performs the necessary subscription book keeping.
 *
 * It maintains mapping for BridgeProcessors that provide
 * SubscriptionCapability, by their prefix. Furthermore, it maintains the book
 * keeping of all Subscriptions made to a topic. Each unique topic is mapped to
 * a single Subscription instance (regardless where the topic data lives or
 * which BridgeProcessor creates the Subscription). Subscription instances are
 * created by the BridgeProcessor that know how to reach that topic data
 * (specified by its prefix).
 * @author Mostafa Gomaa
 */

/** Constructor. */
SubscriptionCapabilityManager::SubscriptionCapabilityManager() : CapabilityManager("Subscription")
{
	__mutex         = new fawkes::Mutex();
	__publish_mutex = new fawkes::Mutex();
}

/** Destructor. */
SubscriptionCapabilityManager::~SubscriptionCapabilityManager()
{
	// TODO: check if something needs to be changed
	if (!finalized_) {
		finalize();
	}

	if (publisher_thread != NULL) {
		publisher_thread->join();
	}

	delete __mutex;
	topic_Subscription_.clear();
}

void
SubscriptionCapabilityManager::init()
{
	if (!initialized_) {
		// TODO::the publish_loop is a temporary way to publish those topics that do
		// not have their own publish on topic update mechanism it runs on a
		// separate thread dedicated to emit periodic publish events causing any
		// subscription registered to this periodic publisher, to call its internal
		// publish
		run_publish_loop = true;
		publisher_thread =
		  std::make_shared<std::thread>(&SubscriptionCapabilityManager::publish_loop, this);

		// std::cout << "publisher loop intrialized"<<std::endl;

		CapabilityManager::init();
	}
}

void
SubscriptionCapabilityManager::finalize()
{
	MutexLocker ml(__mutex);
	MutexLocker ml_publish(__publish_mutex);

	if (!finalized_) {
		run_publish_loop = false;

		for (std::map<std::string, std::shared_ptr<Subscription>>::iterator it =
		       topic_Subscription_.begin();
		     it != topic_Subscription_.end();
		     it++) {
			// TODO::Completely remove this hard coding after creating a proper
			// publisher
			if (it->second->get_processor_prefix() == "clips") {
				unregister_callback(EventType::PUBLISH, it->second);
			}

			it->second->finalize();
		}

		CapabilityManager::finalize();
	}
}

/** Register A BridgeProcessor
 * This method will be called from the BridgeManager whenever a BridgeProcessor
 * is registered to it. If the BridgeProcessor provides a
 * SubscriptionCapability,this method registers the processor with a key
 * indicating its prefix. Each Processor has a unique prefix that will be
 * included in the topic_name of the requests intended for this Processor.
 * Usually you do not need to call this method.
 * @param processor
 * @return true on sucess
 */
bool
SubscriptionCapabilityManager::register_processor(std::shared_ptr<BridgeProcessor> processor)
{
	std::shared_ptr<SubscriptionCapability> Subscription_processor;
	Subscription_processor = std::dynamic_pointer_cast<SubscriptionCapability>(processor);
	if (Subscription_processor == NULL) {
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

/** Handle Subscription Related Requests
 * This handler Processes the "subscribe"/"unsubscribe" JSON requests and
 * extracts the topic prefix (indicating the target BridgeProcessor where the
 * topic data lives) from the topic_name of the request. Then it forwards the
 * extracted request parameters necessary for the operation and forwards them to
 * the internal method that will process the operation.
 * @param d The Dom object containing the deserialized JSON request
 * @param session that made the request and where the replies should be sent.
 */
void
SubscriptionCapabilityManager::handle_message(Document &d, std::shared_ptr<WebSession> session)
{
	// TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if (!d.HasMember("topic") || !d.HasMember("id")) {
		throw fawkes::MissingParameterException(
		  "SubscriptionCPM: Wrong Json Msg, topic name or id missing!");
	}

	// TODO::Pass the Dom the a Protocol Class that will have the deserialized
	// types
	std::string msg_op = std::string(d["op"].GetString());

	std::string msg_topic    = std::string(d["topic"].GetString());
	std::string match_prefix = "";

	// Find longest matching processor presfix within topic_name
	// TODO:Check the logic..might be wrong
	// TODO:posible Optimization. if the same topic name exists in the
	// topic_subscription just get the prefix from there
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
					  "SubscriptionCPM: Could not select processor, 2 conflicting "
					  "names!");

				} else if (processor_prefix.length() > match_prefix.length()) {
					match_prefix = processor_prefix;
				}
			}
		}
	}

	if (match_prefix.length() == 0) {
		throw fawkes::IllegalArgumentException(
		  "SubscriptionCPM: No processor recognized for this topic!");
	}

	// Go with To the proper operation with the bridge_prefix and the request
	// Paramters
	if (msg_op == "subscribe") {
		std::string  topic_name    = "";
		std::string  id            = "";
		std::string  type          = "";
		std::string  compression   = "";
		unsigned int throttle_rate = 0;
		unsigned int queue_length  = 1;
		unsigned int fragment_size = 0;

		if (d.HasMember("topic"))
			topic_name = std::string(d["topic"].GetString());
		if (d.HasMember("id"))
			id = std::string(d["id"].GetString());
		if (d.HasMember("type"))
			type = std::string(d["type"].GetString());
		if (d.HasMember("compression"))
			compression = std::string(d["compression"].GetString());
		if (d.HasMember("throttle_rate"))
			throttle_rate = d["throttle_rate"].GetUint();
		if (d.HasMember("queue_length"))
			queue_length = d["queue_length"].GetUint();
		if (d.HasMember("fragment_size"))
			fragment_size = d["fragment_size"].GetUint();

		subscribe(match_prefix,
		          topic_name,
		          id,
		          type,
		          compression,
		          throttle_rate,
		          queue_length,
		          fragment_size,
		          session);
	} else

	  if (msg_op == "unsubscribe") {
		std::string topic_name = std::string(d["topic"].GetString());
		std::string id         = std::string(d["id"].GetString());

		unsubscribe(match_prefix, topic_name, id, session);
	}
}

/** Callback to Handle Events, The SubscriptionCapabilityManager is Registered
 * To. Will be called when a Subscription instance is terminated for any reason.
 * This callback insures the removal of the terminated Subscription instance
 * from the topic_subscription_mapping making sure the instance could safely go
 * out-of-scope.
 * @param event_type type of the event that caused this callback method to be
 * called.
 * @param event_emitter the object instance initiated the callback method call.
 */
void
SubscriptionCapabilityManager::callback(EventType                     event_type,
                                        std::shared_ptr<EventEmitter> event_emitter)
{
	MutexLocker ml(__mutex);
	if (finalized_) {
		return;
	}

	try {
		// check if the event emitter was a Subscription
		std::shared_ptr<Subscription> subscription;
		subscription = std::dynamic_pointer_cast<Subscription>(event_emitter);
		if (subscription != NULL) {
			if (event_type == EventType::TERMINATE) {
				// construct the prefixed_name from info in the subscription
				std::string prefixed_topic_name;
				if (subscription->get_processor_prefix() == "/") // temp fix to accommodate both prefixes
					prefixed_topic_name = subscription->get_topic_name();
				else
					prefixed_topic_name =
					  subscription->get_processor_prefix() + "/" + subscription->get_topic_name();

				// does the subscription exist (unique per topic_name)
				if (topic_Subscription_.find(prefixed_topic_name) != topic_Subscription_.end()) {
					if (subscription->get_processor_prefix() == "clips") {
						unregister_callback(EventType::PUBLISH, subscription);
					}

					topic_Subscription_[prefixed_topic_name]->finalize();
					topic_Subscription_.erase(prefixed_topic_name);
				}
			}
		}
	} catch (Exception &e) {
		// if exception was fired it only means that the casting failed because the
		// emitter is not a subscription
	}
}

/** Create a Subscription For A Topic and maintains Subscription Book keeping
 * This method will be called by the SubscriptionCapabilityManager's handler on
 * "subscribe" requests. first it forwards the "subscribe" request parameters to
 * the targeted BridgeProcessor, Which will execute the subscription (whatever
 * that means for this BirdgeProcessor!) and return a Subscription Object The
 * Subscription Object is then used to maintain the necessary subscription book
 * keeping.A unique mapping of single Subscription instance per topic is then
 * maintained by flattening all Subscription instances targeted for the one
 * topic.
 * @param bridge_prefix Key to select the BridgeProcessor that will process the
 * subscription.
 * @param topic_name Full name of the topic to subscribe, prefixed with the
 * Processor's prefix (ex, /blackboard/pose).
 * @param id The Id field in the subscription request. Used to map many requests
 * on the same subscription.
 * @param type The type field in the subscription request (According to
 * rosbridge protocol)
 * @param compression The compression field of the rosbridge subscription
 * request
 * @param throttle_rate	The throttle_rate field of the rosbridge subscription
 * request
 * @param queue_length The queue_length field of the rosbridge subscription
 * request
 * @param fragment_size The fragment_size field of the rosbridge subscription
 * request
 * @param session The session that made the subscription request, and where
 * publishing for the topic will be sent
 */
void
SubscriptionCapabilityManager::subscribe(std::string                 bridge_prefix,
                                         std::string                 topic_name,
                                         std::string                 id,
                                         std::string                 type,
                                         std::string                 compression,
                                         unsigned int                throttle_rate,
                                         unsigned int                queue_length,
                                         unsigned int                fragment_size,
                                         std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);
	if (finalized_) {
		return;
	}

	std::shared_ptr<SubscriptionCapability> processor;
	processor = std::dynamic_pointer_cast<SubscriptionCapability>(processores_[bridge_prefix]);

	// always creates a new subscription for that topic with the given Session and
	// request parameters
	std::shared_ptr<Subscription> subscription;
	try {
		// TODO:: pass the pure string arguments or a "protocol" type
		subscription = processor->subscribe(
		  topic_name, id, type, compression, throttle_rate, queue_length, fragment_size, session);

	} catch (Exception &e) {
		throw e;
	} // TODO

	// push it to the topic_subscription_map maintain only ONE Subscription
	// instance per topic. ps.Subscription contains all the sessions mapped to
	// their individual requests

	// Is it a new topic? Just push it to the map and activate the Subscription
	if (topic_Subscription_.find(topic_name) == topic_Subscription_.end()) {
		topic_Subscription_[topic_name] = subscription;

		// Activate the listeners or whatever that publishs
		subscription->activate();

		// Register for Subscriptions to notify me if it was terminated (by calling
		// my callback)
		subscription->register_callback(EventType::TERMINATE, shared_from_this());

		// HUGE TODO :: REMOVE THIS SHIT and make another publishing
		// skeem...subscriptions should allow someone to register a publisher.. (who
		// ever that is publisher..he should emit events..and when the event is
		// emitted.
		// It will call all the publish() from the subscription by the power of
		// call_backes
		if (bridge_prefix == "clips")
			register_callback(EventType::PUBLISH, topic_Subscription_[topic_name]);

	} else {
		topic_Subscription_[topic_name]->subsume(subscription);
	}

	// To be moved back to the subscrib() if the processor
	//..Or does it! u want to keep track of the subscription all the time..leaving
	// that as a choice to the processor does not seem right.
	// topic_Subscription_[topic_name]->add_request(id , compression ,
	// throttle_rate , queue_length , fragment_size , session);
}

/** Remove A Subscription For A Topic and maintain the Subscription Book
 * keeping.
 * @param bridge_prefix Key of the BridgeProcessor that will process the
 * unsubscribe operation.
 * @param topic_name Full name of the topic, prefixed with the Processor's
 * prefix (ex, /blackboard/pose).
 * @param id The Id field in the request. Used to select which request should be
 * removed form the Subscription.
 * @param session The session that made the subscription request, and the
 * destination for the publishing events of that topic.
 */
void
SubscriptionCapabilityManager::unsubscribe(std::string                 bridge_prefix,
                                           std::string                 topic_name,
                                           std::string                 id,
                                           std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);
	if (finalized_) {
		return;
	}

	// Cast the BridgeProcessor to a SubscriptioCapability to perform the
	// operation
	std::shared_ptr<SubscriptionCapability> processor;
	processor = std::dynamic_pointer_cast<SubscriptionCapability>(processores_[bridge_prefix]);

	std::shared_ptr<Subscription> subscription;
	if (topic_Subscription_.find(topic_name) != topic_Subscription_.end()) {
		subscription = topic_Subscription_[topic_name];

		try {
			processor->unsubscribe(id, subscription, session);
		} catch (Exception &e) {
			throw e;
		}

		if (subscription->empty()) {
			topic_Subscription_[topic_name]->finalize();
			topic_Subscription_.erase(topic_name);

			if (bridge_prefix == "clips") {
				unregister_callback(EventType::PUBLISH, topic_Subscription_[topic_name]);
			}
		}

		return;
	}

	// throw exception that topic was not found
}

/** Publish periodically */
// TODO:: this will be replaced with a more proper Publishing mechanism
void
SubscriptionCapabilityManager::publish_loop()
{
	MutexLocker ml(__publish_mutex);

	while (run_publish_loop) {
		emitt_event(EventType::PUBLISH);
		ml.unlock();
		usleep(100000);
		MutexLocker ml(__publish_mutex);
	}
}

void
SubscriptionCapabilityManager::emitt_event(EventType event_type)
{
	MutexLocker ml(mutex_);

	for (it_callables_ = callbacks_[event_type].begin();
	     it_callables_ != callbacks_[event_type].end();
	     it_callables_++) {
		(*it_callables_)->callback(event_type, shared_from_this());
	}
	// do_on_event(event_type);
}
