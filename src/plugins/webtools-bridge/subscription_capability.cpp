/***************************************************************************
 *  subscription_capability.cpp - Subscription Capabiliy Interface
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

#include "subscription_capability.h"

#include "web_session.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <rapidjson/document.h> //To be removed from here after serialzer is moved
#include <rapidjson/error/en.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <utils/time/time.h>

using namespace rapidjson;
using namespace fawkes;

//=================================   Subscription
//===================================

/** @class Subscription "subscription_capability.h"

 * A Subscription instance tracks Subscription requests made from various
 sessions to 'a' topic.
 * More impotently, a Subscription instance is responsible for serializing the
 topic's data and
 * publishing it (on demand) to the subscribed sessions.
 *
 * What's already there:
 * It provides a set of default operations, that will be used to create and
 maintain
 * a meaningful subscription instance (like, adding/removing a request to a
 session, how to publish to sessions,
 * callbacks for sessions termination events and an exemplary implementation of
 serializing a topic into
 * a "publish" opcode JSON .
 * This Class provides the infrastructure necessary to create your own custom
 Subscription.
 *
 * In practice, A session could make several subscription requests to the same
 topic
 * with different parameters (ex, ID, Throttle_rate, compression). Requests made
 by same session
 * are distinguished by their unique IDs (wrt, the session). When a session
 requests an "unsubscribe"
 * operation, it provides the request ID indicating a previously made
 "subscribe" request.
 * The unsubscription is then done by removing that single request (ie, a single
 request
 * should be removed and the rest of the session's requests should remain
 untouched).
 * When a publish is triggered the topic's data should be serialized in a JSON
 message (according to
 * Rosbridge protocol publish opcode message), and sent to each session with a
 selected throttle_rate.
 * The throttle_rate is selected to be the smallest one this session ever
 requested.
 * Details of what triggers the publish, is domain specific and is left to
 implemented by you.
 *
 * How and when to use:.
 * Derive this class and extend its virtual method to prescribe the following,
 WRT your domain:
 * - Any details necessary for your Subscription to have access to the topic's
 data.
 * - Any details used to know when to publish (when Activated).
 [ In activate_impl() ]
 * - How to access a topic's data and serialize it, in order to publish .
 [ In serialize() ]
 * - When should the publish() be called.
 * I.e, when extending this class, add any details or include any objects
 necessary to access the topics
 * data during publishing. The idea is that a Subscription should be self
 contained, It knows when it needs
 * to publish, how to serialize the publish message, to which sessions the
 publishing should be done and
 * with which publish rate. When extending this class you only have to take care
 of implementing how to
 * serialize the topic's data and what triggers the publish() calls. The
 publishing itself is simply done
 * by calling publish(). It handles the publishing to each of the sessions
 assuming serialize() returns
 * the correct JSON (look into the Rosbridge Protocol 'publish' msg and the
 exemplary serialize()
 * implementation included here)
 *
 * Which classes will use it:
 * To keep consistency with our approach. A Subscription should be created by a
 BridgeProcessor that provides
 * SusbcriptionCapabilty. The Subscription instance is created within its
 subscribe() method. The subscribe()
 * method is usually called by the handlers of SubscriptionCapbilityManager when
 they process a msg with "subscribe"
 * opcode. After creation, the Subscription instance is then returned to the
 SubscriptionCapabilityManager, which
 * works on maintaining the book keeping of Subscriptions to different topics.
 *
 * When a Subscription is created it is DORMANT at first. A DORMANT
 Subscriptions can not publish.
 * It only act as data keeper that could potentially become ACTIVE or be
 subsumed by an ACTIVE
 * Subscription. Only ACTIVE subscriptions can publish to sessions or subsume
 other DORMANT Subscription
 * instances targeted to the same topic (for flattening for example, keeping
 only one instance per topic).
 * If you wish to do any procedures that are required before you can serialize
 or publish the topic's data,
 * it is a good approach to do them the implementation of the SActivate_impl()
 virtual method. Activate_impl()
 * will be called whenever the Subscription becomes active. Exemplary procedures
 to be done in Activate_impl(),
 * are registering for events listener relevant to topics changes, Or
 prescribing when the publish should be
 * triggered.
 *
 * When you want to publish a topic simply call publish() and it will take care
 of publishing to all the
 * sessions according to some selected parameters (throttle_rate, queue_lenght,
 compression).
 *
 * In summery, An instance from of Subscription has no information by default
 about when it has to publish
 * the topic or how to serialize it. That information is usually specific to the
 domain of the topic.
 *
 * An exemplary derived class from Subscription, is BlackBoardSubscription. You
 can find it in " BlackBoardPRocessor.cpp"
 *
 * @author Mostafa Gomaa
 */

/** Constructor.
 * @param topic_name Full name of the topic, prefixed with the BridgeProcessor
 * it targets
 * @param prefix the prefix that identifies the BridgeProcessor
 * @param logger Fawkes logger
 * @param clock Fawkes clock
 */
Subscription::Subscription(std::string     topic_name,
                           std::string     prefix,
                           fawkes::Logger *logger,
                           fawkes::Clock * clock)
: active_status_(DORMANT),
  topic_name_(topic_name),
  processor_prefix_(prefix),
  clock_(clock),
  finalized_(false)
{
	logger_ = logger;
	__mutex = new fawkes::Mutex();
}

/** Destructor */
Subscription::~Subscription()
{
	subscriptions_.clear();
	delete __mutex;
}

/** Finalize a subscription before terminating the instance. */
void
Subscription::finalize()
{
	MutexLocker ml(__mutex);
	if (!finalized_) {
		return;
	}

	// If still active deactivate
	if (is_active()) {
		deactivate();
	}
	// call the extended version
	finalize_impl();

	// Deregister from sessions and remove them
	if (!empty()) {
		for (it_subscriptions_ = subscriptions_.begin(); it_subscriptions_ != subscriptions_.end();) {
			remove_session(it_subscriptions_->first);
			it_subscriptions_->second.clear();
			subscriptions_.erase(it_subscriptions_++);
		}

		finalized_ = true;
	}
}

/** Activate This Subscription instance
 * It Changes the status of the subscription to be ACTIVE enabling the publish
 * to be called. When a Subscription is created, It is created as DORMANT
 * subscription.(It can not Publish to the sessions). This method is called from
 * SubscriptionCapabilyManager when it decides that this Subscription is the
 * main one and it should be able to publish. The method will call
 * Activate_impl() by default to execute any extended behaviours.
 */
void
Subscription::activate()
{
	if (!is_active()) {
		activate_impl();
		active_status_ = ACTIVE;
	}
}

/** Make The Subscription DORMANT
 * When a Subscription is DORMANT it can not publish and it could be subsumed by
 * another ACTIVE subscription.
 * The method will call deactivate_impl() by default to execute any extended
 * behaviours.
 */
void
Subscription::deactivate()
{
	if (is_active()) {
		deactivate_impl();
		active_status_ = DORMANT;
	}
}

/** @return True when its an ACTIVE instance */
bool
Subscription::is_active()
{
	return (active_status_ == ACTIVE);
}

/** Is the Subscription Empty ?
 * A subscription is empty if it has no sessions that own any requests.
 * @return true if subscribtion has no sessions registered
 */
bool
Subscription::empty()
{
	// This assumes that the clients removal and the removal of their
	// subscriptions were done correctly
	return subscriptions_.empty();
}

/** @return topic name maintained by this advertisement instance */
std::string
Subscription::get_topic_name()
{
	return topic_name_;
}

/** @return the processor prefix of that topic */
std::string
Subscription::get_processor_prefix()
{
	return processor_prefix_;
}

/** Finalize extended behaviour
 * This method will be called by default from Finalize()
 * Extend this if you wish to add any operations upon finalizing of the your
 * Subscription. After this the your Subscription should be ready to go out of
 * scope or deleted.
 */
void
Subscription::finalize_impl()
{
	// Override to extend behaviour
}

/** Activate extended behaviour
 * This method will be called by default from Activate()
 * Activate_impl() will be called right before the Subscription becomes active.
 * Exemplary procedures to be done in Activate_impl(), are registering for
 * events listener relevant to topics changes, Or prescribing when the publish
 * should be triggered.
 */
void
Subscription::activate_impl()
{
	// Override to extend behaviour
}

/** Deactivate extended behaviour
 * This method will be called by default from dectivate()
 * any operation done to make the Subscription active "could be" undone here.
 */
void
Subscription::deactivate_impl()
{
	// Override to extend behaviour
}

/**Subsumes a DORMANT Subscription instance into an ACTIVE one.
 * This is usually called when there is more than one Subscription instance for
 * the same topic. The owning instance must be an ACTIVE one and the instance to
 * be subsumed must to be DORMANT. The method call results in, the ACTIVE
 * subscription to have all the data copied from the DORMANT one and, the
 * DORMANT subscription is ready to be deleted or go out of scope. ex,
 * ActiveInstance.Subsume(DormantInstance). After the call, the dormant instance
 * could be safely deleted.
 * @param dormant_subscription The Dormant Subscription Instance to subsume
 */
void
Subscription::subsume(std::shared_ptr<Subscription> dormant_subscription)
{
	if (topic_name_ != dormant_subscription->get_topic_name()) {
		// throw exceptoin that they dont belong to the same topic and cant be
		// merged
		return;
	}

	if (!is_active()) {
		// throw exceptoin that they dont belong to the same topic and cant be
		// merged
		return;
	}

	if (dormant_subscription->is_active()) {
		// throw exceptoin that they dont belong to the same topic and cant be
		// merged
		return;
	}

	for (std::map<std::shared_ptr<WebSession>, std::list<Request>>::iterator it_subscriptions =
	       dormant_subscription->subscriptions_.begin();
	     it_subscriptions != dormant_subscription->subscriptions_.end();
	     it_subscriptions++) {
		for (std::list<Request>::iterator it_requests = it_subscriptions->second.begin();
		     it_requests != it_subscriptions->second.end();
		     it_requests++) {
			add_request(it_requests->id,
			            it_requests->compression,
			            it_requests->throttle_rate,
			            it_requests->queue_length,
			            it_requests->fragment_size,
			            it_subscriptions->first);

			// dormant_subscription->remove_request( it_requests ->id ,
			// it_subscriptions ->first);
		}
	}
	dormant_subscription->finalize();
}

/** Add A Subscription Request to A Session
 * Each time a session requests to "subscribe" to the
 * topic the Subscription is tracking, the parameters
 * of that request is added to the list of requests of the requesting
 * session. The request with the least Throttle_rate is always on the
 * top of that listing.This should be called each time "subscribe"
 * request is received to store the request parameters.
 * @param id included in the JSON request
 * @param compression included in the JSON request
 * @param throttle_rate included in the JSON request
 * @param queue_length included in the JSON request
 * @param fragment_size included in the JSON request
 * @param session the session requesting to advertise the topic maintained by
 * this instance.
 */
void
Subscription::add_request(std::string                 id,
                          std::string                 compression,
                          unsigned int                throttle_rate,
                          unsigned int                queue_length,
                          unsigned int                fragment_size,
                          std::shared_ptr<WebSession> session)
{
	Request request;
	// CHANGE:this matches for the pointer not the object
	MutexLocker ml(__mutex);

	it_subscriptions_ = subscriptions_.find(session);

	// if it is a new session, register my terminate_handler for the session's
	// callbacks
	if (it_subscriptions_ == subscriptions_.end()) {
		add_new_session(session);
	}

	// if there was older requests for this session,  point to the same
	// last_published_time
	if (it_subscriptions_ != subscriptions_.end() && !(subscriptions_[session].empty())) {
		// if(subscriptions_[session].find(id) != subscriptions_[session].end())
		// {
		// 	//throw exception..That id already exists for that client on that topic
		// }

		request.last_published_time = subscriptions_[session].front().last_published_time;
	} else {
		request.last_published_time = std::make_shared<fawkes::Time>(clock_);
	}

	request.id            = id;
	request.compression   = compression;
	request.throttle_rate = throttle_rate;
	request.queue_length  = queue_length;
	request.fragment_size = fragment_size;

	// sub_list_mutex_->lock();
	subscriptions_[session].push_back(request);
	subscriptions_[session].sort(compare_throttle_rate); // replace by a lambda function
	                                                     // sub_list_mutex_->unlock();
}

/** Remove A Request Upon an Unsubscribe Operation
 * This removes the request with the ID included in the "Unsubscribe" request.
 * This should be called by each unsubscribe() operation.
 * @param subscription_id ID of the request to be removed
 * @param session that initiated the request
 */
void
Subscription::remove_request(std::string subscription_id, std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);

	it_subscriptions_ = subscriptions_.find(session);

	if (it_subscriptions_ == subscriptions_.end()) {
		// there is no such session. Maybe session was closed before the request is
		// processed
		return;
	}

	for (it_requests_ = subscriptions_[session].begin();
	     it_requests_ != subscriptions_[session].end();
	     it_requests_++) {
		if ((*it_requests_).id == subscription_id) {
			subscriptions_[session].erase(it_requests_);
			break;
		}
	}

	// sub_list_mutex_->lock();
	if (subscriptions_[session].empty()) {
		remove_session(session);
		subscriptions_.erase(session);
	} else {
		// Always have the least throttel rate on the top
		subscriptions_[session].sort(compare_throttle_rate);
	}
}

void
Subscription::emitt_event(EventType event_type)
{
	for (it_callables_ = callbacks_[event_type].begin();
	     it_callables_ != callbacks_[event_type].end();
	     it_callables_++) {
		(*it_callables_)->callback(event_type, shared_from_this());
	}
}

/** Callback To Be Called On Events The Subscription Instance Is Registered To
 * Subscription implements Callable interface and is notified when a Session
 * is terminated. It removes the session from its entries with all of its
 * requests.
 * @param event_type of event responsible for this call
 * @param event_emitter is a ptr to the instance that emitted that event
 */
void
Subscription::callback(EventType event_type, std::shared_ptr<EventEmitter> event_emitter)
{
	// sub_list_mutex_->lock();
	MutexLocker ml(__mutex);
	if (finalized_) {
		return;
	}

	if (event_type == EventType::PUBLISH) {
		ml.unlock();
		publish();
	}

	try {
		// check if the event emitter was a session
		std::shared_ptr<WebSession> session;
		session = std::dynamic_pointer_cast<WebSession>(event_emitter);
		if (session != NULL) {
			if (event_type == EventType::TERMINATE) {
				// make sure the session is still there and was not deleted while
				// waiting for the mutex
				if (subscriptions_.find(session) != subscriptions_.end())
					subscriptions_.erase(session);

				// was it the last session? if yes, Subscription emit TERMINATTION event
				// and destories itself.
				if (subscriptions_.empty()) {
					std::shared_ptr<Subscription> my_self =
					  shared_from_this(); // Just to keep object alive till after its
					                      // deleted from manager
					ml.unlock();          // to allow others to call finialize or do whatever is
					                      // necessary to the object
					emitt_event(EventType::TERMINATE);
				}

				// std::cout<< "Session terminated NICELY :D" << std::endl;

				// was it the last session? if yes, Subscription emit TERMINATTION event
				// and destories itself.
				// if(subscriptions_.empty()){
				// 	std::shared_ptr<Subscription> my_self= shared_from_this();//
				// Just to keep object alive till after its deleted from manager
				// 	emitt_event(EventType::TERMINATE);

				// 	ml.unlock();
				// 	my_self->finalize();
				// 	//std::cout<< "Subscripton topic terminated!" << std::endl;

				// 	//finalize will need the mutex
				// }
				// TODO:check if subscription became empty and trigger the delete from
				// the owning class if that was the case
			}
		}

	} catch (Exception &e) {
		// if exception was fired it only means that the casting failed becasue the
		// emitter is not a session
	}
}

/** Register the session to recive callbacks on emmited Termination Events
 * @param session
 */
void
Subscription::add_new_session(std::shared_ptr<WebSession> session)
{
	// register termination handler
	session->register_callback(EventType::TERMINATE, shared_from_this());
	//	subscriptions_[session];// Check if okay
}

/** Unregister the session from receiving callbacks on emmited Termination
 * Events
 * @param session
 */
void
Subscription::remove_session(std::shared_ptr<WebSession> session)
{
	// deregister termination handler
	session->unregister_callback(EventType::TERMINATE, shared_from_this());
	//	subscriptions_.erase(session);
}

/** Publish A Topic Data To The Sessions.
 * This should be called whenever you want to publish.
 * Events that could trigger a publish, like a periodic clock
 * or an event listener, should make calls to this method, in order to, perform
 * the publishing to subscribed sessions.
 * It checks for each session if enough time has passed since its last publish,
 * it serialize the data and sends the serialized JSON. (Otherwise, for now just
 * drop it new data. Later maybe queue it depending on the queue size). The
 * request parameters with the least Throttle_rate is used if multiple requests
 * exist for the same session .
 */
void
Subscription::publish()
{
	MutexLocker ml(__mutex);
	if (is_active()) {
		for (it_subscriptions_ = subscriptions_.begin(); it_subscriptions_ != subscriptions_.end();
		     it_subscriptions_++) {
			if (it_subscriptions_->second.empty()) {
				// This means unsubscribe() didnt work properly to delete that session
				// after unsubscribing all clients components throw some exception
				remove_session(it_subscriptions_->first);
				subscriptions_.erase(it_subscriptions_++);
				continue;
			}

			// Checking if it is time to publish topic
			// sub_list_mutex_->lock();
			fawkes::Time now(clock_);
			// smallest throttle_rate always on the top
			unsigned int throttle_rate = it_subscriptions_->second.front().throttle_rate;
			std::string  id =
			  it_subscriptions_->second.front().id; // could be done here to minimize locking
			unsigned int last_published =
			  it_subscriptions_->second.front().last_published_time->in_msec();
			unsigned int time_passed = (now.in_msec() - last_published);
			// sub_list_mutex_->unlock();

			if (time_passed >= throttle_rate) {
				std::string complete_json_msg = serialize("publish", topic_name_, id);

				// Only stamp if it was sent
				// time_mutex_->lock();
				it_subscriptions_->second.front().last_published_time->stamp();
				// time_mutex_->unlock();

				// Unnecessary right now. No Mutex at session
				//// To avoid deadlock if the session mutex was locked to process a
				/// request
				////(and the request cant add coz ur locking the subscription)
				////ml.unlock();

				// send msg it to session
				if (complete_json_msg.size() > 1)
					it_subscriptions_->first->send(complete_json_msg);
				////ml.relock();
			}
		}
	}
}

/** Serialize the Topic's Data in a "Publish" message
 * This is a simple exemplary implementation of what the serialize should look
 * like. This has to be implemented and filled with the topic's data, depending
 * on your domain, to be used by publish(). in this example, we just constructs
 * a publish message without any topic data in the msg field. Usually, msg field
 * should contain a JSON that holds the topic data. This is the message that
 * will be sent to sessions. Add the topic data to the JSON in similar fashion.
 * This is called from the Publish method of the Subscription instance when a
 * publish event is due (whenever that is).
 * @param op name of the operation according to rosbridge protocol
 * @param topic_name name of the topic to be serialized
 * @param id
 * @return the serialized json string
 *
 * @todo
 * Serialization could be moved later to a Protocol handling utility.
 */
std::string
Subscription::serialize(std::string op, std::string topic_name, std::string id)
{
	// MutexLocker ml(mutex_);

	// std::cout <<" default serialzer shoudl not come here "<< std::endl;
	std::string prefixed_topic_name =
	  processor_prefix_ + "/" + topic_name_; // not always the cast. (for its ros /topic_name)

	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("msg");
	writer.StartObject();
	// Serialize you data json here
	writer.EndObject(); // End if data json

	writer.EndObject(); // End of complete Json_msg

	////std::cout << s.GetString() << std::endl;

	return s.GetString();
}

//-------------To be removed

/** compares throttle rate of tow requests
 * @param first
 * @param second
 * @return true if throttle rates of both requests is the same
 * @todo
 * replace by a lambda function
 */
bool
Subscription::compare_throttle_rate(Request first, Request second)
{
	return (first.throttle_rate <= second.throttle_rate);
}
