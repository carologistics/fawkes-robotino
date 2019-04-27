/***************************************************************************
 *  subscription_capability.h - Subscription Capabiliy Interface
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

#ifndef __PLUGINS_SUBSCRIPTION_CAPABILITY_H_
#define __PLUGINS_SUBSCRIPTION_CAPABILITY_H_

#include "callable.h"
#include "event_emitter.h"
#include "event_type.h"

#include <list>
#include <map>
#include <memory>

namespace fawkes {
class Clock;
class Time;
class Mutex;
class Logger;
} // namespace fawkes

class Subscription;
class WebSession;
class EventEmitter;

/** @class SubscriptionCapability
 * Abstract class, providing session the ability to un/subscribe for updates a
 * some topic. The semantics of what a subscription means for some doamin, is up
 * to the processor that implements this capability.
 *
 * @author Mostafa Gomaa
 */
class SubscriptionCapability
{
public:
	/** To be implemented by the bridge processor for domain specific semantics
   * @param topic_name the session wishes to subscribe
   * @param id provided in the JSON message of the rosbridge protocol
   * @param type provided in the JSON message of the rosbridge protocol
   * @param compression provided in the JSON message of the rosbridge protocol
   * @param throttle_rate provided in the JSON message of the rosbridge protocol
   * @param queue_length provided in the JSON message of the rosbridge protocol
   * @param fragment_size provided in the JSON message of the rosbridge protocol
   * @param session the session provided the advertise of the rosbridge protocol
   * @return ptr to a newly created dormant subscription instance
   */
	virtual std::shared_ptr<Subscription> subscribe(std::string                 topic_name,
	                                                std::string                 id,
	                                                std::string                 type,
	                                                std::string                 compression,
	                                                unsigned int                throttle_rate,
	                                                unsigned int                queue_length,
	                                                unsigned int                fragment_size,
	                                                std::shared_ptr<WebSession> session) = 0;

	/** To be implemented by the bridge processor for domain specific semantics
   * @param id included in the JSON message
   * @param subscription ptr to the instance that wish to unsubscribe from
   * @param session the session initiating requesting
   */
	virtual void unsubscribe(std::string                   id,
	                         std::shared_ptr<Subscription> subscription,
	                         std::shared_ptr<WebSession>   session) = 0;
};

class Subscription : public Callable,
                     public EventEmitter,
                     public std::enable_shared_from_this<Subscription>
{
public:
	Subscription(std::string     topic_name,
	             std::string     processor_prefix,
	             fawkes::Logger *logger,
	             fawkes::Clock * clock);

	virtual ~Subscription();

	void        finalize(); // Implicitly calles deactivate
	void        activate();
	void        deactivate();
	bool        is_active();
	bool        empty();
	std::string get_topic_name();
	std::string get_processor_prefix();
	void        subsume(std::shared_ptr<Subscription> another_subscription);

	void add_request(std::string                 id,
	                 std::string                 compression,
	                 unsigned int                throttle_rate,
	                 unsigned int                queue_length,
	                 unsigned int                fragment_size,
	                 std::shared_ptr<WebSession> session);
	void remove_request(std::string id, std::shared_ptr<WebSession> session);

	void callback(EventType event_type, std::shared_ptr<EventEmitter> handler);

	void emitt_event(EventType event_type);

	void publish();

protected:
	virtual void        finalize_impl(); /**< Implicitly called from finalize(), implement when need
                      to extend finalization behaviour */
	virtual void        activate_impl(); /**< Implicitly called from  activate(), implement when need
                      to extend activate behaviour  */
	virtual void        deactivate_impl(); /**< Implicitly called from deactivate(), implement when
                        need to extend deactivat behaviour */
	virtual std::string serialize(std::string op, std::string topic, std::string id);

	fawkes::Mutex *__mutex; /**< needed to lock the eternal registry */

protected:
	void add_new_session(std::shared_ptr<WebSession> session);
	void remove_session(std::shared_ptr<WebSession> session);

	/** @brief the posible status of an instance. A DORMANT instance only keeps
   * the data. */
	enum Status { ACTIVE, DORMANT };

	/** @brief a single Request mad by a session for subscription. A session
   * can make multiple requests with different request parameters. */
	struct Request
	{
		Request() : id(""), compression(""), throttle_rate(0), queue_length(1), fragment_size(0)
		{
		}
		~Request()
		{
			last_published_time.reset();
		}
		std::string  id;            /**< specified in rosbridge protocol JSON request*/
		std::string  compression;   /**< specified in rosbridge protocol JSON request*/
		unsigned int throttle_rate; /**< specified in rosbridge protocol JSON request*/
		unsigned int queue_length;  /**< specified in rosbridge protocol JSON request*/
		unsigned int fragment_size; /**< specified in rosbridge protocol JSON request*/
		std::shared_ptr<fawkes::Time>
		  last_published_time; /**< specified in rosbridge protocol JSON request*/
	};

	Status      active_status_;    /**< Tracks the status of the instance */
	std::string topic_name_;       /**< Topic name this instance is maintaining*/
	std::string processor_prefix_; /**< processor specific prefix included in that
                                    topic name*/

	fawkes::Clock * clock_;  /**< Fawkes clock needed for periodic publishing*/
	fawkes::Logger *logger_; /**< Fawkes logger*/

	bool finalized_; /**< Is set to true if it was Object was finalized before */

	/// Contains mapping of session to request it made
	std::map<std::shared_ptr<WebSession>, std::list<Request>> subscriptions_;
	/// Iterator for subsciption_map
	std::map<std::shared_ptr<WebSession>, std::list<Request>>::iterator it_subscriptions_;
	/// Iterator for requests list
	std::list<Request>::iterator it_requests_;

	bool static compare_throttle_rate(Request first, Request second);

	// fawkes::Mutex				*sub_list_mutex_;
	// fawkes::Mutex 				*time_mutex_;
};

#endif