/***************************************************************************
 *  subscription_capability_manager.h- Handler and Dispatcher of Subscription
 *Requests and Objects. Created: 2016 Copyright  2016 Mostafa Gomaa
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

#include "callable.h"
#include "capability_manager.h"
#include "event_emitter.h"
#include "event_type.h"

#include <thread>
#include <unistd.h>

class Subscription;
class SubscriptionCapability;

namespace fawkes {
class Mutex;
}

class SubscriptionCapabilityManager
: public CapabilityManager,
  public EventEmitter,
  public Callable,
  public std::enable_shared_from_this<SubscriptionCapabilityManager>
{
public:
	SubscriptionCapabilityManager();
	~SubscriptionCapabilityManager();

	void init();
	void finalize();

	void handle_message(rapidjson::Document &d, std::shared_ptr<WebSession> session);

	bool register_processor(std::shared_ptr<BridgeProcessor> processor);

	void callback(EventType event_type, std::shared_ptr<EventEmitter> event_emitter);

	void publish_loop();

	void emitt_event(EventType event_type);

private:
	void subscribe(std::string                 bridge_prefix,
	               std::string                 topic_name,
	               std::string                 id,
	               std::string                 type,
	               std::string                 compression,
	               unsigned int                throttle_rate,
	               unsigned int                queue_length,
	               unsigned int                fragment_size,
	               std::shared_ptr<WebSession> session);

	void unsubscribe(std::string                 bridge_prefix,
	                 std::string                 topic_name,
	                 std::string                 id,
	                 std::shared_ptr<WebSession> session);

	std::map<std::string, std::shared_ptr<Subscription>> topic_Subscription_;
	std::shared_ptr<std::thread>                         publisher_thread;
	bool                                                 run_publish_loop;

	fawkes::Mutex *__mutex;
	fawkes::Mutex *__publish_mutex;
};
