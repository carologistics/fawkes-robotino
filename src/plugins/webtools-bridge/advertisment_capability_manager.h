/***************************************************************************
 *  advertisment_capability_manager.h - Handler and Dispatcher of Advertisement
 *and Publish Requests and Objects. Created: 2016 Copyright  2016 Mostafa Gomaa
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
#include "event_type.h"

class Advertisment;
class AdvertismentCapability;
class EventHandter;

namespace fawkes {
class Mutex;
}

class AdvertismentCapabilityManager
: public CapabilityManager,
  public Callable,
  public std::enable_shared_from_this<AdvertismentCapabilityManager>
{
public:
	AdvertismentCapabilityManager();
	~AdvertismentCapabilityManager();

	void init();
	void finalize();

	void handle_message(rapidjson::Document &d, std::shared_ptr<WebSession> session);

	bool register_processor(std::shared_ptr<BridgeProcessor> processor);

	void callback(EventType event_type, std::shared_ptr<EventEmitter> event_emitter);

private:
	void advertise(std::string                 bridge_prefix,
	               std::string                 topic_name,
	               std::string                 id,
	               std::string                 type,
	               std::shared_ptr<WebSession> session);

	void unadvertise(std::string                 bridge_prefix,
	                 std::string                 topic_name,
	                 std::string                 id,
	                 std::shared_ptr<WebSession> session);

	void publish(std::string                 bridge_prefix,
	             std::string                 topic_name,
	             std::string                 id,
	             bool                        latch,
	             std::string                 msg_jsonStr,
	             std::shared_ptr<WebSession> session);

	std::map<std::string, std::shared_ptr<Advertisment>> topic_Advertisment_;

	fawkes::Mutex *__mutex;
};
