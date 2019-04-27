
/***************************************************************************
 *  rosbridge_proxy_processor.h - Processes  Requests Targeted for ROS. Proxy To
 *RosBridge server
 *
 *  Created: Mon Mar 2016
 *  Copyright  2016  Mosafa Gomaa
 *
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

#ifndef __PLUGINS_ROSPROXY_PROCESSOR_H_
#define __PLUGINS_ROSPROXY_PROCESSOR_H_

#include "advertisment_capability.h"
#include "bridge_processor.h"
#include "callable.h"
#include "service_capability.h"
#include "subscription_capability.h"

// TODO:move includes to cpp and use from namespace
#include <logging/logger.h>

#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

namespace fawkes {
class Clock;
class Logger;
class Configuration;
class Mutex;
} // namespace fawkes

using websocketpp::connection_hdl;

//=================================   Subscription
//=================================== its optional here wither to keep track of
// the subscribtions or act as a pure proxy only forwading requests.For now,
// pure proxy

//=================================   Processor
//===================================

class WebSession;
class ProxySession;
class EventEmitter;

class RosBridgeProxyProcessor : public BridgeProcessor,
                                public SubscriptionCapability,
                                public AdvertismentCapability,
                                public ServiceCapability,
                                public Callable,
                                public std::enable_shared_from_this<RosBridgeProxyProcessor>
{
public:
	RosBridgeProxyProcessor(std::string            prefix,
	                        fawkes::Logger *       logger,
	                        fawkes::Configuration *config,
	                        fawkes::Clock *        clock);

	virtual ~RosBridgeProxyProcessor();

	void init();
	void finalize();

	void on_open(std::shared_ptr<WebSession> web_session, connection_hdl hdl);
	void on_fail(connection_hdl hdl);
	void on_close(connection_hdl hdl);

	std::shared_ptr<Subscription> subscribe(std::string                 topic_name,
	                                        std::string                 id,
	                                        std::string                 type,
	                                        std::string                 compression,
	                                        unsigned int                throttle_rate,
	                                        unsigned int                queue_length,
	                                        unsigned int                fragment_size,
	                                        std::shared_ptr<WebSession> session);

	void unsubscribe(std::string                   id,
	                 std::shared_ptr<Subscription> subscription,
	                 std::shared_ptr<WebSession>   session);

	std::shared_ptr<Advertisment> advertise(std::string                 topic_name,
	                                        std::string                 id,
	                                        std::string                 type,
	                                        std::shared_ptr<WebSession> session);

	void unadvertise(std::string                   id,
	                 std::shared_ptr<Advertisment> advertisment,
	                 std::shared_ptr<WebSession>   session);

	void publish(std::string id,
	             bool        latch,
	             std::string msg_in_json // TODO:: figure out a clever way to keep
	                                     // track of msgs types and content without
	                                     // the need to have the info before hands
	             ,
	             std::shared_ptr<Advertisment> advertisment,
	             std::shared_ptr<WebSession>   session);

	void call_service(std::string srv_call_json, std::shared_ptr<WebSession> session);

	void callback(EventType event_type, std::shared_ptr<EventEmitter> handler);

	void process_request(std::shared_ptr<WebSession> session, std::string message);
	void connect_to_rosbridge(std::shared_ptr<WebSession> session);

private:
	std::string rosbridge_uri_;

	fawkes::Logger *       logger_;
	fawkes::Configuration *config_;
	fawkes::Clock *        clock_;
	fawkes::Mutex *        mutex_;

	std::list<std::shared_ptr<ProxySession>>           peers_;
	std::list<std::shared_ptr<ProxySession>>::iterator it_peers_;

	std::shared_ptr<websocketpp::client<websocketpp::config::asio_client>> rosbridge_endpoint_;
	std::shared_ptr<websocketpp::lib::thread>                              proxy_thread_;
};

#endif
