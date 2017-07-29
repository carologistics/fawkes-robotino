/***************************************************************************
 *  webtools_bridge_thread.cpp - Gives Websocket access ,by mimicking the
 *rosbridge protocol, to different Fawkes components
 *
 *  Created: Wed Jan 13 16:33:00 2016
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

#include "webtools_bridge_thread.h"

#include "advertisment_capability_manager.h"
#include "blackboard_processor.h"
#include "bridge_manager.h"
#include "clips_processor.h"
#include "rosbridge_proxy_processor.h"
#include "service_capability_manager.h"
#include "subscription_capability_manager.h"
#include "websocket_server.h"

#include <string>

using namespace fawkes;

/** @class WebtoolsBridgeThread "webtools_bridge_thread.h"
 * A fawkes based impelentation of rosbridge server/proxy
 *
 * @author Mostafa Gomaa
 */

/** Constructor. */
WebtoolsBridgeThread::WebtoolsBridgeThread()
: Thread("BridgeThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

WebtoolsBridgeThread::~WebtoolsBridgeThread()
{
}

void
WebtoolsBridgeThread::init()
{
	logger->log_info("Webtools-Bridge-thread", "initiating");

	int  rosbridge_port = config->get_int("/webtools-bridge/rosbridge-port");
	bool in_simulation  = config->get_bool("/clips-agent/rcll2016/enable-sim");

	if (in_simulation) {
		std::string server_bash =
		  "../src/plugins/webtools-bridge/./launch_server.bash -p " + std::to_string(rosbridge_port);
		if (system(server_bash.c_str()))
			logger->log_error("Error running %s: %s", server_bash.c_str(), ::strerror(errno));
		// Maybe wait a bit after starting the servers
	}

	bridge_manager_ = std::make_shared<BridgeManager>();

	std::shared_ptr<SubscriptionCapabilityManager> subscription_cpm =
	  std::make_shared<SubscriptionCapabilityManager>();

	std::shared_ptr<AdvertismentCapabilityManager> advertisment_cpm =
	  std::make_shared<AdvertismentCapabilityManager>();

	std::shared_ptr<ServiceCapabilityManager> service_cpm =
	  std::make_shared<ServiceCapabilityManager>();

	// register capability managers to the bridge under the operations they should
	// handle
	bridge_manager_->register_operation_handler("subscribe", subscription_cpm);
	bridge_manager_->register_operation_handler("unsubscribe", subscription_cpm);

	bridge_manager_->register_operation_handler("advertise", advertisment_cpm);
	bridge_manager_->register_operation_handler("unadvertise", advertisment_cpm);
	bridge_manager_->register_operation_handler("publish", advertisment_cpm);

	bridge_manager_->register_operation_handler("call_service", service_cpm);

	// register processors
	bridge_manager_->register_processor(
	  std::make_shared<BridgeBlackBoardProcessor>("blackboard", logger, config, blackboard, clock));

	bridge_manager_->register_processor(
	  std::make_shared<RosBridgeProxyProcessor>("/", logger, config, clock));

	std::map<std::string, LockPtr<CLIPS::Environment>> rv = clips_env_mgr->environments();

	if (rv.find("agent") != rv.end()) {
		bridge_manager_->register_processor(
		  std::make_shared<ClipsProcessor>("clips", logger, config, clock, clips_env_mgr));
	} else {
		logger->log_warn(name(), " CLIPS agent was not found. No bridge will be registered for it!");
	}

	try {
		web_server_ = websocketpp::lib::make_shared<WebSocketServer>(logger, bridge_manager_);
		web_server_->run(config->get_int("/webtools-bridge/port"));
	} catch (...) {
		logger->log_warn(name(), " Wepsocekt Server crached");
	}
}

void
WebtoolsBridgeThread::finalize()
{
}

void
WebtoolsBridgeThread::loop()
{
}
