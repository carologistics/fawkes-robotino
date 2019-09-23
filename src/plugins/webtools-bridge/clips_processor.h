
/***************************************************************************
 * clips_processor.h - Processor for Bridge Requests Targeted for Clips Facts
 *
 *  Created: Mon April 11 2016
 *  Copyright  21016 Mostafa Gomaa
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

#ifndef __PLUGINS_CLIPS_PROCESSOR_H_
#define __PLUGINS_CLIPS_PROCESSOR_H_

#include "bridge_processor.h"
#include "subscription_capability.h"

#include <config/config.h>
#include <core/utils/lockptr.h>

#include <string>

namespace fawkes {
class Clock;
class Logger;
class CLIPSEnvManager;
class Mutex;
} // namespace fawkes

namespace CLIPS {
class Environment;
}

class WebSession;

//=================================   Subscription
//===================================

class ClipsSubscription : public Subscription
{
public:
	ClipsSubscription(std::string                               topic_name,
	                  std::string                               processor_prefix,
	                  fawkes::Logger *                          logger,
	                  fawkes::Clock *                           clock,
	                  fawkes::LockPtr<CLIPS::Environment> &     clips,
	                  fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr);

	~ClipsSubscription();

	void activate_impl();
	void deactivate_impl();
	void finalize_impl();

	// void publish_loop();

	std::string serialize(std::string op, std::string topic, std::string id);

private:
	fawkes::LockPtr<CLIPS::Environment>      clips_;
	fawkes::LockPtr<fawkes::CLIPSEnvManager> clips_env_mgr_;
};

//=================================   Processor
//===================================

class ClipsProcessor : public BridgeProcessor, public SubscriptionCapability
{
public:
	ClipsProcessor(std::string                               prefix,
	               fawkes::Logger *                          logger,
	               fawkes::Configuration *                   config,
	               fawkes::Clock *                           clock,
	               fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr);

	virtual ~ClipsProcessor(); // why did i leave this virtual. does it make sense

	void init();

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

private:
	fawkes::Logger *       logger_;
	fawkes::Configuration *config_;
	fawkes::Clock *        clock_;

	fawkes::LockPtr<fawkes::CLIPSEnvManager> clips_env_mgr_;
	fawkes::LockPtr<CLIPS::Environment>      clips_;

	std::string env_name_;
};

#endif
