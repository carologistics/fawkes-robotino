/***************************************************************************
 *  clips_mips_scheduler_thread.cpp - CLIPS feature for scheduling with MIPS
 *
 *  Created: Sun 8 Mar 2020 17:44:08 CET 17:44
 *  Copyright  2020  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

#include "clips_mip_scheduler_thread.h"

#include <core/threading/mutex_locker.h>

/** @class ClipsMipSchedulerThread "clips_mip_scheduler_thread.h"
 * Plugin thread that provides the MIP-scheduler CLIPS features.
 * @author Mostafa Gomaa
 */

ClipsMipSchedulerThread::ClipsMipSchedulerThread()
: Thread("ClipsMipSchedulerThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("mip-scheduler"),
  CLIPSFeatureAspect(this)
{
}

ClipsMipSchedulerThread::~ClipsMipSchedulerThread()
{
	clips_envs_.clear();
}

void
ClipsMipSchedulerThread::init()
{
	gurobi_env_    = new GRBEnv();
	GRBModel model = GRBModel(*gurobi_env_);
	model.set(GRB_StringAttr_ModelName, "SchedulingRcll");
	model_ = &model;

	logger->log_info(name(), "Intilized");
};

void
ClipsMipSchedulerThread::loop()
{
	logger->log_info(name(), "Looping");
};

/** Initialize the context and add a 'build-mip-model' CLIPS function.
 * @param env_name The name of the environment.
 * @param clips The CLIPS environment to add the parser functionality to.
 */
void
ClipsMipSchedulerThread::clips_context_init(const std::string &                  env_name,
                                            fawkes::LockPtr<CLIPS::Environment> &clips)
{
	logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());
	clips_envs_[env_name] = clips;
	fawkes::MutexLocker lock(clips_envs_[env_name].objmutex_ptr());

	clips->add_function("generate-datasets",
	                    sigc::slot<void>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::gen_datasets),
	                                    env_name)));
}

void
ClipsMipSchedulerThread::clips_context_destroyed(const std::string &env_name)
{
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
	clips_envs_.erase(env_name);
}

/** CLIPS function to build the MIPS model used for scheduling.
 * This generates the datasets and paramters of events from clips facts and
 * build the MIPS model
 * @param env_name The name of the calling environment
 */
void
ClipsMipSchedulerThread::gen_datasets(std::string env_name)
{
	logger->log_info(name(), "Generating datasets");
	fawkes::MutexLocker lock(clips_envs_[env_name].objmutex_ptr());
	CLIPS::Environment &env = **(clips_envs_[env_name]);

	CLIPS::Fact::pointer fact = env.get_facts();
	std::string          events_set;
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() != "wm-fact") {
			fact = fact->next();
			continue;
		}
		std::vector<std::string> slots = fact->slot_names();
		if (std::find(slots.begin(), slots.end(), "id") == slots.end()
		    || std::find(slots.begin(), slots.end(), "key") == slots.end()
		    || std::find(slots.begin(), slots.end(), "value") == slots.end()
		    || std::find(slots.begin(), slots.end(), "values") == slots.end()) {
			logger->log_info(name(), "Missing slot in wm-fact");
			throw fawkes::Exception("Missing slot in wm-fact");
		} else {
			CLIPS::Values slot_id_v     = fact->slot_value("id");
			CLIPS::Values slot_key_v    = fact->slot_value("key");
			CLIPS::Values slot_value_v  = fact->slot_value("value");
			CLIPS::Values slot_values_v = fact->slot_value("values");
			std::size_t   found         = slot_id_v[0].as_string().find("/scheduling/event");
			logger->log_info(name(), "Found somthing %li", found);
			if (found != std::string::npos) {
				logger->log_info(name(), slot_id_v[0].as_string().c_str());
				events_set += " " + slot_value_v[0].as_string();
			}
		}
		fact = fact->next();
	}
}
