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
	gurobi_env_ = new GRBEnv("mip_log.lp");
}

ClipsMipSchedulerThread::~ClipsMipSchedulerThread()
{
	clips_envs_.clear();
	delete gurobi_model_;
	delete gurobi_env_;
}

void
ClipsMipSchedulerThread::init()
{
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

	clips->add_function(
	  "scheduler-set-event-location",
	  sigc::slot<void, std::string, std::string>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::set_event_location), env_name)));
	clips->add_function(
	  "scheduler-set-event-duration",
	  sigc::slot<void, std::string, int>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::set_event_duration), env_name)));
	clips->add_function(
	  "scheduler-add-event-resource",
	  sigc::slot<void, std::string, std::string, int>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::add_event_resource), env_name)));
	clips->add_function("scheduler-add-event-precedence",
	                    sigc::slot<void, std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::add_event_precedence),
	                      env_name)));
	clips->add_function("scheduler-add-plan-event",
	                    sigc::slot<void, std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::add_plan_event), env_name)));
	clips->add_function("scheduler-add-goal-event",
	                    sigc::slot<void, std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::add_goal_event), env_name)));
	clips->add_function("scheduler-add-goal-plan",
	                    sigc::slot<void, std::string, std::string>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::add_goal_plan),
	                                    env_name)));
	clips->add_function("scheduler-generate-model",
	                    sigc::slot<void>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::build_model),
	                                    env_name)));
}

void
ClipsMipSchedulerThread::clips_context_destroyed(const std::string &env_name)
{
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
	clips_envs_.erase(env_name);
}

void
ClipsMipSchedulerThread::set_event_location(std::string env_name,
                                            std::string event_name,
                                            std::string location)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	events_[event_name]->location = location;
}

//{
//	logger->log_info(name(), "Generating datasets");
//	fawkes::MutexLocker lock(clips_envs_[env_name].objmutex_ptr());
//	CLIPS::Environment &env = **(clips_envs_[env_name]);
//
//	CLIPS::Fact::pointer fact = env.get_facts();
//	std::string          events_set;
//	while (fact) {
//		CLIPS::Template::pointer tmpl = fact->get_template();
//		if (tmpl->name() != "wm-fact") {
//			fact = fact->next();
//			continue;
//		}
//		std::vector<std::string> slots = fact->slot_names();
//		if (std::find(slots.begin(), slots.end(), "id") == slots.end()
//		    || std::find(slots.begin(), slots.end(), "key") == slots.end()
//		    || std::find(slots.begin(), slots.end(), "value") == slots.end()
//		    || std::find(slots.begin(), slots.end(), "values") == slots.end()) {
//			logger->log_info(name(), "Missing slot in wm-fact");
//			throw fawkes::Exception("Missing slot in wm-fact");
//		} else {
//			CLIPS::Values slot_id_v     = fact->slot_value("id");
//			CLIPS::Values slot_key_v    = fact->slot_value("key");
//			CLIPS::Values slot_value_v  = fact->slot_value("value");
//			CLIPS::Values slot_values_v = fact->slot_value("values");
//			std::size_t   found         = slot_id_v[0].as_string().find("/scheduling/event");
//			logger->log_info(name(), "Found somthing %li", found);
//			if (found != std::string::npos) {
//				logger->log_info(name(), slot_id_v[0].as_string().c_str());
//				events_set += " " + slot_value_v[0].as_string();
//			}
//		}
//		fact = fact->next();
//	}
//}

void
ClipsMipSchedulerThread::set_event_duration(std::string env_name,
                                            std::string event_name,
                                            int         duration)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	events_[event_name]->duration = duration;
}

void
ClipsMipSchedulerThread::add_event_resource(std::string env_name,
                                            std::string event_name,
                                            std::string res_name,
                                            int         req)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	events_[event_name]->resources[res_name] = req;

	if (req >= 0)
		resource_producers_[res_name].push_back(events_[event_name]);
	else
		resource_consumers_[res_name].push_back(events_[event_name]);
}

void
ClipsMipSchedulerThread::add_event_precedence(std::string env_name,
                                              std::string event_name,
                                              std::string preceded)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	if (events_.find(preceded) == events_.end())
		events_[preceded] = new Event(preceded);

	events_[event_name]->precedes.push_back(events_[preceded]);
}

void
ClipsMipSchedulerThread::add_plan_event(std::string env_name,
                                        std::string plan_name,
                                        std::string event_name)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	plan_events_[plan_name].push_back(events_[event_name]);
	events_[event_name]->plan = plan_name;
}

void
ClipsMipSchedulerThread::add_goal_event(std::string env_name,
                                        std::string goal_name,
                                        std::string event_name)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	goal_events_[goal_name].push_back(events_[event_name]);
	events_[event_name]->goal = goal_name;
}

void
ClipsMipSchedulerThread::add_goal_plan(std::string env_name,
                                       std::string goal_name,
                                       std::string plan_name)
{
	goal_plans_[goal_name].push_back(plan_name);
}

void
ClipsMipSchedulerThread::build_model(std::string env_name)
{
	try {
		gurobi_model_ = new GRBModel(*gurobi_env_);
		gurobi_model_->set(GRB_StringAttr_ModelName, "SchedulingRcll");
	} catch (GRBException e) {
	}
	//Init Gurobi Vars
	for (auto const &iE : events_)
		gurobi_vars_time_[iE.first] =
		  gurobi_model_->addVar(0, 900, 1, GRB_INTEGER, ("T_" + iE.first).c_str());

	for (auto const &iR : resource_producers_)
		for (auto const &iEp : resource_producers_[iR.first])
			for (auto const &iEc : resource_consumers_[iR.first])
				if (iEp->goal != iEc->goal || iEp->goal.size() == 0)
					gurobi_vars_sequence_[iEp->name][iEc->name] = gurobi_model_->addVar(
					  0, 1, 0, GRB_BINARY, ("X_" + iR.first + "_" + iEp->name + "." + iEc->name).c_str());

	for (auto const &iP : plan_events_)
		gurobi_vars_selection_[iP.first] =
		  gurobi_model_->addVar(0, 1, 0, GRB_BINARY, ("P_" + iP.first).c_str());

	//Constraint 1
	for (auto const &iE1 : events_)
		for (auto const &iE2 : iE1.second->precedes) {
			logger->log_info(name(), (iE1.first + "-->" + iE2->name + ",").c_str());
			gurobi_model_->addConstr(gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
			                           >= iE1.second->duration,
			                         ("PRES_" + iE1.second->name + "<" + iE2->name).c_str());
		}
}
