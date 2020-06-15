/***************************************************************************
 *  clips_mips_scheduler_thread.h - CLIPS feature for scheduling with MIPS
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

#ifndef _PLUGINS_CLIPS_MIP_SCHEDULER_THREAD_H_
#define _PLUGINS_CLIPS_MIP_SCHEDULER_THREAD_H_

#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <clipsmm.h>
#include <gurobi_c++.h>
#include <map>
#include <string>

class ClipsMipSchedulerThread : public fawkes::Thread,
                                public fawkes::LoggingAspect,
                                public fawkes::CLIPSFeature,
                                public fawkes::CLIPSFeatureAspect
{
public:
	ClipsMipSchedulerThread();
	virtual ~ClipsMipSchedulerThread();

	virtual void init();
	virtual void loop();
	virtual void finalize(){};

	virtual void clips_context_init(const std::string &                  env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::LockPtr<CLIPS::Environment> clips_env_;

private:
	GRBEnv *                                         gurobi_env_ = 0;
	std::map<std::string, std::unique_ptr<GRBModel>> gurobi_models_;

	//struct Entity{
	//    std::string name;
	//    enum type {RESOURCE, GOAL, PLAN}
	//};

	struct Event
	{
		Event(std::string n)
		{
			name = n;
		};
		std::string                name;
		float                      duration = 0;
		float                      lbound   = 0;
		float                      ubound   = GRB_INFINITY;
		std::map<std::string, int> resources;
		std::vector<Event *>       precedes;
		std::string                goal = "";
		std::string                plan = "";
		//    enum at {START, END};
		//    Entity entity;
	};

	std::map<std::string, Event *>                                      events_;
	std::map<std::string, std::map<Event *, std::map<Event *, double>>> res_setup_duration_;
	std::map<std::string, std::vector<Event *>>                         plan_events_;
	std::map<std::string, std::vector<Event *>>                         goal_events_;
	std::map<std::string, std::vector<std::string>>                     goal_plans_;
	std::map<std::string, std::string>                                  plan_goal_;

	std::map<std::string, GRBVar>                                               gurobi_vars_time_;
	std::map<std::string, GRBVar>                                               gurobi_vars_plan_;
	std::map<std::string, std::map<std::string, std::map<std::string, GRBVar>>> gurobi_vars_sequence_;

private:
	void set_event_duration(std::string env_name, std::string event_name, float duration);
	void set_event_bounds(std::string env_name, std::string event_name, float lb, float ub);
	void
	     add_event_resource(std::string env_name, std::string event_name, std::string res_name, int req);
	void set_resource_setup_duration(std::string env_name,
	                                 std::string res,
	                                 std::string event1,
	                                 std::string event2,
	                                 double      duration);
	void add_event_precedence(std::string env_name, std::string event_name, std::string preceded);
	void add_plan_event(std::string env_name, std::string plan_name, std::string event_name);
	void add_goal_event(std::string env_name, std::string goal_name, std::string event_name);
	void add_goal_plan(std::string env_name, std::string goal_name, std::string plan_name);
	void build_model(std::string env_name, std::string model_id);
	void check_progress(std::string env_name, std::string model_id);
};
#endif /* !PLUGINS_CLIPS_MIP_SCHEDULER_THREAD_H__ */
