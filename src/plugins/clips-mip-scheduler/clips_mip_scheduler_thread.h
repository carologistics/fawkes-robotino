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
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> clips_envs_;

	GRBEnv *  gurobi_env_ = 0;
	GRBModel *model_      = 0;

<<<<<<< HEAD
	struct Event
	{
		Event(std::string n)
		{
			name = n;
		};
		std::string                name;
		std::string                location;
		int                        duration;
		std::map<std::string, int> resources;
		std::vector<Event *>       precedes;
=======
     struct Event {
         Event(std::string n)  { name = n; };
         std::string                    name;
         std::string                    location;
         int                            duration;
         std::map<std::string,int>      resources;
         std::vector<Event*>            precedes;

         GRBVar*                         time = 0;
         std::string                    goal = "";
         std::string                    plan = "";
     };
>>>>>>> fba35ed17... clips_mip_scheduler: Event trackes its goal and plan (for easier access)

		GRBVar *time = 0;
	};

	std::map<std::string, Event *> events_;

	std::map<std::string, std::vector<Event *>> resource_producers_;
	std::map<std::string, std::vector<Event *>> resource_consumers_;
	std::map<std::string, std::vector<Event *>> plan_events_;
	std::map<std::string, std::vector<Event *>> goal_events_;

	std::map<std::string, std::vector<std::string>> goal_plans_;

private:
	void set_event_location(std::string env_name, std::string event_name, std::string location);
	void set_event_duration(std::string env_name, std::string event_name, int duration);
	void
	     add_event_resource(std::string env_name, std::string event_name, std::string res_name, int req);
	void add_event_precedence(std::string env_name, std::string event_name, std::string preceded);
	void add_plan_event(std::string env_name, std::string plan_name, std::string event_name);
	void add_goal_event(std::string env_name, std::string goal_name, std::string event_name);
	void add_goal_plan(std::string env_name, std::string goal_name, std::string plan_name);
};
#endif /* !PLUGINS_CLIPS_MIP_SCHEDULER_THREAD_H__ */
