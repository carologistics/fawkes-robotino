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
	struct Selector
	{
		Selector(std::string n)
		{
			name     = n;
			selected = false;
		};
		std::string name;
		bool        selected;
	};

	typedef std::shared_ptr<Selector> Selector_ptr;
	void                              create_selector(std::string selector_name);
	Selector_ptr                      get_selector(std::string selector_name);

private:
	struct Event
	{
		Event(std::string event)
		{
			name = event;
		};
		std::string                name;
		int                        duration = 0;
		float                      lbound   = 0;
		float                      ubound   = GRB_INFINITY;
		std::map<std::string, int> resources;
		Selector_ptr               selector;
	};

	typedef std::shared_ptr<Event> Event_ptr;
	void                           create_event(std::string event_name);
	Event_ptr                      get_event(std::string event_name);

private:
	struct Edge
	{
		Edge(std::string res_name, std::string res_state, Event_ptr from_event, Event_ptr to_event)
		{
			resource_name  = res_name;
			resource_state = res_state;
			from           = from_event;
			to             = to_event;
		};

		static std::string
		create_edge_name(std::string res_name,
		                 std::string res_state,
		                 std::string from_name,
		                 std::string to_name)
		{
			return "[" + res_name + "][" + res_state + "][" + from_name + "][" + to_name + "]";
		};

		std::string               resource_name;
		std::string               resource_state;
		Event_ptr                 from;
		Event_ptr                 to;
		int                       duration;
		std::vector<Selector_ptr> selectors;

		std::string
		name()
		{
			return create_edge_name(resource_name, resource_state, from->name, to->name);
		};
	};

	typedef std::shared_ptr<Edge> Edge_ptr;
	void                          create_edge(std::string resource_name,
	                                          std::string resource_state,
	                                          std::string event1,
	                                          std::string event2);
	Edge_ptr                      get_edge(std::string resource_name,
	                                       std::string resource_state,
	                                       std::string event1,
	                                       std::string event2);

private:
	std::map<std::string, Event_ptr>    events_;
	std::map<std::string, Selector_ptr> selectors_;
	std::map<std::string, Edge_ptr>     edges_;

	std::map<Event_ptr, std::vector<Event_ptr>>       precedence_;
	std::map<Selector_ptr, std::vector<Edge_ptr>>     selector_edges_;
	std::map<Selector_ptr, std::vector<Selector_ptr>> select_one_groups_;
	std::map<Selector_ptr, std::vector<Selector_ptr>> select_all_groups_;

private:
	fawkes::LockPtr<CLIPS::Environment> clips_env_;

	void clips_add_event(std::string env_name,
	                     std::string event_name,
	                     int         duraton,
	                     float       lbound,
	                     float       ubound);
	void clips_add_event_resource(std::string env_name,
	                              std::string event_name,
	                              std::string res_name,
	                              int         req);
	void
	clips_add_event_precedence(std::string env_name, std::string event_name, std::string preceded);
	void
	     clips_set_event_selector(std::string env_name, std::string event_name, std::string selector_name);
	void clips_set_edge_duration(std::string env_name,
	                             std::string res_name,
	                             std::string res_state,
	                             std::string event1,
	                             std::string event2,
	                             int         duration);
	void clips_add_edge_selector(std::string env_name,
	                             std::string res_name,
	                             std::string res_state,
	                             std::string event1,
	                             std::string event2,
	                             std::string selector_name);
	void clips_set_selector_selected(std::string env_name,
	                                 std::string selector_name,
	                                 std::string selected);
	void clips_add_to_select_one_group(std::string env_name,
	                                   std::string selector_name,
	                                   std::string group_name);
	void clips_add_to_select_all_group(std::string env_name,
	                                   std::string selector_name,
	                                   std::string group_name);

private:
	GRBEnv *                                         gurobi_env_ = 0;
	std::map<std::string, std::unique_ptr<GRBModel>> gurobi_models_;

	std::map<std::string, GRBVar> gurobi_vars_time_;
	std::map<std::string, GRBVar> gurobi_vars_plan_;
	//EdgeVars[resource_name][resource_state][from_name][to_name]
	std::map<std::string, std::map<std::string, std::map<std::string, std::map<std::string, GRBVar>>>>
	  gurobi_vars_sequence_;

	void        clips_build_model(std::string env_name, std::string model_id);
	std::string clips_check_progress(std::string env_name, std::string model_id);
};
#endif /* !PLUGINS_CLIPS_MIP_SCHEDULER_THREAD_H__ */
