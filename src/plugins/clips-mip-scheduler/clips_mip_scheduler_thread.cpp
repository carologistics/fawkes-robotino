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
	clips_env_ = clips;
	fawkes::MutexLocker lock(clips_env_.objmutex_ptr());

	clips->add_function("scheduler-add-event-resource",
	                    sigc::slot<void, std::string, std::string, int>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_add_event_resource),
	                      env_name)));

	clips->add_function("scheduler-add-event-precedence",
	                    sigc::slot<void, std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_add_event_precedence),
	                      env_name)));

	clips->add_function(
	  "scheduler-add-atomic-event",
	  sigc::slot<void, std::string, int, float, float>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_add_event), env_name)));
	clips->add_function("scheduler-set-selector-selected",
	                    sigc::slot<void, std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_set_selector_selected),
	                      env_name)));

	clips->add_function(
	  "scheduler-set-edge-duration",
	  sigc::slot<void, std::string, std::string, std::string, std::string, int>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_set_edge_duration),
	                  env_name)));

	clips->add_function(
	  "scheduler-add-edge-selector",
	  sigc::slot<void, std::string, std::string, std::string, std::string, std::string>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_add_edge_selector),
	                  env_name)));

	clips->add_function("scheduler-set-event-selector",
	                    sigc::slot<void, std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_set_event_selector),
	                      env_name)));

	clips->add_function(
	  "scheduler-add-to-select-all-group",
	  sigc::slot<void, std::string, std::string>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_add_to_select_all_group),
	                  env_name)));

	clips->add_function(
	  "scheduler-add-to-select-one-group",
	  sigc::slot<void, std::string, std::string>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_add_to_select_one_group),
	                  env_name)));

	clips->add_function(
	  "scheduler-generate-model",
	  sigc::slot<void, std::string, CLIPS::Values, CLIPS::Values>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_build_model), env_name)));

	clips->add_function("scheduler-optimization-status",
	                    sigc::slot<std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::clips_check_progress),
	                      env_name)));
}

void
ClipsMipSchedulerThread::clips_context_destroyed(const std::string &env_name)
{
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
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
ClipsMipSchedulerThread::create_event(std::string event_name)
{
	if (events_.find(event_name) != events_.end())
		return;
	events_[event_name] = std::make_shared<Event>(event_name);
	logger->log_info(name(), "Creating Event[%s] ", event_name.c_str());
}

void
ClipsMipSchedulerThread::create_edge(std::string resource_name,
                                     std::string resource_state,
                                     std::string event1,
                                     std::string event2)
{
	std::string edge_name = Edge::create_edge_name(resource_name, resource_state, event1, event2);
	if (edges_.find(edge_name) != edges_.end())
		return;

	Event_ptr event1_ptr = get_event(event1);
	Event_ptr event2_ptr = get_event(event2);
	Edge_ptr edge_ptr = std::make_shared<Edge>(resource_name, resource_state, event1_ptr, event2_ptr);

	edges_[edge_name] = edge_ptr;

	logger->log_info(name(), "Created Edge %s ", edge_name.c_str());
}

void
ClipsMipSchedulerThread::create_selector(std::string selector_name)
{
	if (selectors_.find(selector_name) != selectors_.end())
		return;
	selectors_[selector_name] = std::make_shared<Selector>(selector_name);
	logger->log_info(name(), "Creating selector %s ", selector_name.c_str());
}

ClipsMipSchedulerThread::Event_ptr
ClipsMipSchedulerThread::get_event(std::string event_name)
{
	if (events_.find(event_name) == events_.end())
		create_event(event_name);

	return events_[event_name];
}

ClipsMipSchedulerThread::Edge_ptr
ClipsMipSchedulerThread::get_edge(std::string resource_name,
                                  std::string resource_state,
                                  std::string event1,
                                  std::string event2)
{
	std::string edge_name = Edge::create_edge_name(resource_name, resource_state, event1, event2);
	if (edges_.find(edge_name) == edges_.end())
		create_edge(resource_name, resource_state, event1, event2);

	return edges_[edge_name];
}

ClipsMipSchedulerThread::Selector_ptr
ClipsMipSchedulerThread::get_selector(std::string selector_name)
{
	if (selectors_.find(selector_name) == selectors_.end())
		create_selector(selector_name);

	return selectors_[selector_name];
}

void
ClipsMipSchedulerThread::clips_add_event(std::string env_name,
                                         std::string event_name,
                                         int         duration,
                                         float       lbound,
                                         float       ubound)
{
	Event_ptr event = get_event(event_name);
	event->duration = duration;
	event->lbound   = lbound;
	event->ubound   = ubound;
	logger->log_info(name(), "Event[%s] duration %d ", event_name.c_str(), duration);
}

void
ClipsMipSchedulerThread::clips_add_event_resource(std::string env_name,
                                                  std::string event_name,
                                                  std::string res_name,
                                                  int         req)
{
	get_event(event_name)->resources[res_name] = abs(req);
	logger->log_info(
	  name(), "Resource-req: %s uses %s in %d units  ", event_name.c_str(), res_name.c_str(), req);
}

void
ClipsMipSchedulerThread::clips_add_event_precedence(std::string env_name,
                                                    std::string event_name,
                                                    std::string preceded)
{
	Event_ptr before = get_event(event_name);
	Event_ptr after  = get_event(preceded);
	precedence_[before].push_back(after);
	logger->log_info(name(),
	                 "Precedence: %s directly before %s  ",
	                 event_name.c_str(),
	                 preceded.c_str());
}

void
ClipsMipSchedulerThread::clips_set_event_selector(std::string env_name,
                                                  std::string event_name,
                                                  std::string selector_name)
{
	Event_ptr    event    = get_event(event_name);
	Selector_ptr selector = get_selector(selector_name);

	event->selector = selector;
	//selector_events_[selector].push_back(event);
}

void
ClipsMipSchedulerThread::clips_set_edge_duration(std::string env_name,
                                                 std::string resource_name,
                                                 std::string resource_state,
                                                 std::string event1,
                                                 std::string event2,
                                                 int         duration)
{
	Edge_ptr edge  = get_edge(resource_name, resource_state, event1, event2);
	edge->duration = duration;

	logger->log_info(name(), "Edge %s duration %d ", edge->name().c_str(), duration);
}

void
ClipsMipSchedulerThread::clips_add_edge_selector(std::string env_name,
                                                 std::string resource_name,
                                                 std::string resource_state,
                                                 std::string event1,
                                                 std::string event2,
                                                 std::string selector_name)
{
	Edge_ptr     edge     = get_edge(resource_name, resource_state, event1, event2);
	Selector_ptr selector = get_selector(selector_name);

	selector_edges_[selector].push_back(edge);
	edge->selectors.push_back(selector);
	//logger->log_info(name(), "Edge %s selector %s", edge->name().c_str(), selector_name.c_str());
}

void
ClipsMipSchedulerThread::clips_set_selector_selected(std::string env_name,
                                                     std::string selector_name,
                                                     std::string selected)
{
	get_selector(selector_name)->selected = (selected == "TRUE") ? true : false;
	//logger->log_info(name(), "Selector[%s]: is %s  ", selector_name.c_str(), selected.c_str());
}

void
ClipsMipSchedulerThread::clips_add_to_select_all_group(std::string env_name,
                                                       std::string selector_name,
                                                       std::string group_name)
{
	Selector_ptr group    = get_selector(group_name);
	Selector_ptr selector = get_selector(selector_name);

	select_all_groups_[group].push_back(selector);

	//logger->log_info(name(),
	//                 "SelectAllGroup[%s]: add selector [%s]  ",
	//                 group_name.c_str(),
	//                 selector_name.c_str());
}

void
ClipsMipSchedulerThread::clips_add_to_select_one_group(std::string env_name,
                                                       std::string selector_name,
                                                       std::string group_name)
{
	Selector_ptr group    = get_selector(group_name);
	Selector_ptr selector = get_selector(selector_name);

	select_one_groups_[group].push_back(selector);

	//logger->log_info(name(),
	//                 "SelectOneGroup[%s]: add selector [%s]  ",
	//                 group_name.c_str(),
	//                 selector_name.c_str());
}

void
ClipsMipSchedulerThread::clips_build_model(std::string   env_name,
                                           std::string   model_id,
                                           CLIPS::Values param_names,
                                           CLIPS::Values param_values)
{
	try {
		gurobi_models_[model_id] = std::make_unique<GRBModel>(*gurobi_env_);
		gurobi_models_[model_id]->set(GRB_StringAttr_ModelName, model_id);

		std::map<std::string, std::string> param_map;
		for (size_t i = 0; i < param_names.size(); ++i) {
			if (param_names[i].type() != CLIPS::TYPE_SYMBOL
			    && param_names[i].type() != CLIPS::TYPE_STRING) {
				logger->log_error(name(), "Param of scheduler is not a string or symbol");
				continue;
			}
			switch (param_values[i].type()) {
			case CLIPS::TYPE_FLOAT:
				param_map[param_names[i].as_string()] = std::to_string(param_values[i].as_float());
				break;
			case CLIPS::TYPE_INTEGER:
				param_map[param_names[i].as_string()] = std::to_string(param_values[i].as_integer());
				break;
			case CLIPS::TYPE_SYMBOL:
			case CLIPS::TYPE_STRING:
				param_map[param_names[i].as_string()] = param_values[i].as_string();
				break;
			default:
				logger->log_error(name(),
				                  "Param '%s' of scheduler of invalid type",
				                  param_names[i].as_string().c_str());
				break;
			}
		}

		for (auto const &iP : param_map) {
			gurobi_models_[model_id]->set(iP.first, iP.second);
			logger->log_info(name(),
			                 "Solver params: setting %s to %s ",
			                 iP.first.c_str(),
			                 iP.second.c_str());
		}

		//Init Gurobi Time Vars (T)
		for (auto const &iE : events_) {
			//logger->log_info(name(), "t[%s] ", iE.first.c_str());
			gurobi_vars_time_[iE.first] = gurobi_models_[model_id]->addVar(
			  iE.second->lbound, iE.second->ubound, 0, GRB_INTEGER, ("t[" + iE.first + "]").c_str());
		}
		//Init Gurobi event sequencing Vars (X)
		for (auto const &iE : edges_) {
			Edge_ptr e = iE.second;
			//logger->log_info(name(), "x %s", e->name.c_str());
			gurobi_vars_sequence_[e->resource_name][e->resource_state][e->from->name][e->to->name] =
			  gurobi_models_[model_id]->addVar(0, 1, 0, GRB_BINARY, ("x" + e->name()).c_str());
		}

		//Init Gurobi plan selection Vars (S)
		for (auto const &iP : selectors_) {
			//logger->log_info(name(), "s[%s]", iP.first.c_str());
			int lb = 0;
			if (iP.second->selected)
				lb = 1;
			gurobi_vars_plan_[iP.first] =
			  gurobi_models_[model_id]->addVar(lb, 1, 0, GRB_BINARY, ("s[" + iP.first + "]").c_str());
		}
		//Objective
		GRBVar Tmax = gurobi_models_[model_id]->addVar(0, GRB_INFINITY, 1, GRB_INTEGER, "Tmax");
		gurobi_models_[model_id]->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

		for (auto const &iS : selectors_) {
			if (iS.second->selected)
				gurobi_models_[model_id]->addConstr(gurobi_vars_plan_[iS.first] == 1,
				                                    ("SelectedEvent{" + iS.first + "}").c_str());
			//logger->log_info(name(), "C2: plans of goal %s", iS.first.c_str());
		}

		//Constraint 1
		for (auto const &iE1 : events_)
			for (auto const &iE2 : precedence_[iE1.second]) {
				//logger->log_info(name(),
				//                 "C1: %s - %s >= %u ",
				//                 iE2->name.c_str(),
				//                 iE1.second->name.c_str(),
				//                 iE1.second->duration);

				if (iE1.second->selector == iE2->selector) {
					//logger->log_info(name(), "Presd_InPlan: %s  ", iE1.second->plan.c_str() );
					gurobi_models_[model_id]->addConstr(
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    >= iE1.second->duration + 1,
					  ("PresdInPlan{" + iE1.second->name + "<<" + iE2->name + "}").c_str());

				} else if (iE1.second->selector->selected && iE2->selector->selected) {
					//logger->log_info(name(), "Presd_AcrossGoals: %s << %s ", iE1.second->goal.c_str() , iE2 -> goal.c_str() );
					gurobi_models_[model_id]->addConstr(
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    >= iE1.second->duration + 1,
					  ("PresdAcrossGoals{" + iE1.second->name + "<<" + iE2->name + "}").c_str());
				} else if (!(iE1.second->selector->selected) && iE2->selector->selected
				           && iE1.second->selector != iE2->selector) {
					//logger->log_info(name(), "Presd_PlanGoal_End: %s << %s ", iE1.second->plan.c_str() , iE2 -> goal.c_str() );
					gurobi_models_[model_id]->addGenConstrIndicator(
					  gurobi_vars_plan_[iE1.second->selector->name],
					  1,
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    == iE1.second->duration,
					  ("PresdPlanGoal_End{" + iE1.second->name + "<<" + iE2->name + "}").c_str());
				} else if (iE1.second->selector->selected && !(iE2->selector->selected)
				           && iE1.second->selector != iE2->selector) {
					// logger->log_info(name(), "Presd_PlanGoal_Start: %s << %s ", iE1.second->goal.c_str() , iE2 -> plan.c_str() );
					gurobi_models_[model_id]->addGenConstrIndicator(
					  gurobi_vars_plan_[iE2->selector->name],
					  1,
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    == iE1.second->duration,
					  ("PresdPlanGoal_Start{" + iE1.second->name + "<<" + iE2->name + "}").c_str());
				}
			}

		//Constraint (2) Select-one groups
		for (auto const &iG : select_one_groups_) {
			GRBLinExpr RHS = gurobi_vars_plan_[iG.first->name];
			GRBLinExpr LHS = 0;
			for (auto const &iP : iG.second)
				LHS += gurobi_vars_plan_[iP->name];

			gurobi_models_[model_id]->addConstr(LHS - RHS == 0,
			                                    ("SelectOneGroup{" + iG.first->name + "}").c_str());
		}

		//Constraint (3) Select-all groups
		for (auto const &iG : select_all_groups_) {
			GRBLinExpr RHS = gurobi_vars_plan_[iG.first->name];
			GRBLinExpr LHS = 0;
			for (auto const &iP : iG.second) {
				LHS = gurobi_vars_plan_[iP->name];
				gurobi_models_[model_id]->addConstr(LHS - RHS == 0,
				                                    ("SelectAllGroup{" + iG.first->name + "}").c_str());
			}
		}

		//Edges Selection GenConstrIndicator
		//for (auto const &ie : edges_)
		//{
		//	Edge *      e              = ie.second;
		//	std::string edge_name      = ie.first;
		//	std::string resource       = e->resource;
		//	std::string producer       = e->from->name;
		//	std::string consumer       = e->to->name;
		//	std::string cname          = "Select_X" + edge_name;
		//	int         event_duration = e->from->duration;
		//	double      setup_duration = e->duration;
		//	gurobi_models_[model_id]->addGenConstrIndicator(
		//	  gurobi_vars_sequence_[resource][producer][consumer],
		//	  1,
		//	  gurobi_vars_time_[consumer] - gurobi_vars_time_[producer] - event_duration - setup_duration
		//	    >= 0,
		//	  cname.c_str());
		//logger->log_info(name(), "C8: %s", cname.c_str());
		//}

		//Constrs : Consumers edge-selectors
		for (auto const &is : selector_edges_) {
			std::string selector_name = is.first->name;
			int         l             = is.second.size();
			double      sosWieghts[l];
			GRBVar      sosVars[l];
			GRBLinExpr  constr_LHS = 0;
			GRBLinExpr  constr_RHS = gurobi_vars_plan_[selector_name];
			int         i          = 0;
			for (auto const &ie : is.second) {
				std::string resource_name  = ie->resource_name;
				std::string resource_state = ie->resource_state;
				std::string producer       = ie->from->name;
				std::string consumer       = ie->to->name;
				GRBVar edgeVar = gurobi_vars_sequence_[resource_name][resource_state][producer][consumer];
				constr_LHS += edgeVar;
				sosVars[i]    = edgeVar;
				sosWieghts[i] = i;
				i++;
			}
			//logger->log_info(name(), "flow %s ", selector_name.c_str());
			gurobi_models_[model_id]->addConstr(constr_LHS - constr_RHS == 0,
			                                    ("EdgeSelector " + selector_name).c_str());
			//SOS
			if (solver_params_.find("SOS") != solver_params_.end() && solver_params_["SOS"] == "ENABLE")
				gurobi_models_[model_id]->addSOS(sosVars, sosWieghts, i, GRB_SOS_TYPE1);
		}

		//Constraints 8,3,4,5,6
		for (auto const &iR : gurobi_vars_sequence_) {
			std::map<std::string, std::vector<GRBVar>> inflow_vars;
			std::map<std::string, std::vector<GRBVar>> outflow_vars;
			std::string                                resource_name = iR.first;
			//Edges Selection GenConstrIndicator
			for (auto const &iRStates : iR.second) {
				std::string resource_state = iRStates.first;
				for (auto const &iEprod : iRStates.second) {
					std::string producer = iEprod.first;
					for (auto const &iEcons : iEprod.second) {
						std::string consumer = iEcons.first;
						GRBVar      edgeVar  = iEcons.second;
						Edge_ptr    edge     = get_edge(resource_name, resource_state, producer, consumer);
						std::string cname    = "EdgeSelection_X" + edge->name();
						int         event_duration = get_event(producer)->duration;
						double      setup_duration = edge->duration;
						gurobi_models_[model_id]->addGenConstrIndicator(edgeVar,
						                                                1,
						                                                gurobi_vars_time_[consumer]
						                                                    - gurobi_vars_time_[producer]
						                                                    - event_duration - setup_duration
						                                                  >= 1,
						                                                cname.c_str());
						//logger->log_info(name(), "C8: %s", cname.c_str());
						inflow_vars[consumer].push_back(edgeVar);
						outflow_vars[producer].push_back(edgeVar);
					}
				}
			}

			//Constrs: Flowout == FlowIn
			for (auto const &iE : events_) {
				std::string event_name    = iE.first;
				GRBLinExpr  expr_out      = 0;
				GRBLinExpr  expr_in       = 0;
				GRBLinExpr  expr_selector = gurobi_vars_plan_[events_[event_name]->selector->name];

				if (outflow_vars.find(event_name) != outflow_vars.end())
					for (auto const &iFlowVar : outflow_vars[event_name])
						expr_out += iFlowVar;
				else
					expr_out = abs(events_[event_name]->resources[iR.first]);

				if (inflow_vars.find(event_name) != inflow_vars.end())
					for (auto const &iFlowVar : inflow_vars[event_name])
						expr_in += iFlowVar;
				else
					expr_in = abs(events_[event_name]->resources[iR.first]);

				//logger->log_info(name(), "FlowConservation %s ", event_name.c_str());
				gurobi_models_[model_id]->addConstr(
				  expr_in == expr_out, ("FlowConservation[" + iR.first + "][" + event_name + "]").c_str());

				if (outflow_vars.find(event_name) != outflow_vars.end()) {
					//logger->log_info(name(), "SelectableFlowOut %s ", event_name.c_str());
					gurobi_models_[model_id]->addConstr(
					  expr_out == expr_selector * abs(events_[event_name]->resources[iR.first]),
					  ("FlowOut[" + iR.first + "][" + event_name + "]").c_str());
				}

				if (inflow_vars.find(event_name) != inflow_vars.end()) {
					//logger->log_info(name(), "SelectableFlowIn %s ", event_name.c_str());
					gurobi_models_[model_id]->addConstr(
					  expr_in == expr_selector * abs(events_[event_name]->resources[iR.first]),
					  ("FlowIn[" + iR.first + "][" + event_name + "]").c_str());
				}
			}

			//SOS1
			if (solver_params_.find("SOS") != solver_params_.end() && solver_params_["SOS"] == "ENABLE") {
				//Constrs : Consumers in-flow SOS1
				for (auto const &iCons : inflow_vars) {
					std::string event_name = iCons.first;
					int         l          = inflow_vars[iCons.first].size();
					double      sosWieghts[l];
					GRBVar      sosVars[l];
					int         i = 0;
					for (auto const &iFlowVar : iCons.second) {
						sosVars[i]    = iFlowVar;
						sosWieghts[i] = i;
						i++;
					}
					if (abs(events_[event_name]->resources[iR.first]) == 1)
						gurobi_models_[model_id]->addSOS(sosVars, sosWieghts, i, GRB_SOS_TYPE1);
				}

				//Constrs: Producers out-flow SOS1
				for (auto const &iProd : outflow_vars) {
					std::string event_name = iProd.first;
					int         l          = outflow_vars[iProd.first].size();
					double      sosWieghts[l];
					GRBVar      sosVars[l];
					int         i = 0;
					for (auto const &iFlowVar : iProd.second) {
						sosVars[i]    = iFlowVar;
						sosWieghts[i] = i;
						i++;
					}
					if (abs(events_[event_name]->resources[iR.first]) == 1)
						gurobi_models_[model_id]->addSOS(sosVars, sosWieghts, i, GRB_SOS_TYPE1);
				}
			}
		}

		//Constraint 10
		for (auto const &iE : events_)
			gurobi_models_[model_id]->addConstr(Tmax - gurobi_vars_time_[iE.first] >= 1,
			                                    ("Tmax<<" + iE.first).c_str());

		gurobi_models_[model_id]->write("gurobi_model.lp");

		gurobi_models_[model_id]->optimizeasync();
	} catch (GRBException &e) {
		gurobi_models_[model_id]->write("gurobi_model.lp");
		logger->log_error(name(), "Error code = %u ", e.getErrorCode());
		logger->log_error(name(), e.getMessage().c_str());
	} catch (...) {
		logger->log_error(name(), "Exception during optimization");
	}
}

std::string
ClipsMipSchedulerThread::clips_check_progress(std::string env_name, std::string model_id)
{
	try {
		const int status = gurobi_models_[model_id]->get(GRB_IntAttr_Status);

		if ((status == GRB_LOADED)) {
			logger->log_warn(name(), "Model is loaded, but no solution information available %u", status);
			return "LOADED";
		}

		if ((status == GRB_INPROGRESS)) {
			logger->log_warn(name(), "Optimization in progress %u", status);
			return "INPROGRESS";
		} else {
			logger->log_warn(name(), "Optimization NOT in progress %u", status);
		}

		if (status == GRB_UNBOUNDED) {
			logger->log_warn(name(), "The model cannot be solved  because it is unbounded");
		}

		if (status == GRB_OPTIMAL || status == GRB_TIME_LIMIT) {
			gurobi_models_[model_id]->sync();

			logger->log_warn(name(),
			                 "The optimal objective is %f ",
			                 gurobi_models_[model_id]->get(GRB_DoubleAttr_ObjVal));
			gurobi_models_[model_id]->write("gurobi_model.sol");

			//Post result facts
			fawkes::MutexLocker lock(clips_env_.objmutex_ptr());
			CLIPS::Environment &env = **(clips_env_);

			//Post process Time Vars (T)
			for (auto const &iE : gurobi_vars_time_) {
				std::string type  = "EVENT-TIME";
				int         value = int(iE.second.get(GRB_DoubleAttr_X));
				env.assert_fact_f("(scheduler-info (type %s) (descriptors %s )(value  %d ))",
				                  type.c_str(),
				                  iE.first.c_str(),
				                  value);
			}

			//Post process sequencing Vars (X)
			for (auto const &iR : gurobi_vars_sequence_)
				for (auto const &iRStates : iR.second)
					for (auto const &iEprod : iRStates.second)
						for (auto const &iEcons : iEprod.second) {
							std::string type  = "EVENT-SEQUENCE";
							int         value = int(iEcons.second.get(GRB_DoubleAttr_X));
							env.assert_fact_f("(scheduler-info (type %s) (descriptors %s %s %s %s) (value  %d ))",
							                  type.c_str(),
							                  iR.first.c_str(),
							                  iRStates.first.c_str(),
							                  iEprod.first.c_str(),
							                  iEcons.first.c_str(),
							                  value);
						}

			//Post process plan selection Vars (S)
			for (auto const &iP : gurobi_vars_plan_) {
				std::string type  = "PLAN-SELECTION";
				int         value = int(iP.second.get(GRB_DoubleAttr_X));
				env.assert_fact_f("(scheduler-info (type %s) (descriptors %s ) (value  %d ))",
				                  type.c_str(),
				                  iP.first.c_str(),
				                  value);
			}

			gurobi_models_[model_id]->reset();
			return "OPTIMAL";
		}

		if ((status != GRB_INF_OR_UNBD) && (status != GRB_INFEASIBLE)) {
			logger->log_warn(name(), "Optimization was stopped with status %u", status);
		}

		if (status == GRB_INFEASIBLE) {
			// do IIS
			logger->log_info(name(), "The model is infeasible; computing IIS");
			gurobi_models_[model_id]->sync();

			gurobi_models_[model_id]->computeIIS();
			logger->log_info(name(), "The following constraint(s) cannot be satisfied:");
			GRBConstr *c = gurobi_models_[model_id]->getConstrs();
			for (int i = 0; i < gurobi_models_[model_id]->get(GRB_IntAttr_NumConstrs); ++i)
				if (c[i].get(GRB_IntAttr_IISConstr) == 1)
					logger->log_info(name(), c[i].get(GRB_StringAttr_ConstrName).c_str());

			delete[] c;
			return "INFEASIBLE";
		}

		if ((status != GRB_INF_OR_UNBD)) {
			return "INF_OR_UNBD";
		}

		//gurobi_models_[model_id]->write("gurobi_model.rlp");
		//gurobi_models_[model_id]->write("gurobi_model.ilp");
		//gurobi_models_[model_id]->write("gurobi_model.rew");

		//gurobi_models_[model_id]->write("gurobi_model.hnt");
		//gurobi_models_[model_id]->write("gurobi_model.bas");
		//gurobi_models_[model_id]->write("gurobi_model.prm");
		//gurobi_models_[model_id]->write("gurobi_model.attr");
		//gurobi_models_[model_id]->write("gurobi_model.json");
	} catch (GRBException &e) {
		logger->log_error(name(), "Error code = %u ", e.getErrorCode());
		logger->log_error(name(), e.getMessage().c_str());
	} catch (...) {
		logger->log_error(name(), "Exception during optimization");
	}
	return "";
};
