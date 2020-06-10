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

	clips->add_function(
	  "scheduler-set-event-duration",
	  sigc::slot<void, std::string, int>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::set_event_duration), env_name)));
	clips->add_function(
	  "scheduler-add-event-resource",
	  sigc::slot<void, std::string, std::string, int>(
	    sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::add_event_resource), env_name)));
	clips->add_function("scheduler-set-resource-setup-duration",
	                    sigc::slot<void, std::string, std::string, std::string, double>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::set_resource_setup_duration),
	                      env_name)));
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
	                    sigc::slot<void, std::string>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &ClipsMipSchedulerThread::build_model),
	                                    env_name)));
	clips->add_function("scheduler-optimization-status",
	                    sigc::slot<void, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsMipSchedulerThread::check_progress), env_name)));
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
ClipsMipSchedulerThread::set_event_duration(std::string env_name,
                                            std::string event_name,
                                            int         duration)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	events_[event_name]->duration = duration;

	logger->log_info(name(), "Duration: %s takes %u sec  ", event_name.c_str(), duration);
}

void
ClipsMipSchedulerThread::add_event_resource(std::string env_name,
                                            std::string event_name,
                                            std::string res_name,
                                            int         req)
{
	if (events_.find(event_name) == events_.end())
		events_[event_name] = new Event(event_name);

	events_[event_name]->resources[res_name] = abs(req);

	logger->log_info(
	  name(), "Resource-req: %s uses %s in %d units  ", event_name.c_str(), res_name.c_str(), req);
}

void
ClipsMipSchedulerThread::set_resource_setup_duration(std::string env_name,
                                                     std::string res,
                                                     std::string event1,
                                                     std::string event2,
                                                     double      duration)
{
	if (events_.find(event1) == events_.end())
		events_[event1] = new Event(event1);

	if (events_.find(event2) == events_.end())
		events_[event2] = new Event(event2);

	res_setup_duration_[res][events_[event1]][events_[event2]] = duration;

	logger->log_info(name(),
	                 " Setup [%s]: from %s to %s takes %lf sec",
	                 res.c_str(),
	                 event1.c_str(),
	                 event2.c_str(),
	                 duration);
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

	logger->log_info(name(),
	                 "Precedence: %s directly before %s  ",
	                 event_name.c_str(),
	                 preceded.c_str());
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

	logger->log_info(name(), "Event[plan]: %s in %s  ", event_name.c_str(), plan_name.c_str());
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

	logger->log_info(name(), "Event[goal]: %s in %s  ", event_name.c_str(), goal_name.c_str());
}

void
ClipsMipSchedulerThread::add_goal_plan(std::string env_name,
                                       std::string goal_name,
                                       std::string plan_name)
{
	goal_plans_[goal_name].push_back(plan_name);
	plan_goal_[plan_name].push_back(goal_name);

	logger->log_info(name(), "Plan: %s achives Goal %s  ", plan_name.c_str(), goal_name.c_str());
}

void
ClipsMipSchedulerThread::build_model(std::string env_name, std::string model_id)
{
	try {
		gurobi_models_[model_id] = std::make_unique<GRBModel>(*gurobi_env_);
		gurobi_models_[model_id]->set(GRB_StringAttr_ModelName, model_id);
		//Init Gurobi Time Vars (T)
		for (auto const &iE : events_)
			gurobi_vars_time_[iE.first] = gurobi_models_[model_id]->addVar(
			  0, GRB_INFINITY, 0, GRB_INTEGER, ("t{" + iE.first + "}").c_str());

		//Init Gurobi event sequencing Vars (X)
		for (auto const &iR : res_setup_duration_)
			for (auto const &iEprod : iR.second)
				for (auto const &iEcons : iEprod.second)
				//if (iEp->goal != iEc->goal || iEp->goal.size() == 0)
				{
					std::string resource = iR.first;
					std::string producer = iEprod.first->name;
					std::string consumer = iEcons.first->name;
					std::string vname    = "{" + resource + "}{" + producer + "__" + consumer + "}";
					gurobi_vars_sequence_[resource][producer][consumer] =
					  gurobi_models_[model_id]->addVar(0, 1, 0, GRB_BINARY, ("x" + vname).c_str());
				}

		//Init Gurobi plan selection Vars (S)
		for (auto const &iP : plan_events_)
			gurobi_vars_plan_[iP.first] =
			  gurobi_models_[model_id]->addVar(0, 1, 0, GRB_BINARY, ("p{" + iP.first + "}").c_str());

		//Objective
		GRBVar Tmax = gurobi_models_[model_id]->addVar(0, GRB_INFINITY, 1, GRB_INTEGER, "Tmax");
		gurobi_models_[model_id]->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

		//Constraint 1
		for (auto const &iE1 : events_)
			for (auto const &iE2 : iE1.second->precedes) {
				//logger->log_info(name(),
				//                 "C1: %s - %s >= %u ",
				//                 iE2->name.c_str(),
				//                 iE1.second->name.c_str(),
				//                 iE1.second->duration);

				if (iE1.second->plan == iE2->plan && iE2->plan != ""
				    && plan_events_.find(iE1.second->plan) != plan_events_.end()
				    && plan_events_.find(iE2->plan) != plan_events_.end()) {
					//logger->log_info(name(), "Presd_InPlan: %s  ", iE1.second->plan.c_str() );
					gurobi_models_[model_id]->addConstr(
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    >= iE1.second->duration,
					  ("PresdInPlan{" + iE1.second->name + "<<" + iE2->name + "}").c_str());

				} else if (iE1.second->goal != ""
				           && goal_events_.find(iE1.second->goal) != goal_events_.end()
				           && goal_events_.find(iE2->goal) != goal_events_.end()) {
					//logger->log_info(name(), "Presd_AcrossGoals: %s << %s ", iE1.second->goal.c_str() , iE2 -> goal.c_str() );
					gurobi_models_[model_id]->addConstr(
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    >= iE1.second->duration,
					  ("PresdAcrossGoals{" + iE1.second->name + "<<" + iE2->name + "}").c_str());
				} else if ((plan_events_.find(iE1.second->plan) != plan_events_.end()
				            && goal_events_.find(iE2->goal) != goal_events_.end())) {
					//logger->log_info(name(), "Presd_PlanGoal_End: %s << %s ", iE1.second->plan.c_str() , iE2 -> goal.c_str() );
					gurobi_models_[model_id]->addGenConstrIndicator(
					  gurobi_vars_plan_[iE1.second->plan],
					  1,
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    == iE1.second->duration,
					  ("PresdPlanGoal_End{" + iE1.second->name + "<<" + iE2->name + "}").c_str());
				} else if ((goal_events_.find(iE1.second->goal) != goal_events_.end()
				            && plan_events_.find(iE2->plan) != plan_events_.end())) {
					// logger->log_info(name(), "Presd_PlanGoal_Start: %s << %s ", iE1.second->goal.c_str() , iE2 -> plan.c_str() );
					gurobi_models_[model_id]->addGenConstrIndicator(
					  gurobi_vars_plan_[iE2->plan],
					  1,
					  gurobi_vars_time_[iE2->name] - gurobi_vars_time_[iE1.second->name]
					    == iE1.second->duration,
					  ("PresdPlanGoal_Start{" + iE1.second->name + "<<" + iE2->name + "}").c_str());
				}
			}

		//Constraint (2) Plan Selection
		for (auto const &iG : goal_plans_) {
			GRBLinExpr sum = 0;
			for (auto const &iP : iG.second)
				sum += gurobi_vars_plan_[iP];

			gurobi_models_[model_id]->addConstr(sum == 1, ("TotalGoalPlans{" + iG.first + "}").c_str());
			//logger->log_info(name(), "C2: plans of goal %s", iG.first.c_str());
		}

		//Constraints 8,3,4,5,6
		for (auto const &iR : res_setup_duration_) {
			std::map<std::string, std::vector<GRBVar>> inflow_vars;
			std::map<std::string, std::vector<GRBVar>> outflow_vars;

			//Edges Selection GenConstrIndicator
			for (auto const &iEprod : iR.second) {
				for (auto const &iEcons : iEprod.second) {
					std::string resource = iR.first;
					std::string producer = iEprod.first->name;
					std::string consumer = iEcons.first->name;
					std::string cname    = "Select_X{" + resource + "}{" + producer + "->" + consumer + "}";
					int         event_duration = iEprod.first->duration;
					double      setup_duration = iEcons.second;
					gurobi_models_[model_id]->addGenConstrIndicator(
					  gurobi_vars_sequence_[resource][producer][consumer],
					  1,
					  gurobi_vars_time_[consumer] - gurobi_vars_time_[producer] - event_duration
					      - setup_duration
					    >= 0,
					  cname.c_str());
					//logger->log_info(name(), "C8: %s", cname.c_str());

					inflow_vars[consumer].push_back(gurobi_vars_sequence_[iR.first][producer][consumer]);
					outflow_vars[producer].push_back(gurobi_vars_sequence_[iR.first][producer][consumer]);
				}
			}

			std::string event_name;
			int         i;
			int         l;
			//Constrs : Consumers in-flow
			for (auto const &iCons : inflow_vars) {
				event_name = iCons.first;
				l          = inflow_vars[iCons.first].size();
				double     sosWieghts[l];
				GRBVar     sosVars[l];
				GRBLinExpr constr_LHS = 0;
				i                     = 0;
				for (auto const &iFlowVar : iCons.second) {
					constr_LHS += iFlowVar;
					sosVars[i]    = iFlowVar;
					sosWieghts[i] = i;
					i++;
				}
				GRBLinExpr constr_RHS = 1;
				if (gurobi_vars_plan_.find(events_[event_name]->plan) != gurobi_vars_plan_.end())
					constr_RHS = gurobi_vars_plan_[events_[event_name]->plan];
				logger->log_info(name(), "flowIn %s ", event_name.c_str());
				gurobi_models_[model_id]->addConstr(
				  constr_LHS - constr_RHS * abs(events_[event_name]->resources[iR.first]) == 0,
				  ("FlowIn{" + iR.first + "}{" + event_name + "}").c_str());
				//SOS
				if (abs(events_[event_name]->resources[iR.first]) == 1)
					gurobi_models_[model_id]->addSOS(sosVars, sosWieghts, i, GRB_SOS_TYPE1);
			}

			//Constrs: Producers out-flow
			for (auto const &iProd : outflow_vars) {
				event_name = iProd.first;
				l          = outflow_vars[iProd.first].size();
				double     sosWieghts[l];
				GRBVar     sosVars[l];
				GRBLinExpr constr_LHS = 0;
				i                     = 0;
				for (auto const &iFlowVar : iProd.second) {
					constr_LHS += iFlowVar;
					sosVars[i]    = iFlowVar;
					sosWieghts[i] = i;
					i++;
				}
				GRBLinExpr constr_RHS = 1;
				if (gurobi_vars_plan_.find(events_[event_name]->plan) != gurobi_vars_plan_.end())
					constr_RHS = gurobi_vars_plan_[events_[event_name]->plan];
				logger->log_info(name(), "flowOut %s ", event_name.c_str());
				gurobi_models_[model_id]->addConstr(
				  constr_LHS - constr_RHS * abs(events_[event_name]->resources[iR.first]) == 0,
				  ("FlowOut{" + iR.first + "}{" + event_name + "}").c_str());
				//SOS
				if (abs(events_[event_name]->resources[iR.first]) == 1)
					gurobi_models_[model_id]->addSOS(sosVars, sosWieghts, i, GRB_SOS_TYPE1);
			}
		}

		//Constraint 10
		for (auto const &iE : events_)
			gurobi_models_[model_id]->addConstr(Tmax - gurobi_vars_time_[iE.first] >= 0,
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

void
ClipsMipSchedulerThread::check_progress(std::string env_name, std::string model_id)
{
	try {
		const int status = gurobi_models_[model_id]->get(GRB_IntAttr_Status);

		if ((status == GRB_LOADED)) {
			logger->log_warn(name(), "Model is loaded, but no solution information available %u", status);
			return;
		}

		if ((status == GRB_INPROGRESS)) {
			logger->log_warn(name(), "Optimization in progress %u", status);
			return;
		} else {
			logger->log_warn(name(), "Optimization NOT in progress %u", status);
		}

		if (status == GRB_UNBOUNDED) {
			logger->log_warn(name(), "The model cannot be solved  because it is unbounded");
		}

		if (status == GRB_OPTIMAL) {
			gurobi_models_[model_id]->sync();

			logger->log_warn(name(),
			                 "The optimal objective is %f ",
			                 gurobi_models_[model_id]->get(GRB_DoubleAttr_ObjVal));
			gurobi_models_[model_id]->write("gurobi_model.sol");

			//Post result facts
			fawkes::MutexLocker lock(clips_env_.objmutex_ptr());
			CLIPS::Environment &env = **(clips_env_);

			//Post process Time Vars (T)
			for (auto const &iE : events_) {
				std::string type  = "EVENT-TIME";
				float       value = gurobi_vars_time_[iE.first].get(GRB_DoubleAttr_X);
				env.assert_fact_f("(scheduler-info (type %s) (descriptors %s )(value  %f ))",
				                  type.c_str(),
				                  iE.first.c_str(),
				                  value);
			}

			//Post process sequencing Vars (X)
			for (auto const &iR : res_setup_duration_)
				for (auto const &iEprod : iR.second)
					for (auto const &iEcons : iEprod.second) {
						std::string type = "EVENT-SEQUENCE";
						float       value =
						  gurobi_vars_sequence_[iR.first][iEprod.first->name][iEcons.first->name].get(
						    GRB_DoubleAttr_X);
						env.assert_fact_f("(scheduler-info (type %s) (descriptors %s %s %s) (value  %f ))",
						                  type.c_str(),
						                  iR.first.c_str(),
						                  iEprod.first->name.c_str(),
						                  iEcons.first->name.c_str(),
						                  value);
					}

			//Post process plan selection Vars (S)
			for (auto const &iP : plan_events_) {
				std::string type  = "PLAN-SELECTION";
				float       value = gurobi_vars_plan_[iP.first].get(GRB_DoubleAttr_X);
				env.assert_fact_f("(scheduler-info (type %s) (descriptors %s ) (value  %f ))",
				                  type.c_str(),
				                  iP.first.c_str(),
				                  value);
			}

			gurobi_models_[model_id]->reset();
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
};
