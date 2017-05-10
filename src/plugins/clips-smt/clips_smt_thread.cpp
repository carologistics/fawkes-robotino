
/***************************************************************************
 *clips_smt_thread.cpp -Smt feature for CLIPS
 *
 *Created: Created on Fry Dec 16 14:48 2016 by Igor Nicolai Bongartz
 ****************************************************************************/

/*This program is free software; you can redistribute it and/or modify
 *it under the terms of the GNU General Public License as published by
 *the Free Software Foundation; either version 2 of the License, or
 *(at your option) any later version.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 *GNU Library General Public License for more details.
 *
 *Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "clips_smt_thread.h"
// #include <utils/sub_process/proc.h>
#include <core/threading/mutex_locker.h>

#include <navgraph/navgraph.h>
#include <navgraph/yaml_navgraph.h>
// #include <navgraph/constraints/static_list_edge_constraint.h>
#include <navgraph/constraints/static_list_edge_cost_constraint.h>
#include <navgraph/constraints/constraint_repo.h>

#include <llsf_msgs/Plan.pb.h>

using namespace fawkes;
using Rational = mpq_class;

/** @class ClipsNavGraphThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsSmtThread::ClipsSmtThread()
: Thread("ClipsSmtThread", Thread::OPMODE_WAITFORWAKEUP) ,
CLIPSFeature("smt"), CLIPSFeatureAspect(this) // CLIPSFeature("navgraph") before
{
}


/** Destructor. */
ClipsSmtThread::~ClipsSmtThread()
{
}


void
ClipsSmtThread::init()
{
	// Init navgraph
	//edge_cost_constraint_ = new NavGraphStaticListEdgeCostConstraint("static-edge-cost");
	//navgraph->constraint_repo()->register_constraint(edge_cost_constraint_);
	//navgraph->add_change_listener(this);

	// Test z3 extern binary
	// proc_z3_ = NULL;
	clips_smt_test_z3();
	clips_smt_test_formulaGenerator();

	// Test python
	//proc_python_ = NULL;
	//clips_smt_test_python();

	// Set options
	add_constraint_closest_node=true;
	use_encoder_bool=false;
	use_speedup=true;
	use_furthest=false;
}


void
ClipsSmtThread::finalize()
{
	//navgraph->remove_change_listener(this);
	//navgraph->constraint_repo()->unregister_constraint(edge_cost_constraint_->name());
	//delete edge_cost_constraint_;

	//Handle z3 extern binary
	// if (proc_z3_) {
	// logger->log_info(name(), "Killing z3 extern bianry proc");
	// proc_z3_->kill(SIGINT);
	// }
	// delete proc_z3_;

	// Handle python
	// if (proc_python_) {
	// logger->log_info(name(), "Killing python proc");
	// proc_python_->kill(SIGINT);
	// }
	// delete proc_python_;

	envs_.clear();
}


void
ClipsSmtThread::clips_context_init(const std::string &env_name,
					LockPtr<CLIPS::Environment> &clips)
{
	envs_[env_name] = clips;
	logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

	clips.lock();
	//clips->batch_evaluate(SRCDIR"/clips/navgraph.clp");
	//clips_smt_load(clips);

	clips->add_function("smt-request",
	sigc::slot<CLIPS::Value, std::string, void *>(
		sigc::bind<0>(
			sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_request), env_name)
		)
	);

	clips->add_function("smt-get-plan",
	sigc::slot<CLIPS::Value, std::string>(
		sigc::bind<0>(
			sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_get_plan), env_name)
		)
	);

	clips->add_function("smt-done",
	sigc::slot<CLIPS::Value, std::string>(
		sigc::bind<0>(
			sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_done), env_name)
		)
	);

/**
	clips->add_function("navgraph-block-edge",
	sigc::slot<void, std::string, std::string>(
		sigc::bind<0>(
			sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_block_edge), env_name)
		)
	);

	clips->add_function("navgraph-unblock-edge",
	sigc::slot<void, std::string, std::string>(
		sigc::bind<0>(
			sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_unblock_edge), env_name)
		)
	);
**/

clips.unlock();
}

void
ClipsSmtThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

/**
 * Methods for Communication with the agent
 *	- Request performs an activation of the loop function
 *	- Done asks weather the loop is finisihed or not
 **/

CLIPS::Value
ClipsSmtThread::clips_smt_request(std::string env_name, std::string handle, void *msgptr)
{
	// Cast clips msgptr to protobuf_data
	std::shared_ptr<google::protobuf::Message> *m =
	static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
	if (!*m) return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);

	data.CopyFrom(**m); // Use data with subpoint-methods, e.g. data.robots(0).name() OR number_machines
	data_env = env_name;
	data_handle = handle;

	// Use handle to associate request to solution
	logger->log_info(name(),"Handle request: %s", handle.c_str());

	// Wakeup the loop function
		logger->log_info(name(), "At end of request, wake up the loop" );
		wakeup();

	return CLIPS::Value("RUNNING", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsSmtThread::clips_smt_get_plan(std::string env_name, std::string handle)
{
	// Just a simple demonstration, all robots would move to the same place...
	std::shared_ptr<llsf_msgs::ActorGroupPlan> agplan(new llsf_msgs::ActorGroupPlan());
	for (const auto &r : std::vector<std::string>{"R-1", "R-2", "R-3"}) {
		llsf_msgs::ActorSpecificPlan *actor_plan = agplan->add_plans();
		actor_plan->set_actor_name(r);
		llsf_msgs::SequentialPlan *plan = actor_plan->mutable_sequential_plan();
		llsf_msgs::PlanAction *action;
		llsf_msgs::PlanActionParameter *param;

		action = plan->add_actions();
		action->set_name("enter-field");


		if(r.compare("R-1")==0){
			std::map<int, std::string>::iterator it_actions;
			for(it_actions = actions_robot_1.begin(); it_actions!=actions_robot_1.end(); ++it_actions) {
				action = plan->add_actions();
				action->set_name("move");
				param = action->add_params();
				param->set_key("to");
				param->set_value(it_actions->second);
			}
		}
		else if(r.compare("R-2")==0){
			std::map<int, std::string>::iterator it_actions;
			for(it_actions = actions_robot_2.begin(); it_actions!=actions_robot_2.end(); ++it_actions) {
				action = plan->add_actions();
				action->set_name("move");
				param = action->add_params();
				param->set_key("to");
				param->set_value(it_actions->second);
			}
		}
		else if(r.compare("R-3")==0){
			std::map<int, std::string>::iterator it_actions;
			for(it_actions = actions_robot_3.begin(); it_actions!=actions_robot_3.end(); ++it_actions) {
				action = plan->add_actions();
				action->set_name("move");
				param = action->add_params();
				param->set_key("to");
				param->set_value(it_actions->second);
			}
		}

	}


	// Use handle to associate plan to initial request
	logger->log_info(name(),"Return plan of request: %s", handle.c_str());
	logger->log_info(name(),"Plan:\n%s", agplan->DebugString().c_str());

	// Fill the empty protobuf message with the computed task

	std::shared_ptr<google::protobuf::Message> *m =
		new std::shared_ptr<google::protobuf::Message>(agplan);

	return CLIPS::Value(m, CLIPS::TYPE_EXTERNAL_ADDRESS);
}

CLIPS::Value
ClipsSmtThread::clips_smt_done(std::string foo, std::string bar)
{
	return CLIPS::Value(running() ? "FALSE" : "TRUE", CLIPS::TYPE_SYMBOL);
}

/**
 * Loop function activated by clips_smt_request()
 *	- Init number of robots and machines
 *	- Prepare navgraph information as distances between robots and machines
 *	- Create formulas
 *	- Give determined formula to solver
 *	- React on solver solution
 **/

void
ClipsSmtThread::loop()
{
	logger->log_info(name(), "Thread performs loop and is running [%d]", running());

	number_robots = data.robots().size()-1;
	number_machines = data.machines().size();
	number_bits = ceil(log2(number_machines));

	// Compute distances between nodes using navgraph
	clips_smt_fill_node_names();
	clips_smt_compute_distances_robots();
	clips_smt_compute_distances_machines();

	// Declare variable for encoding
	std::map<std::string, z3::expr> variables_pos;
	std::map<std::string, z3::expr> variables_p;
	std::map<std::string, z3::expr> variables_d;
	std::map<std::string, z3::expr> variables_m;

	// Declare formulas for encoding
	z3::expr_vector formula_bool = clips_smt_encoder_bool(variables_pos, variables_p, variables_d, variables_m);
	z3::expr_vector formula = clips_smt_encoder(variables_pos, variables_d, variables_m);

	// Give it to z3 solver
	if(use_encoder_bool) clips_smt_solve_formula(variables_pos, variables_d, variables_m, formula_bool);
	else clips_smt_solve_formula(variables_pos, variables_d, variables_m, formula);
	logger->log_info(name(), "Thread reached end of loop");

	envs_[data_env].lock();
	envs_[data_env]->assert_fact_f("(smt-plan-complete \"%s\")", data_handle.c_str());
	envs_[data_env].unlock();
}

/**
 * Navgraph Methods
 *	- Extract names of nodes
 *	- Compute the distances between the robots and machines
 *	- Compute the distances between the machines themself and the
 **/

void
ClipsSmtThread::graph_changed() throw()
{
	 //wakeup();
}

void
ClipsSmtThread::clips_smt_fill_node_names()
{
	logger->log_info(name(), "Get name of machines using protobuf data");
	node_names_.clear();

	// C-ins-in is never in the list of machines
	node_names_[0] = "C-ins-in";

	// Read names of machines automatically
	// for(int i=0; i<number_machines; ++i){
	// std::string machine_name = data.machines(i).name().c_str();
	// machine_name += "-I";
	// // logger->log_info(name(), "Add %s to node_names_", machine_name.c_str());
	// node_names_[i+1] = machine_name;
	// }

	node_names_[1] = "C-BS-I";
	node_names_[2] = "C-RS1-I";
	node_names_[3] = "C-RS2-I";
	node_names_[4] = "C-CS1-I";
	node_names_[5] = "C-CS2-I";
	node_names_[6] = "C-DS-I";
	// node_names_[7] = "C-BS-O";
	// node_names_[8] = "C-RS1-O";
	// node_names_[9] = "C-RS2-O";
	// node_names_[10] = "C-CS1-O";
	// node_names_[11] = "C-CS2-O";
	// node_names_[12] = "C-DS-O";
}

void
ClipsSmtThread::clips_smt_compute_distances_robots()
{
	logger->log_info(name(), "Compute distances between robots and machines using navgraph");

	MutexLocker lock(navgraph.objmutex_ptr());

	// Compute distances between robotos positions and machines
	for(int r = 0; r < number_robots+1; ++r){
		//logger->log_info(name(), "Position of robot %s is (%f,%f)", data.robots(r).name().c_str(), data.robots(r).pose().x(), data.robots(r).pose().y());
		if(data.robots(r).name().compare("R-1")==0 || data.robots(r).name().compare("R-2")==0 || data.robots(r).name().compare("R-3")==0) { // Avoid robot 'RefBox'
			NavGraphNode robot_node(data.robots(r).name().c_str(), data.robots(r).pose().x(), data.robots(r).pose().y());
			NavGraphNode from = navgraph->closest_node(robot_node.x(), robot_node.y());

			for (unsigned int i = 1; i < node_names_.size(); ++i) {
				std::pair<std::string, std::string> nodes_pair(robot_node.name(), node_names_[i]);

				NavGraphNode to = navgraph->node(node_names_[i]);
				NavGraphPath p = navgraph->search_path(from, to);

				//logger->log_info(name(), "Distance between node %s and node %s is %f", robot_node.name().c_str(), node_names_[i].c_str(), p.cost()+navgraph->cost(from, robot_node));
				distances_[nodes_pair] = p.cost() + navgraph->cost(from, robot_node); // Use 'R-1' 'R-2' and 'R-3' to identify robots in distances_
			}
		}
	}
}

void
ClipsSmtThread::clips_smt_compute_distances_machines()
{
	logger->log_info(name(), "Compute distances between all machines using navgraph");

	MutexLocker lock(navgraph.objmutex_ptr());

	// Compute distances between unconnected C-ins-in and all other machines
	NavGraphNode ins_node(navgraph->node("C-ins-in"));
	NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		std::pair<std::string, std::string> nodes_pair(ins_node.name(), node_names_[i]);

		NavGraphNode to = navgraph->node(node_names_[i]);
		NavGraphPath p = navgraph->search_path(from, to);

		// logger->log_info(name(), "Distance between node %s and node %s is %f", from.name().c_str(), node_names_[i].c_str(), p.cost());
		distances_[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
	}

	// Compute distances between machines
	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		for (unsigned int j = 1; j < node_names_.size(); ++j) {
			if (i == j) continue;
			std::pair<std::string, std::string> nodes_pair(node_names_[i], node_names_[j]);

			NavGraphPath p = navgraph->search_path(node_names_[i], node_names_[j]);
			// logger->log_info(name(), "Distance between node %s and node %s is %f", node_names_[i].c_str(), node_names_[j].c_str(), p.cost());
			distances_[nodes_pair] = p.cost();
		}
	}
}

/*
 * Methods for encoding the given protobuf data in formulas
 *	- Encode formula for exploration phase by Francesco Leofante
 *	- Encode formula for game by Leonard Kopp
 */

// First exploration encoder

z3::expr_vector
ClipsSmtThread::clips_smt_encoder(std::map<std::string, z3::expr>& variables_pos, std::map<std::string, z3::expr>& variables_d, std::map<std::string, z3::expr>& variables_m)
{
	logger->log_info(name(), "Create z3 encoder");

	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);

	std::map<std::string, z3::expr>::iterator it_map;
	std::map<int, std::string>::iterator it_node_names;

	logger->log_info(name(), "Add variables");

	// Variables pos_i_j
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = -3; j < number_machines+1; ++j) {
			z3::expr var(_z3_context);
			std::string varName = "pos_" + std::to_string(i) + "_" + std::to_string(j);
			var=_z3_context.int_const((varName).c_str());
			variables_pos.insert(std::make_pair(varName, var));
		}
	}

	// Variables d_i_j
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 0; j < number_machines+1; ++j) {
			z3::expr var(_z3_context);
			std::string varName = "d_" + std::to_string(i) + "_" + std::to_string(j);
			var=_z3_context.real_const((varName).c_str());
			variables_d.insert(std::make_pair(varName, var));
		}
	}

	// Variables m_i
	for(int i = 1; i < number_robots+1; ++i){
		z3::expr var(_z3_context);
		std::string varName = "m_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		variables_m.insert(std::make_pair(varName, var));
	}

	logger->log_info(name(), "Add constraints");

	// Init variable true and false
	z3::expr var_false(_z3_context);
	var_false = _z3_context.bool_val(false);
	z3::expr var_true(_z3_context);
	var_true = _z3_context.bool_val(true);

	// Get distances
	float min_distance_first=100, min_distance=100, max_distance_first=0, max_distance=0;
	int min_m_first=0, max_m_first=0;

	// Get min_distance and max_distance
	for(int n = 0; n < number_machines+1; ++n) {
		for(int m = 1; m < number_machines+1; ++m) {
			if(n!=m) {
				float distance = distances_[std::make_pair(node_names_[n], node_names_[m])];

				if(distance<min_distance) {
					min_distance = distance;
				}

				if(distance>max_distance) {
					max_distance = distance;
				}
			}
		}
	}

	// Get min_distance_first and min_m_first
	for(int m = 1; m < number_machines+1; ++m) {
		float distance = distances_[std::make_pair("C-ins-in", node_names_[m])];

		if(distance<min_distance_first) {
			min_distance_first = distance;
			min_m_first = m;
		}
		if(distance>max_distance_first) {
			max_distance_first = distance;
			max_m_first = m;
		}
	}

	// Constraint: d_i_0 = 0
	for(int i = 1; i < number_robots+1; ++i){

		it_map = variables_d.find("d_"+std::to_string(i)+"_0");
		if(it_map != variables_d.end()) {

			z3::expr var_d_i_0 = it_map->second;
			z3::expr constraint( var_d_i_0 == 0);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_d_%i_0 not found", i);
		}
	}

	// Constraint: d_i_j <= d_i_M
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 0; j < number_machines+1; ++j) {

			z3::expr var_d_i_j(_z3_context);
			z3::expr var_d_i_M(_z3_context);

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_d.end()) {
				var_d_i_j = it_map->second;
			}
			else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(number_machines));
			if(it_map != variables_d.end()) {
				var_d_i_M = it_map->second;
			}
			else {
				logger->log_error(name(), "var_d_%i_M not found", i);
			}

			z3::expr constraint( var_d_i_j <= var_d_i_M);
			constraints.push_back(constraint);
		}
	}


	if(use_speedup){
		// Constraint: d_i_j >= min_distance
		for(int i = 1; i < number_robots+1; ++i){
			for(int j = 1; j < number_machines; ++j) { // Test j from 1 instead of 0

				z3::expr var_d_i_j(_z3_context);

				it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
				if(it_map != variables_d.end()) {
					var_d_i_j = it_map->second;
				}
				else {
					logger->log_error(name(), "var_d_%i_%i not found", i, j);
				}

				z3::expr min_distance_z3 = _z3_context.real_val((std::to_string(min_distance)).c_str());
				z3::expr constraint( var_d_i_j >= min_distance_z3);
				constraints.push_back(constraint);
			}
		}

		// Constraint: d_i_j-d_i_j-1 <= max_distance
		for(int i = 1; i < number_robots+1; ++i){
			for(int j = 1; j < number_machines; ++j) {

				z3::expr var_d_i_j(_z3_context);
				z3::expr var_d_i_j_1(_z3_context);

				it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
				if(it_map != variables_d.end()) {
					var_d_i_j = it_map->second;
				}
				else {
					logger->log_error(name(), "var_d_%i_%i not found", i, j);
				}

				it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j-1));
				if(it_map != variables_d.end()) {
					var_d_i_j_1 = it_map->second;
				}
				else {
					logger->log_error(name(), "var_d_%i_%i not found", i, j-1);
				}

				z3::expr max_distance_z3 = _z3_context.real_val((std::to_string(max_distance)).c_str());
				z3::expr constraint( var_d_i_j - var_d_i_j_1 <= max_distance_z3);
				constraints.push_back(constraint);
			}
		}
	}

	// Constraint: pos_1_n == -1 for all n=-3,..., -1
	for(it_map = variables_pos.begin(); it_map != variables_pos.end(); ++it_map){
		if(it_map->first.compare("pos_1_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -1);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_1_-2")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -1);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_1_-3")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -1);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_2_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -1);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_2_-2")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -2);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_2_-3")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -2);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_3_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -1);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_3_-2")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -2);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_3_-3")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == -3);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_1_0")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == 0);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_2_0")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == 0);
			constraints.push_back(constraint);
		}
		else if(it_map->first.compare("pos_3_0")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable == 0);
			constraints.push_back(constraint);
		}
	}

	// First move constraint:
	for(int i = 1; i < number_robots+1; ++i){
		z3::expr var_pose_i_1(_z3_context);
		z3::expr var_d_i_1(_z3_context);

		it_map = variables_pos.find("pos_"+std::to_string(i)+"_1");
		if(it_map != variables_pos.end()) {
			var_pose_i_1 = it_map->second;
		}
		else {
			logger->log_error(name(), "var_pos_%i_1 not found", i);
		}

		it_map = variables_d.find("d_"+std::to_string(i)+"_1");
		if(it_map != variables_d.end()) {
			var_d_i_1 = it_map->second;
		}
		else {
			logger->log_error(name(), "var_d_%i_1 not found", i);
		}

		z3::expr constraint(var_false);

		for(it_node_names = node_names_.begin(); it_node_names != node_names_.end(); ++it_node_names){
			if(it_node_names->second.compare("C-ins-in")!=0){
				float distance = distances_[std::make_pair("C-ins-in", it_node_names->second)];
				// logger->log_info(name(), "Distance is %f to %s", distance, it_node_names->second.c_str());
				z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
				constraint = constraint || (var_pose_i_1 == it_node_names->first && var_d_i_1 == distance_z3);
			}
		}

		constraints.push_back(constraint);
	}

	// Other moves constraint:
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 2; j < number_machines+1; ++j) {
			z3::expr var_pose_i_j(_z3_context);
			z3::expr var_pos_i_j_1(_z3_context);
			z3::expr var_d_i_j(_z3_context);
			z3::expr var_d_i_j_1(_z3_context);
			z3::expr var_d_i_M(_z3_context);

			it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_pos.end()) {
				var_pose_i_j = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i_%i not found", i, j);
			}

			it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j-1));
			if(it_map != variables_pos.end()) {
				var_pos_i_j_1 = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i_%i not found", i, j-1);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_d.end()) {
				var_d_i_j = it_map->second;
			}
			else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j-1));
			if(it_map != variables_d.end()) {
				var_d_i_j_1 = it_map->second;
			}
			else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j-1);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(number_machines));
			if(it_map != variables_d.end()) {
				var_d_i_M = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i_M not found", i);
			}

			z3::expr constraint1(var_pose_i_j==-4 && var_d_i_M==var_d_i_j_1);
			z3::expr var(_z3_context);
			var = _z3_context.bool_val(false);
			z3::expr constraint2(var);

			for(int k = 1; k < number_machines+1; ++k) {
				for(int l = 1; l < number_machines+1; ++l) {
					if(k!=l) {
						float distance = distances_[std::make_pair(node_names_[k], node_names_[l])];
						z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
						constraint2 = constraint2 || (var_pos_i_j_1==k && var_pose_i_j==l && var_d_i_j==(var_d_i_j_1+distance_z3));
					}
				}
			}

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Robot can not visit the same machine twice <- Problem
	// Added initialization of constraint to false.
	// it might be the case that concat on uninitialized expr gives problems.

	for(int i = 1; i < number_machines+1; ++i){

		z3::expr constraint1(var_false);

		for(int j = 1; j < number_robots+1; ++j) {
			for(int k = 1; k < number_machines+1; ++k) {

				z3::expr var_pos_i_k(_z3_context);

				it_map = variables_pos.find("pos_"+std::to_string(j)+"_"+std::to_string(k));
				if(it_map != variables_pos.end()) {
					var_pos_i_k = it_map->second;
				}
				else {
					logger->log_error(name(), "var_pos_%i_%i not found", j, k);

				}

				z3::expr constraint2(var_true);

				for(int u = 1; u < number_robots+1;++u){
					for(int v = 1; v < number_machines+1; ++v){

						z3::expr var_pos_u_v(_z3_context);

						it_map = variables_pos.find("pos_"+std::to_string(u)+"_"+std::to_string(v));
						if(it_map != variables_pos.end()) {
							var_pos_u_v = it_map->second;
						} else {
							logger->log_error(name(), "var_pos_%i_%i not found", u, v);
						}

						z3::expr var_j_u_k_v(_z3_context);
						if(j==u && k==v) {
							var_j_u_k_v = var_true;
						} else {
							var_j_u_k_v = var_false;
						}

						constraint2 = constraint2 && (!(var_pos_u_v == i) || var_j_u_k_v);
					}
				}
				constraint2 = constraint2 && (var_pos_i_k == i);
				constraint1 = constraint1 || constraint2;
			}
		}
		constraints.push_back(constraint1);
	}

	// Encoding maximum distance <- Problem

	z3::expr constraint_max_distance(var_false);
	for(int i = 1; i < number_robots+1; ++i){

		z3::expr var_m_i(_z3_context);
		z3::expr var_d_i_M(_z3_context);

		it_map = variables_m.find("m_"+std::to_string(i));
		if(it_map != variables_m.end()) {
			var_m_i = it_map->second;
		} else {
			logger->log_error(name(), "var_m_%i not found", i);
		}

		it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(number_machines));
		if(it_map != variables_d.end()) {
			var_d_i_M = it_map->second;
		} else {
			logger->log_error(name(), "var_d_%i_M not found", i);
		}

		// z3::expr constraint1(var_m_i==0);
		// z3::expr constraint2(var_m_i==1);
		z3::expr constraint(var_m_i);

		for(int j = 1; j < number_robots+1; ++j) {
			if(j!=i) {
				z3::expr var_d_j_M(_z3_context);

				it_map = variables_d.find("d_"+std::to_string(j)+"_"+std::to_string(number_machines));
				if(it_map != variables_d.end()) {
					var_d_j_M = it_map->second;
				} else {
					logger->log_error(name(), "var_d_%i_M not found", j);
				}

				constraint = constraint && (var_d_j_M < var_d_i_M);
			}
		}
		constraint_max_distance = constraint_max_distance || constraint;
	}
	constraints.push_back(constraint_max_distance);

	// Additional constraint first robot goes to the closest machine pos_1_1=k and d_1_1=distances_("C-ins-in",k)
	if(add_constraint_closest_node) {
		z3::expr constraint(var_false);

		for(int j=1; j< number_machines+1; ++j) {
			z3::expr var_p_1_j(_z3_context);

			it_map = variables_pos.find("pos_"+std::to_string(1)+"_"+std::to_string(j));
			if(it_map != variables_pos.end()) {
				var_p_1_j= it_map->second;
			} else {
				logger->log_error(name(), "var_p_1_%i not found", j);
			}

			if(use_furthest) constraint = constraint || (var_p_1_j==max_m_first);
			else constraint = constraint || (var_p_1_j==min_m_first);
		}
		constraints.push_back(constraint);
	}

	return constraints;
}

// Second exploration encoder with boolean binary encoding of positions

z3::expr_vector
ClipsSmtThread::clips_smt_encoder_bool(std::map<std::string, z3::expr>& variables_pos, std::map<std::string, z3::expr>& variables_p, std::map<std::string, z3::expr>& variables_d, std::map<std::string, z3::expr>& variables_m)
{
	logger->log_info(name(), "Create z3 encoder with bools");

	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);

	std::map<std::string, z3::expr>::iterator it_map;
	std::map<int, std::string>::iterator it_node_names;

	logger->log_info(name(), "Add variables");

	// Variables pos_i_j_k
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = -3; j < number_machines+1; ++j) {
			for(int k = -4;k < number_machines+1; ++k) {
				z3::expr var(_z3_context);
				std::string varName = "pos_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
				var=_z3_context.bool_const((varName).c_str());
				variables_pos.insert(std::make_pair(varName, var));
			}
		}
	}

	// Variables p_i_j_k for boolean encoding of k, i.e., k = 1 -> 0001
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = -3; j < number_machines+1; ++j) {
			for(int k = 0;k < number_bits+1; ++k) { // TODO Check if +1 is correct
				z3::expr var(_z3_context);
				std::string varName = "p_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
				var=_z3_context.bool_const((varName).c_str());
				variables_p.insert(std::make_pair(varName, var));
			}
		}
	}

	// Variables d_i_j
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 0; j < number_machines+1; ++j) {
			z3::expr var(_z3_context);
			std::string varName = "d_" + std::to_string(i) + "_" + std::to_string(j);
			var=_z3_context.real_const((varName).c_str());
			variables_d.insert(std::make_pair(varName, var));
		}
	}

	// Variables m_i TODO check
	for(int i = 1; i < number_robots+1; ++i){
		z3::expr var(_z3_context);
		std::string varName = "m_" + std::to_string(i);
		var=_z3_context.bool_const((varName).c_str());
		variables_m.insert(std::make_pair(varName, var));
	}

	logger->log_info(name(), "Get min_distance and max_distance");

	float min_distance_first=100, min_distance=100, max_distance=0;
	int min_m_first=0;

	// Get min_distance and max_distance TODO only for not from C-ins-in
	for(int n = 0; n < number_machines+1; ++n) {
		for(int m = 1; m < number_machines+1; ++m) {
			if(n!=m) {
				float distance = distances_[std::make_pair(node_names_[n], node_names_[m])];

				if(distance<min_distance) {
					min_distance = distance;
				}

				if(distance>max_distance) {
					max_distance = distance;
				}
			}
		}
	}

	// Get min_distance_first and min_m_first
	for(int m = 1; m < number_machines+1; ++m) {
		float distance = distances_[std::make_pair("C-ins-in", node_names_[m])];

		if(distance<min_distance_first) {
			min_distance_first = distance;
			min_m_first = m;
		}
	}

	// Init variable true and false
	z3::expr var_false(_z3_context);
	var_false = _z3_context.bool_val(false);
	z3::expr var_true(_z3_context);
	var_true = _z3_context.bool_val(true);

	logger->log_info(name(), "Add constraints");

	// Constraint: d_i_0 = 0
	for(int i = 1; i < number_robots+1; ++i){

		it_map = variables_d.find("d_"+std::to_string(i)+"_0");
		if(it_map != variables_d.end()) {

			z3::expr var_d_i_0 = it_map->second;
			z3::expr constraint( var_d_i_0 == 0);
			constraints.push_back(constraint);
		} else {
			logger->log_error(name(), "var_d_%i_0 not found", i);
		}
	}

	// Constraint: d_i_j <= d_i_M
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 1; j < number_machines; ++j) {

			z3::expr var_d_i_j(_z3_context);
			z3::expr var_d_i_M(_z3_context);

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_d.end()) {
				var_d_i_j = it_map->second;
			} else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(number_machines));
			if(it_map != variables_d.end()) {
				var_d_i_M = it_map->second;
			} else {
				logger->log_error(name(), "var_d_%i_M not found", i);
			}

			z3::expr constraint( var_d_i_j <= var_d_i_M);
			constraints.push_back(constraint);
		}
	}

	// Constraint: d_i_j >= min_distance
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 1; j < number_machines; ++j) { // Test j from 1 instead of 0

			z3::expr var_d_i_j(_z3_context);

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_d.end()) {
				var_d_i_j = it_map->second;
			} else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j);
			}

			z3::expr min_distance_z3 = _z3_context.real_val((std::to_string(min_distance)).c_str());
			z3::expr constraint( var_d_i_j >= min_distance_z3);
			constraints.push_back(constraint);
		}
	}

	// Constraint: d_i_j-d_i_j-1 <= max_distance
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 1; j < number_machines; ++j) {

			z3::expr var_d_i_j(_z3_context);
			z3::expr var_d_i_j_1(_z3_context);

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_d.end()) {
				var_d_i_j = it_map->second;
			} else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j-1));
			if(it_map != variables_d.end()) {
				var_d_i_j_1 = it_map->second;
			} else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j-1);
			}

			z3::expr max_distance_z3 = _z3_context.real_val((std::to_string(max_distance)).c_str());
			z3::expr constraint( var_d_i_j - var_d_i_j_1 <= max_distance_z3);
			constraints.push_back(constraint);
		}
	}

	// Constraint: pos_1_n == -1 for all n=-3,..., -1
	for(it_map = variables_pos.begin(); it_map != variables_pos.end(); ++it_map){
		if(it_map->first.compare("pos_1_-1_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_1_-2_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		}  else if(it_map->first.compare("pos_1_-3_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_2_-1_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_2_-2_-2")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_2_-3_-2")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_3_-1_-1")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_3_-2_-2")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_3_-3_-3")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_1_0_0")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_2_0_0")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		} else if(it_map->first.compare("pos_3_0_0")==0){
			z3::expr variable = it_map->second;
			z3::expr constraint( variable );
			constraints.push_back(constraint);
		}
	}

	// Additional constraint first robot goes to the closest machine pos_1_1=k and d_1_1=distances_("C-ins-in",k)
	z3::expr var_pos_1_1_min(_z3_context);
	z3::expr var_d_1_1(_z3_context);

	it_map = variables_pos.find("pos_"+std::to_string(1)+"_"+std::to_string(1)+"_"+std::to_string(min_m_first));
	if(it_map != variables_pos.end()) {
		var_pos_1_1_min = it_map->second;
	}
	else {
		logger->log_error(name(), "var_pos_1_1_%i not found", min_m_first);
	}

	it_map = variables_d.find("d_"+std::to_string(1)+"_"+std::to_string(1));
	if(it_map != variables_d.end()) {
		var_d_1_1 = it_map->second;
	}
	else {
		logger->log_error(name(), "var_d_1_1 not found");
	}

	z3::expr min_distance_z3 = _z3_context.real_val((std::to_string(min_distance_first)).c_str());
	z3::expr constraint_closest_node(var_pos_1_1_min && var_d_1_1==min_distance_z3);
	constraints.push_back(constraint_closest_node);

	// Constraint for successive steps: if a robot moves, d is incremented, otherwise it is not
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = 1; j < number_machines+1; ++j) {

			z3::expr var_pos_src(_z3_context);
			z3::expr var_pos_dst(_z3_context);
			z3::expr var_pos_v1(_z3_context);
			z3::expr var_d_src(_z3_context);
			z3::expr var_d_dst(_z3_context);
			z3::expr var_d_i_M(_z3_context);

			it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j)+"_-4");
			if(it_map != variables_pos.end()) {
				var_pos_v1 = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i_%i_-4 not found", i, j);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
			if(it_map != variables_d.end()) {
				var_d_dst = it_map->second;
			}
			else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j-1));
			if(it_map != variables_d.end()) {
				var_d_src = it_map->second;
			}
			else {
				logger->log_error(name(), "var_d_%i_%i not found", i, j-1);
			}

			it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(number_machines));
			if(it_map != variables_d.end()) {
				var_d_i_M = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i_M not found", i);
			}

			z3::expr constraint1(var_pos_v1 && var_d_i_M==var_d_src);
			z3::expr var(_z3_context);
			z3::expr constraint2(var_false);

			for(int k = 0; k < number_machines+1; ++k) {

				it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j-1)+"_"+std::to_string(k));
				if(it_map != variables_pos.end()) {
					var_pos_src = it_map->second;
				}
				else {
					logger->log_error(name(), "var_pos_%i_%i_%i not found", i, j-1, k);
				}

				for(int l = 1; l < number_machines+1; ++l) {
					it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(l));
					if(it_map != variables_pos.end()) {
						var_pos_dst = it_map->second;
					}
					else {
						logger->log_error(name(), "var_pos_%i_%i_%i not found", i, j, l);
					}

					if(k!=l) {
						float distance = distances_[std::make_pair(node_names_[k], node_names_[l])];
						z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
						constraint2 = constraint2 || (var_pos_src && var_pos_dst && var_d_dst==(var_d_src+distance_z3));
					}
				}
			}

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Robot can not visit the same machine twice <- Problem
	// Added initialization of constraint to false.
	// it might be the case that concat on uninitialized expr gives problems.

	for(int k = 1; k < number_machines+1; ++k){

		z3::expr constraint1(var_false); // TODO check

		for(int i = 1; i < number_robots+1; ++i) {
			for(int j = 1; j < number_machines+1; ++j) {

				z3::expr var_pos_i_j_k(_z3_context);

				it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(k));
				if(it_map != variables_pos.end()) {
					var_pos_i_j_k = it_map->second;
				}
				else {
					logger->log_error(name(), "var_pos_%i_%i_%i not found", i, j, k);
				}

				z3::expr constraint2(var_pos_i_j_k);

				for(int u = 1; u < number_robots+1;++u){
					for(int v = 1; v < number_machines+1; ++v){

						z3::expr var_pos_u_v_k(_z3_context);

						it_map = variables_pos.find("pos_"+std::to_string(u)+"_"+std::to_string(v)+"_"+std::to_string(k));
						if(it_map != variables_pos.end()) {
							var_pos_u_v_k = it_map->second;
						}
						else {
							logger->log_error(name(), "var_pos_%i_%i_%i not found", u, v, k);
						}

						if(i==u && j==v) {
							continue;
						} else {
							constraint2 = constraint2 && !(var_pos_u_v_k);
						}
					}
				}
				constraint1 = constraint1 || constraint2;
			}
		}
		constraints.push_back(constraint1);
	}

	z3::expr constraint_max_distance(var_true); // Before var_m_i==0
	// Encoding maximum distance <- Problem
	for(int i = 1; i < number_robots+1; ++i){

		z3::expr var_m_i(_z3_context);

		it_map = variables_m.find("m_"+std::to_string(i));
		if(it_map != variables_m.end()) {
			var_m_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_m_%i not found", i);
		}

		z3::expr constraint2(var_true); // Before var_m_i==1

		for(int j = 1; j < number_robots+1; ++j) {

			if(!(j==i)) {
				z3::expr var_d_j_M(_z3_context);
				z3::expr var_d_i_M(_z3_context);

				it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(number_machines));
				if(it_map != variables_d.end()) {
					var_d_i_M = it_map->second;
				}
				else {
					logger->log_error(name(), "var_d_%i_M not found", i);
				}

				it_map = variables_d.find("d_"+std::to_string(j)+"_"+std::to_string(number_machines));
				if(it_map != variables_d.end()) {
					var_d_j_M = it_map->second;
				}
				else {
					logger->log_error(name(), "var_d_%i_M not found", j);
				}

				constraint2 = constraint2 && (var_d_j_M <= var_d_i_M);
			}
		}

		constraint_max_distance = constraint_max_distance || (var_m_i && constraint2); // TODO check var_m_i==1 and not 0
	}

	constraints.push_back(constraint_max_distance);

	// Constraints to encode the relation pos_i_j_k <-> (... binary encoding ...)
	for(int i = 1; i < number_robots+1; ++i){
		for(int j = -3; j < number_machines+1; ++j) {
			for(int k = -4; k < number_machines+1; ++k) {
				z3::expr var_pos(_z3_context);
				z3::expr var_pos_bool(_z3_context);

				it_map = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(k));
				if(it_map != variables_pos.end()) {
					var_pos = it_map->second;
				}
				else {
					logger->log_error(name(), "var_pos_%i_%i_%i not found", i, j,k);
				}

				std::string binary;

				switch(k) {
					case -4:
						binary = std::bitset<4>(number_machines+4).to_string(); // Use 8 as bigger than 4
						break;
					case -3:
						binary = std::bitset<4>(number_machines+3).to_string();
						break;
					case -2:
						binary = std::bitset<4>(number_machines+2).to_string();
						break;
					case -1:
						binary = std::bitset<4>(number_machines+1).to_string();
						break;
					default:
						binary = std::bitset<4>(k).to_string();
						break;
				}

				// logger->log_info(name(), "Convert %i into [%s]", k, binary.c_str());
				z3::expr constraint2(var_true);
				int ind=0;

				for(int l = binary.size()-1; l > -1; --l) { // start at number_bits as 8 is to big and everything between is 0
					it_map = variables_p.find("p_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(ind));
					if(it_map != variables_p.end()) {
						var_pos_bool = it_map->second;
					}
					else {
						logger->log_error(name(), "var_p_%i_%i_%i not found", i, j, ind);
					}

					if(binary[l] == '1') {
						constraint2 = constraint2 && var_pos_bool;
					}
					else if(binary[l] == '0') {
						constraint2 = constraint2 && !var_pos_bool;
					}

					ind++;

				}

				constraints.push_back( (!var_pos || constraint2) && (!constraint2 || var_pos));
			}
		}
	}
	return constraints;
}

GameData
ClipsSmtThread::clips_smt_convert_protobuf_to_gamedata()
{
	GameData _generatorData = GameData();

	//machines
	for (int i = 0; i < number_machines; i++)
	{
		//name -> id
		Machine _tmpMachine = Machine("dummy", atoi(data.machines(i).name().c_str()));
		//type -> type
		_tmpMachine.setType(data.machines(i).type());
		//TODO (Lukas) WorkingPiece
		//TODO (Lukas) Distances
		//_generatorData.addMachine(_tmpMachine);
		}


		//Robots
		for (int i = 0; i < number_robots+1; i++)
		{
		//name -> id
		Robot _tmpRobot = Robot(atoi(data.robots(i).name().c_str()));
		//TODO (Lukas) Distances
	}

//Orders
	for (int i = 0; i < data.orders().size(); i++)
	{
		Workpiece _tmpWorkPiece = Workpiece();
		//Color conversions
		int _baseColor = data.orders(i).base_color()+1;
		int _capColor = data.orders(i).cap_color();

		std::vector<int> _ringColors;
		for (int j = 0; j < data.orders(i).ring_colors().size(); j++){
			_ringColors.push_back(data.orders(i).ring_colors(i)+1);
		}

		std::cout << "BC" << _baseColor << "CC" << _capColor << std::endl;
		//TODO fix this conversion:
		/*
		_tmpWorkPiece.setBaseColor(_baseColor);
		_tmpWorkPiece.setCapColor(_capColor);
		_tmpWorkPiece.setRingColor(_ringColors);
		*/

		//GameData::Order _tmpOrder = GameData::Order(data.orders(i).id());
		//TODO (Lukas) Distances
	}

	return _generatorData;
}

// Solve encoding of exploration phase
void
 ClipsSmtThread::clips_smt_solve_formula(std::map<std::string, z3::expr>& variables_pos, std::map<std::string, z3::expr>& variables_d, std::map<std::string, z3::expr>& variables_m,z3::expr_vector formula)
{
	logger->log_info(name(), "Solve z3 formula");

	z3::optimize z3Optimizer(_z3_context);

	std::map<std::string, z3::expr>::iterator it_map;

	for (unsigned i = 0; i < formula.size(); i++) {
		z3Optimizer.add(formula[i]);
		//logger->log_info(name(), "Solving constraint %s", formula[i]);
		//std::cout << "Solving constrinat " << formula[i] << std::endl;
	}

	// Add objective functions
	z3::expr d_1_M(_z3_context);
	z3::expr d_2_M(_z3_context);
	z3::expr d_3_M(_z3_context);
	z3::expr m_1(_z3_context);
	z3::expr m_2(_z3_context);
	z3::expr m_3(_z3_context);

	it_map = variables_d.find("d_"+std::to_string(1)+"_"+std::to_string(number_machines));
	if(it_map != variables_d.end()) {
		d_1_M = it_map->second;
	}
	else {
		logger->log_error(name(), "var_d_1_M not found");
	}

	it_map = variables_d.find("d_"+std::to_string(2)+"_"+std::to_string(number_machines));
	if(it_map != variables_d.end()) {
		d_2_M = it_map->second;
	}
	else {
		logger->log_error(name(), "var_d_2_M not found");
	}

	it_map = variables_d.find("d_"+std::to_string(3)+"_"+std::to_string(number_machines));
	if(it_map != variables_d.end()) {
		d_3_M = it_map->second;
	}
	else {
		logger->log_error(name(), "var_d_3_M not found");
	}

	it_map = variables_m.find("m_"+std::to_string(1));
	if(it_map != variables_m.end()) {
		m_1 = it_map->second;
	}
	else {
		logger->log_error(name(), "var_m_1 not found");
	}

	it_map = variables_m.find("m_"+std::to_string(2));
	if(it_map != variables_m.end()) {
		m_2 = it_map->second;
	}
	else {
		logger->log_error(name(), "var_m_2 not found");
	}

	it_map = variables_m.find("m_"+std::to_string(3));
	if(it_map != variables_m.end()) {
		m_3 = it_map->second;
	}
	else {
		logger->log_error(name(), "var_m_3 not found");
	}

	//z3Optimizer.minimize(m_1*d_1_M + m_2*d_2_M + m_3*d_3_M);
	z3Optimizer.minimize(d_1_M + d_2_M + d_3_M);

	// Export stats into clips_smt_thread_stats.txt
	std::ofstream outputFile;
	outputFile.open("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats.txt", std::ofstream::out | std::ofstream::app);

	// Add time stamp to stats
	time_t now = time(0);
	char* dt = ctime(&now);
	outputFile << std::endl << dt << "Scenario with " << node_names_.size()-1 << " machines" << std::endl;

	// Begin measuring solving time
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	z3::set_param("pp.decimal", true);

	// Export formula into .smt file
	// std::ofstream outputFile_smt2("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/encoder_bool_formula.smt"); // TODO (Igor) Exchange path with config value
	// outputFile_smt2 << z3Optimizer.to_smt2() << std::endl;
	// outputFile_smt2.close();

	if (z3Optimizer.check() == z3::sat){
		// End measuring solving time in case of sat
		end = std::chrono::high_resolution_clock::now();

		// Add sat to stats
		outputFile << "z3Optimizer.check() returns sat" << std::endl;

		logger->log_info(name(), "Finished solving and optimizing formula with SAT");

		z3::model model = z3Optimizer.get_model();

		for(unsigned i=0; i<model.size(); ++i) {
			z3::func_decl function = model[i];
			// std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;

			std::string function_name = function.name().str();
			z3::expr expr = model.get_const_interp(function);
			int interp;
			Z3_get_numeral_int(_z3_context, expr, &interp);

			for(int j=1; j<number_machines+1; ++j){
				if(use_encoder_bool) {
					for(int k=1; k<number_machines+1; ++k){
						if(expr.is_bool() && expr.bool_value()>0) {
							std::string compare_with_1 = "pos_1_";
							compare_with_1 += std::to_string(j);
							compare_with_1 += "_";
							compare_with_1 += std::to_string(k);
							std::string compare_with_2 = "pos_2_";
							compare_with_2 += std::to_string(j);
							compare_with_2 += "_";
							compare_with_2 += std::to_string(k);
							std::string compare_with_3 = "pos_3_";
							compare_with_3 += std::to_string(j);
							compare_with_3 += "_";
							compare_with_3 += std::to_string(k);

							if(function_name.compare(compare_with_1)==0) {
								actions_robot_1[j] = node_names_[k];
							}
							else if(function_name.compare(compare_with_2)==0) {
								actions_robot_2[j] = node_names_[k];
							}
							else if(function_name.compare(compare_with_3)==0) {
								actions_robot_3[j] = node_names_[k];
							}
						}
					}
				} else {
					if(interp>0) {
						std::string compare_with_1 = "pos_1_";
						compare_with_1 += std::to_string(j);
						std::string compare_with_2 = "pos_2_";
						compare_with_2 += std::to_string(j);
						std::string compare_with_3 = "pos_3_";
						compare_with_3 += std::to_string(j);

						if(function_name.compare(compare_with_1)==0) {
							actions_robot_1[j] = node_names_[interp];
						}
						else if(function_name.compare(compare_with_2)==0) {
							actions_robot_2[j] = node_names_[interp];
						}
						else if(function_name.compare(compare_with_3)==0) {
							actions_robot_3[j] = node_names_[interp];
						}
					}
				}
			}
		}

		// Add plan specified by the model to stats
		outputFile << std::endl << "Begin of plan" << std::endl << "R-1 goes to "<< actions_robot_1.size() << " machines:";
		std::map<int, std::string>::iterator it_actions_1;
		for(it_actions_1 = actions_robot_1.begin(); it_actions_1!=actions_robot_1.end(); ++it_actions_1) {
			outputFile << " [" << it_actions_1->first << "] " << it_actions_1->second;
		}
		outputFile << std::endl << "R-2 goes to "<< actions_robot_2.size() << " machines:";
		std::map<int, std::string>::iterator it_actions_2;
		for(it_actions_2 = actions_robot_2.begin(); it_actions_2!=actions_robot_2.end(); ++it_actions_2) {
			outputFile << " [" << it_actions_2->first << "] " << it_actions_2->second;
		}
		outputFile << std::endl << "R-3 goes to "<< actions_robot_3.size() << " machines:";
		std::map<int, std::string>::iterator it_actions_3;
		for(it_actions_3 = actions_robot_3.begin(); it_actions_3!=actions_robot_3.end(); ++it_actions_3) {
			outputFile << " [" << it_actions_3->first << "] " << it_actions_3->second;
		}
		outputFile << std::endl << "End of plan"<< std::endl << std::endl;

	} else {

		// End measuring solving time in case of unsat
		end = std::chrono::high_resolution_clock::now();

		// Add unsat to stats
		outputFile << "z3Optimizer.check() returns unsat" << std::endl;

		logger->log_info(name(), "Finished solving and optimizing formula with UNSAT");
	}

	if(add_constraint_closest_node && !use_encoder_bool){
		outputFile << "With additional constraint_closest_node";
		if(use_furthest) outputFile << " [Furthest]" << std::endl << std::endl;
		else outputFile << " [Closest]" << std::endl << std::endl;
	}
	else if(use_encoder_bool) outputFile << "Using encoder_bool" << std::endl << std::endl;

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	// logger->log_info(name(), "Time difference is %f ms", diff); // Measure time in nanoseconds but display in milliseconds for convenience

	outputFile << "Time used for solving: " << diff_ms << " ms, " << diff_m << " m" << std::endl << "__________ __________ __________";
	outputFile.close();
}

/**
 * Test methods
 * - z3: Export carl formula into .smt file and import .smt file back into z3 formula in order to call the z3 solver
 * - carl: Call a carl method and compare computed to known output
 * - formulaGenerator: Similar to z3 test method, but with test case of formulaGenerator
 **/

void
ClipsSmtThread::clips_smt_test_z3()
{

	logger->log_info(name(), "Test z3 extern binary");

	logger->log_info(name(), "Setup carl test formula");
	carl::Variable x = carl::freshRealVariable("x");
	Rational r = 4;
	carl::MultivariatePolynomial<Rational> mp = Rational(r*r)*x*x + r*x + r;
	carl::Formula<carl::MultivariatePolynomial<Rational>> f(mp, carl::Relation::GEQ);

	logger->log_info(name(), "Export carl test formula to file carl_formula.smt");
	std::ofstream outputFile("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/carl_formula.smt"); // TODO (Igor) Exchange path with config value
	outputFile << carl::outputSMTLIB(carl::Logic::QF_NRA, {f});
	outputFile.close();

	logger->log_info(name(), "Import carl test formula from file carl_formula.smt into z3 formula");

	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, "/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/carl_formula.smt", 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	z3::solver s(_z3_context);
	s.add(e);
	if(s.check() == z3::sat) logger->log_info(name(), "Test of import .smt file into z3 constraint did work");
	else logger->log_info(name(), "Test of import .smt file into z3 constraint did NOT work");

	// std::remove("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/carl_formula.smt"); // TODO (Igor) Add functionality to remove intermediate formula.smt
}

void ClipsSmtThread::clips_smt_test_formulaGenerator()
{
	logger->log_info(name(), "Test FormulaGenerator extern binary");

	carl::Variable x = carl::freshRealVariable("x");
	Rational r = 4;
	carl::MultivariatePolynomial<Rational> mp = Rational(r*r)*x*x + r*x + r;
	carl::Formula<carl::MultivariatePolynomial<Rational>> f(mp, carl::Relation::GEQ);

	GameData gD = FormulaGeneratorTest::createGameDataTestCase();
	FormulaGenerator fg = FormulaGenerator(1, gD);

	logger->log_info(name(), "Export FormulaGenerator formula to file fg_formula.smt");
	std::ofstream outputFile("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt"); // TODO (Igor) Exchange path with config value
	outputFile << carl::outputSMTLIB(carl::Logic::QF_NIRA, {fg.createFormula()});
	outputFile.close();

	logger->log_info(name(), "Import FormulaGenerator formula from file fg_formula.smt into z3 formula");

	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, "/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt", 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	z3::solver s(_z3_context);
	s.add(e);
	if(s.check() == z3::sat) {
		logger->log_info(name(), "Test of import fg_formula.smt file into z3 constraint did work");

		z3::model model = s.get_model();
		for(unsigned i=0; i<model.size(); ++i) {
			z3::func_decl function = model[i];
			std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;
		}
	}
	else logger->log_info(name(), "Test of import fg_formula.smt file into z3 constraint did NOT work");

	//std::remove("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt"); // TODO (Igor) Add functionality to remove intermediate formula.smt

}
