
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
	clips_smt_test_formulaGenerator();

	// initialize maps for Francesco's encoder for C0-C1
	state1_machines["not_prep"]=0;
	state1_machines["retrieve_C1"]=1;
	state1_machines["retrieve_C2"]=2;
	state1_machines["mount_C1"]=3;
	state1_machines["mount_C2"]=4;
	state1_machines["prep_B1"]=5;
	state1_machines["prep_B2"]=6;
	state1_machines["prep_B3"]=7;
	state1_machines["slide_one_prep_B1C1"]=8;
	state1_machines["slide_one_prep_B1C2"]=9;
	state1_machines["slide_one_prep_B2C1"]=10;
	state1_machines["slide_one_prep_B2C2"]=11;
	state1_machines["slide_one_prep_B3C1"]=12;
	state1_machines["slide_one_prep_B3C2"]=13;
	state1_machines["mount_R1"]=14;
	state1_machines["slide_one_prep_B1R1C1"]=15;
	state1_machines["slide_one_prep_B1R1C2"]=16;
	state1_machines["slide_one_prep_B2R1C1"]=17;
	state1_machines["slide_one_prep_B2R1C2"]=18;
	state1_machines["slide_one_prep_B3R1C1"]=19;
	state1_machines["slide_one_prep_B3R1C2"]=20;

	state2_machines["empty"]=0;
	state2_machines["has_C1"]=1;
	state2_machines["has_C2"]=2;
	state2_machines["has_R1"]=3;

	state3_machines["full"]=-1;
	state3_machines["empty"]=0;
	state3_machines["B1"]=1;
	state3_machines["B2"]=2;
	state3_machines["B3"]=3;
	state3_machines["B1C1"]=4;
	state3_machines["B1C2"]=5;
	state3_machines["B2C1"]=6;
	state3_machines["B2C2"]=7;
	state3_machines["B3C1"]=8;
	state3_machines["B3C2"]=9;
	state3_machines["BRC1"]=10;
	state3_machines["BRC2"]=11;
	state3_machines["B1R1"]=12;
	state3_machines["B2R1"]=13;
	state3_machines["B3R1"]=14;
	state3_machines["B1R1C1"]=15;
	state3_machines["B1R1C2"]=16;
	state3_machines["B2R1C1"]=17;
	state3_machines["B2R1C2"]=18;
	state3_machines["B3R1C1"]=19;
	state3_machines["B3R1C2"]=20;

	products["nothing"]=0;
	products["B1"]=1;
	products["B2"]=2;
	products["B3"]=3;
	products["B1C1"]=4;
	products["B1C2"]=5;
	products["B2C1"]=6;
	products["B2C2"]=7;
	products["B3C1"]=8;
	products["B3C2"]=9;
	products["BRC1"]=10;
	products["BRC2"]=11;
	products["B1R1"]=12;
	products["B2R1"]=13;
	products["B3R1"]=14;
	products["B1R1C1"]=15;
	products["B1R1C2"]=16;
	products["B2R1C1"]=17;
	products["B2R1C2"]=18;
	products["B3R1C1"]=19;
	products["B3R1C2"]=20;

	machine_groups["CS"]=0;
	machine_groups["BS"]=1;
	machine_groups["DS"]=2;
	machine_groups["RS"]=3;
}


void
ClipsSmtThread::finalize()
{
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


		// Francesco
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

		// // Leonard
		// if(r.compare("R-1")==0){
		// 	std::map<int, std::string>::iterator it_actions;
		// 	for(int move_to: actions_robot_fg_1) {
		// 		action = plan->add_actions();
		// 		action->set_name("move");
		// 		param = action->add_params();
		// 		param->set_key("to");
		// 		param->set_value(node_names_[move_to]);
		// 	}
		// }
		// else if(r.compare("R-2")==0){
		// 	std::map<int, std::string>::iterator it_actions;
		// 	for(int move_to: actions_robot_fg_2) {
		// 		action = plan->add_actions();
		// 		action->set_name("move");
		// 		param = action->add_params();
		// 		param->set_key("to");
		// 		param->set_value(node_names_[move_to]);
		// 	}
		// }
		// else if(r.compare("R-3")==0){
		// 	std::map<int, std::string>::iterator it_actions;
		// 	for(int move_to: actions_robot_fg_3) {
		// 		action = plan->add_actions();
		// 		action->set_name("move");
		// 		param = action->add_params();
		// 		param->set_key("to");
		// 		param->set_value(node_names_[move_to]);
		// 	}
		// }
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

	actions_robot_fg_1.clear();
	actions_robot_fg_2.clear();
	actions_robot_fg_3.clear();

	number_robots = data.robots().size()-1;
	number_machines = data.machines().size();
	number_bits = ceil(log2(number_machines));

	// Compute distances between nodes using navgraph
	clips_smt_fill_node_names();
	clips_smt_fill_robot_names();
	clips_smt_compute_distances_robots();
	clips_smt_compute_distances_machines();

	// Test formulaEncoder
	// Declare variable for encoding
	std::map<std::string, z3::expr> varStartTime;
	std::map<std::string, z3::expr> varRobotDuration;
	std::map<std::string, z3::expr> varRobotPosition;
	std::map<std::string, z3::expr> varMachineDuration;
	std::map<std::string, z3::expr> varR;
	std::map<std::string, z3::expr> varA;
	std::map<std::string, z3::expr> varM;
	std::map<std::string, z3::expr> varHold;
	std::map<std::string, z3::expr> varS;
	std::map<std::string, z3::expr> varRew;
	std::map<std::string, z3::expr> varInit;

	// Declare formulas for encoding
	z3::expr_vector formula = clips_smt_encoder(varStartTime, varRobotDuration, varRobotPosition, varMachineDuration, varR, varA, varM, varHold, varS, varRew, varInit);

	// Give it to z3 solver
	clips_smt_solve_formula(formula);

	// Test formulaGenerator
	// logger->log_info(name(), "Convert protobuf to gamedata");
	// clips_smt_convert_protobuf_to_gamedata();

	// // Test precomputed .smt2 files
	// logger->log_info(name(), "Call precomputed .smt2 file and optimize.");
	// clips_smt_optimize_formula_from_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/FormulaEncodings/benchmark_navgraph-costs-027_planner.smt2", "d_1");

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

	// C-ins-in is never in the list of machines, therefore add it here
	node_names_[0] = "C-ins-in";

	// // Read names of machines automatically
	// for(int i=0; i<number_machines; ++i){
	// 	std::string machine_name = data.machines(i).name().c_str();
	// 	machine_name += "-I";
	// 	// logger->log_info(name(), "Add %s to node_names_", machine_name.c_str());
	// 	node_names_[i+1] = machine_name;
	// }

	// Set names of machines fix
	// node_names_[1] = "C-BS-I";
	// node_names_[2] = "C-RS1-I";
	// node_names_[3] = "C-RS2-I";
	// node_names_[4] = "C-CS1-I";
	// node_names_[5] = "C-CS2-I";
	// node_names_[6] = "C-DS-I";
	// node_names_[7] = "C-BS-O";
	// node_names_[8] = "C-RS1-O";
	// node_names_[9] = "C-RS2-O";
	// node_names_[10] = "C-CS1-O";
	// node_names_[11] = "C-CS2-O";
	// node_names_[12] = "C-DS-O";

	node_names_[1] = "C-BS-I";
	node_names_[2] = "C-CS1-I";
	node_names_[3] = "C-CS1-O";
	node_names_[4] = "C-DS-I";
	node_names_[5] = "C-RS1-I";
	node_names_[6] = "C-RS1-I";

	node_names_inverted["C-ins-in"] = 0;
	node_names_inverted["C-BS-I"] = 1;
	node_names_inverted["C-CS1-I"] = 2;
	node_names_inverted["C-CS2-O"] = 3;
	node_names_inverted["C-DS-I"] = 4;
	node_names_inverted["C-RS1-I"] = 5;
	node_names_inverted["C-RS1-O"] = 6;
}

void
ClipsSmtThread::clips_smt_fill_robot_names()
{
	logger->log_info(name(), "Get name of robots using protobuf data");
	robot_names_.clear();

	// Read names of robots automatically
	int i_true=0;
	for(int i=0; i<number_robots+1; ++i){
		std::string robot_name = data.robots(i).name().c_str();

		if(!robot_name.compare("RefBox")==0) {
			// Not hitting 'RefBox'
			// logger->log_info(name(), "Add %s to robot_names_", robot_name.c_str());
			robot_names_[i_true] = robot_name;
			i_true++;
		}
	}

	// Set names of robots fix
	// robot_names_[0] = "R-1";
	// robot_names_[1] = "R-2";
	// robot_names_[2] = "R-3";
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

	// Prepare file stream to export the distances between machines
	ofstream myfile;
	myfile.open ("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/navgraph-costs-000.csv");

	// Compute distances between unconnected C-ins-in and all other machines
	NavGraphNode ins_node(navgraph->node("C-ins-in"));
	NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		std::pair<std::string, std::string> nodes_pair(ins_node.name(), node_names_[i]);

		NavGraphNode to = navgraph->node(node_names_[i]);
		// logger->log_info(name(), "Position of machine %s is (%f,%f)", to.name().c_str(), to.x(), to.y());

		NavGraphPath p = navgraph->search_path(from, to);

		// logger->log_info(name(), "Distance between node %s and node %s is %f", from.name().c_str(), node_names_[i].c_str(), p.cost());
		myfile << "C-ins-in;" << node_names_[i].c_str() <<";" << p.cost()+navgraph->cost(from, ins_node) << "\n";
		distances_[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
	}

	// Compute distances between machines
	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		for (unsigned int j = 1; j < node_names_.size(); ++j) {
			if (i == j) continue;
			std::pair<std::string, std::string> nodes_pair(node_names_[i], node_names_[j]);

			NavGraphPath p = navgraph->search_path(node_names_[i], node_names_[j]);
			// logger->log_info(name(), "Distance between node %s and node %s is %f", node_names_[i].c_str(), node_names_[j].c_str(), p.cost());
			myfile << node_names_[i].c_str() << ";" << node_names_[j].c_str() <<";" << p.cost() << "\n";
			distances_[nodes_pair] = p.cost();
		}
	}

	myfile.close();
}

/*
 * Methods for encoding the given protobuf data in formulas
 *	- Encode formula for exploration phase by Francesco Leofante
 *	- Encode formula for game by Leonard Kopp
 */

// First C0-C1 encoder

z3::expr_vector
ClipsSmtThread::clips_smt_encoder(std::map<std::string, z3::expr>& varStartTime,
									std::map<std::string, z3::expr>& varRobotDuration,
									std::map<std::string, z3::expr>& varRobotPosition,
									std::map<std::string, z3::expr>& varMachineDuration,
									std::map<std::string, z3::expr>& varR,
									std::map<std::string, z3::expr>& varA,
									std::map<std::string, z3::expr>& varM,
									std::map<std::string, z3::expr>& varHold,
									std::map<std::string, z3::expr>& varS,
									std::map<std::string, z3::expr>& varRew,
									std::map<std::string, z3::expr>& varInit
								)
{
	logger->log_info(name(), "Create z3 encoder");

	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);

	std::map<std::string, z3::expr>::iterator it_map;
	std::map<int, std::string>::iterator it_node_names;

	logger->log_info(name(), "Add variables");

	// Variables initDist_i_j
	for(int i = 0; i < number_machines+1; ++i){
		for(int j = i+1; j < number_machines+1; ++j) {
			z3::expr var(_z3_context);
			std::string varName = "initDist_" + std::to_string(i) + "_" + std::to_string(j);
			var=_z3_context.real_const((varName).c_str());
			varInit.insert(std::make_pair(varName, var));
		}
	}

	// Variables initPos and initHold
	for(int i = 1; i < number_robots+1; ++i){
		z3::expr var(_z3_context);
		std::string varName = "initPos_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varInit.insert(std::make_pair(varName, var));

		varName= "initHold_" + std::to_string(i);
		var =_z3_context.int_const((varName).c_str());
		varInit.insert(std::make_pair(varName, var));
	}

	// Variables initState1_i, initState2_i and initState3_i
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		z3::expr var(_z3_context);
		std::string varName = "initState1_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varInit.insert(std::make_pair(varName, var));

		varName = "initState2_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varInit.insert(std::make_pair(varName, var));

		varName = "initState3_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varInit.insert(std::make_pair(varName, var));
	}

	// Variables depending on plan_horizon
	for(int i=1; i<plan_horizon+1; ++i){
		z3::expr var(_z3_context);
		std::string varName = "t_" + std::to_string(i);
		var=_z3_context.real_const((varName).c_str());
		varStartTime.insert(std::make_pair(varName, var));

		varName = "rd_" + std::to_string(i);
		var=_z3_context.real_const((varName).c_str());
		varRobotDuration.insert(std::make_pair(varName, var));

		varName = "pos_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varRobotPosition.insert(std::make_pair(varName, var));

		varName = "md_" + std::to_string(i);
		var=_z3_context.real_const((varName).c_str());
		varMachineDuration.insert(std::make_pair(varName, var));

		varName = "R_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varR.insert(std::make_pair(varName, var));

		varName = "A_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varA.insert(std::make_pair(varName, var));

		varName = "M_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varM.insert(std::make_pair(varName, var));

		varName = "holdA_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varHold.insert(std::make_pair(varName, var));

		varName = "state1A_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varS.insert(std::make_pair(varName, var));

		varName = "state2A_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varS.insert(std::make_pair(varName, var));

		varName = "state3A_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varS.insert(std::make_pair(varName, var));

		varName = "holdB_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varHold.insert(std::make_pair(varName, var));

		varName = "state1B_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varS.insert(std::make_pair(varName, var));

		varName = "state2B_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varS.insert(std::make_pair(varName, var));

		varName = "state3B_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
		varS.insert(std::make_pair(varName, var));

		varName = "rew" + std::to_string(i);
		var=_z3_context.real_const((varName).c_str());
		varRew.insert(std::make_pair(varName, var));
	}

	logger->log_info(name(), "Add constraints");

	// Init variable true and false
	z3::expr var_false(_z3_context);
	var_false = _z3_context.bool_val(false);
	z3::expr var_true(_z3_context);
	var_true = _z3_context.bool_val(true);

	// // Get distances
	// float min_distance_first=100, min_distance=100, max_distance_first=0, max_distance=0;
	// int min_m_first=0, max_m_first=0;

	// // Get min_distance and max_distance
	// for(int n = 0; n < number_machines+1; ++n) {
	// 	for(int m = 1; m < number_machines+1; ++m) {
	// 		if(n!=m) {
	// 			float distance = distances_[std::make_pair(node_names_[n], node_names_[m])];
	//
	// 			if(distance<min_distance) {
	// 				min_distance = distance;
	// 			}
	//
	// 			if(distance>max_distance) {
	// 				max_distance = distance;
	// 			}
	// 		}
	// 	}
	// }
	//
	// // Get min_distance_first and min_m_first
	// for(int m = 1; m < number_machines+1; ++m) {
	// 	float distance = distances_[std::make_pair("C-ins-in", node_names_[m])];
	//
	// 	if(distance<min_distance_first) {
	// 		min_distance_first = distance;
	// 		min_m_first = m;
	// 	}
	// 	if(distance>max_distance_first) {
	// 		max_distance_first = distance;
	// 		max_m_first = m;
	// 	}
	// }

	// Constraints depending on plan_horizon
	for(int i = 1; i < plan_horizon+1; ++i){

		// VarStartTime
		z3::expr var_t_i(_z3_context);
		z3::expr var_t_i_1(_z3_context);

		it_map = varStartTime.find("t_"+std::to_string(i));
		if(it_map != varStartTime.end()) {
			var_t_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_t_%i not found", i);
		}

		if(i==1){
			z3::expr constraint( 0 <= var_t_i);
			constraints.push_back(constraint);
		}
		else {
			it_map = varStartTime.find("t_"+std::to_string(i-1));
			if(it_map != varStartTime.end()) {
				var_t_i_1 = it_map->second;
			}
			else {
				logger->log_error(name(), "var_t_%i_1 not found", i);
			}

			z3::expr constraint( var_t_i_1 <= var_t_i);
			constraints.push_back(constraint);
		}

		// VarRobotDuration
		it_map = varRobotDuration.find("rd_"+std::to_string(i));
		if(it_map != varRobotDuration.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( 0 <= var);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_rd_%i not found", i);
		}

		// VarRobotPosition
		it_map = varRobotPosition.find("pos_"+std::to_string(i));
		if(it_map != varRobotPosition.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( 1 <= var && var <= number_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_pos_%i not found", i);
		}

		// VarMachineDuration
		it_map = varMachineDuration.find("md_"+std::to_string(i));
		if(it_map != varMachineDuration.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( 0 <= var);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_md_%i not found", i);
		}

		// VarR
		it_map = varR.find("R_"+std::to_string(i));
		if(it_map != varR.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( 1 <= var && var <= number_robots);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_R_%i not found", i);
		}

		// VarA
		it_map = varA.find("A_"+std::to_string(i));
		if(it_map != varA.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( 1 <= var && var <= number_actions);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_A_%i not found", i);
		}

		// VarM
		it_map = varM.find("M_"+std::to_string(i));
		if(it_map != varM.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_machine_groups <= var && var <= max_machine_groups);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_M_%i not found", i);
		}

		// VarHoldA
		it_map = varHold.find("holdA_"+std::to_string(i));
		if(it_map != varHold.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_products <= var && var <= max_products);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_holdA_%i not found", i);
		}

		// VarState1A
		it_map = varS.find("state1A_"+std::to_string(i));
		if(it_map != varS.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_state1_machines <= var && var <= max_state1_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_state1A_%i not found", i);
		}
		// VarState2A
		it_map = varS.find("state2A_"+std::to_string(i));
		if(it_map != varS.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_state2_machines <= var && var <= max_state2_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_state2A_%i not found", i);
		}
		// VarState3A
		it_map = varS.find("state3A_"+std::to_string(i));
		if(it_map != varS.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_state3_machines <= var && var <= max_state3_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_state3A_%i not found", i);
		}

		// VarHoldB
		it_map = varHold.find("holdB_"+std::to_string(i));
		if(it_map != varHold.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_products <= var && var <= max_products);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_holdB_%i not found", i);
		}

		// VarState1B
		it_map = varS.find("state1B_"+std::to_string(i));
		if(it_map != varS.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_state1_machines <= var && var <= max_state1_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_state1B_%i not found", i);
		}
		// VarState2A
		it_map = varS.find("state2B_"+std::to_string(i));
		if(it_map != varS.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_state2_machines <= var && var <= max_state2_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_state2B_%i not found", i);
		}
		// VarState3A
		it_map = varS.find("state3B_"+std::to_string(i));
		if(it_map != varS.end()) {
			z3::expr var = it_map->second;
			z3::expr constraint( min_state3_machines <= var && var <= max_state3_machines);
			constraints.push_back(constraint);
		}
		else {
			logger->log_error(name(), "var_state3B_%i not found", i);
		}

	}

	// Constraint: robot states are initially consistent
	for(int i=1; i<number_robots+1; ++i){
		z3::expr var_h_i(_z3_context);
		z3::expr var_pos_i(_z3_context);

		it_map = varInit.find("initHold_"+std::to_string(i));
		if(it_map != varInit.end()) {
			var_h_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_h_%i not found", i);
		}
		it_map = varInit.find("initPos_"+std::to_string(i));
		if(it_map != varInit.end()) {
			var_pos_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_pos_%i not found", i);
		}

		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr var_r_ip(_z3_context);
			z3::expr var_t_ip(_z3_context);
			z3::expr var_h_ip(_z3_context);
			z3::expr var_pos_ip(_z3_context);

			it_map = varR.find("R_"+std::to_string(ip));
			if(it_map != varR.end()) {
				var_r_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_r_%i not found", ip);
			}
			it_map = varStartTime.find("t_"+std::to_string(ip));
			if(it_map != varStartTime.end()) {
				var_t_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_t_%i not found", ip);
			}
			it_map = varHold.find("holdA_"+std::to_string(ip));
			if(it_map != varHold.end()) {
				var_h_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_holdA_%i not found", ip);
			}
			it_map = varRobotPosition.find("pos_"+std::to_string(ip));
			if(it_map != varRobotPosition.end()) {
				var_pos_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i not found", ip);
			}

			z3::expr constraint1( !(var_r_ip == i));

			for(int ipp=1; ipp<ip; ++ipp){
				it_map = varR.find("R_"+std::to_string(ipp));
				if(it_map != varR.end()) {
					z3::expr var = it_map->second;

					constraint1 = constraint1 || var==i;
				}
				else {
					logger->log_error(name(), "var_r_%i not found", ipp);
				}
			}

			z3::expr constraint2(var_false);

			for(int k=0; k<number_machines+1; ++k){
				for(int l=1; l<number_machines+1; ++l){
					z3::expr var_d_k_l(_z3_context);
					z3::expr var_d_l_k(_z3_context);

					it_map = varInit.find("initDist_"+std::to_string(k)+"_"+std::to_string(l));
					if(it_map != varInit.end()) {
						var_d_k_l = it_map->second;
					}
					else {
						logger->log_error(name(), "var_d_%i_%i not found", k,l);
					}
					it_map = varInit.find("initDist_"+std::to_string(l)+"_"+std::to_string(k));
					if(it_map != varInit.end()) {
						var_d_l_k = it_map->second;
					}
					else {
						logger->log_error(name(), "var_d_%i_%i not found", l,k);
					}

					if(k<l){
						constraint2 = constraint2 || (var_pos_i==k && var_pos_ip==l && var_t_ip>=var_d_k_l);
					}
					else if(l<k){
						constraint2 = constraint2 || (var_pos_i==k && var_pos_ip==l && var_t_ip>=var_d_l_k);
					}
					else {
						constraint2 = constraint2 || (var_pos_i==k && var_pos_ip==l && var_t_ip>=0);
					}
				}
			}

			constraints.push_back(constraint1 || (var_h_ip==var_h_i && constraint2));
		}
	}

	// Constraint: robot states are inductively consistent
	for(int i=1; i<plan_horizon+1; ++i){
		z3::expr var_r_i(_z3_context);
		z3::expr var_t_i(_z3_context);
		z3::expr var_h_i(_z3_context);
		z3::expr var_pos_i(_z3_context);
		z3::expr var_rd_i(_z3_context);


		it_map = varR.find("R_"+std::to_string(i));
		if(it_map != varR.end()) {
			var_r_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_r_%i not found", i);
		}
		it_map = varStartTime.find("t_"+std::to_string(i));
		if(it_map != varStartTime.end()) {
			var_t_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_t_%i not found", i);
		}
		it_map = varHold.find("holdB_"+std::to_string(i));
		if(it_map != varHold.end()) {
			var_h_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_holdB_%i not found", i);
		}
		it_map = varRobotPosition.find("pos"+std::to_string(i));
		if(it_map != varRobotPosition.end()) {
			var_pos_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_pos_%i not found", i);
		}
		it_map = varRobotDuration.find("rd_"+std::to_string(i));
		if(it_map != varRobotDuration.end()) {
			var_rd_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_rd_%i not found", i);
		}

		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr var_r_ip(_z3_context);
			z3::expr var_t_ip(_z3_context);
			z3::expr var_h_ip(_z3_context);
			z3::expr var_pos_ip(_z3_context);

			it_map = varR.find("R_"+std::to_string(ip));
			if(it_map != varR.end()) {
				var_r_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_r_%i not found", ip);
			}
			it_map = varStartTime.find("t_"+std::to_string(ip));
			if(it_map != varStartTime.end()) {
				var_t_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_t_%i not found", ip);
			}
			it_map = varHold.find("holdA_"+std::to_string(ip));
			if(it_map != varHold.end()) {
				var_h_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_holdA_%i not found", ip);
			}
			it_map = varRobotPosition.find("pos_"+std::to_string(ip));
			if(it_map != varRobotPosition.end()) {
				var_pos_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i not found", ip);
			}

			z3::expr constraint1( !(var_r_ip == var_r_i));

			for(int ipp=i+1; ipp<ip; ++ipp){
				it_map = varR.find("R_"+std::to_string(ipp));
				if(it_map != varR.end()) {
					z3::expr var = it_map->second;

					constraint1 = constraint1 || var==var_r_i;
				}
				else {
					logger->log_error(name(), "var_r_%i not found", ipp);
				}
			}

			z3::expr constraint2(var_false);

			for(int k=1; k<number_machines+1; ++k){
				for(int l=1; l<number_machines+1; ++l){
					z3::expr var_d_k_l(_z3_context);
					z3::expr var_d_l_k(_z3_context);

					it_map = varInit.find("initDist_"+std::to_string(k)+"_"+std::to_string(l));
					if(it_map != varInit.end()) {
						var_d_k_l = it_map->second;
					}
					else {
						logger->log_error(name(), "var_d_%i_%i not found", k,l);
					}
					it_map = varInit.find("initDist_"+std::to_string(l)+"_"+std::to_string(k));
					if(it_map != varInit.end()) {
						var_d_l_k = it_map->second;
					}
					else {
						logger->log_error(name(), "var_d_%i_%i not found", l,k);
					}

					if(k<l){
						constraint2 = constraint2 || (var_pos_i==k && var_pos_ip==l && var_t_ip>=var_t_i+var_rd_i+var_d_k_l);
					}
					else if(l<k){
						constraint2 = constraint2 || (var_pos_i==k && var_pos_ip==l && var_t_ip>=var_t_i+var_rd_i+var_d_l_k);
					}
					else {
						constraint2 = constraint2 || (var_pos_i==k && var_pos_ip==l && var_t_ip>=var_t_i+var_rd_i);
					}
				}
			}

			constraints.push_back(constraint1 || (var_h_ip==var_h_i && constraint2));
		}
	}

	// Constraint: machine states are initially consistent
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		z3::expr var_s1_i(_z3_context);
		z3::expr var_s2_i(_z3_context);
		z3::expr var_s3_i(_z3_context);


		it_map = varInit.find("initState1_"+std::to_string(i));
		if(it_map != varInit.end()) {
			var_s1_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_s1_%i not found", i);
		}
		it_map = varInit.find("initState2_"+std::to_string(i));
		if(it_map != varInit.end()) {
			var_s2_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_s2_%i not found", i);
		}
		it_map = varInit.find("initState3_"+std::to_string(i));
		if(it_map != varInit.end()) {
			var_s3_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_s3_%i not found", i);
		}

		for(int ip=1; ip<plan_horizon+1; ++ip){
			z3::expr var_m_ip(_z3_context);
			z3::expr var_s1_ip(_z3_context);
			z3::expr var_s2_ip(_z3_context);
			z3::expr var_s3_ip(_z3_context);

			it_map = varM.find("M_"+std::to_string(ip));
			if(it_map != varM.end()) {
				var_m_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_m_%i not found", ip);
			}
			it_map = varS.find("state1A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_s1_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_s1_%i not found", ip);
			}
			it_map = varS.find("state2A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_s2_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_s2_%i not found", ip);
			}
			it_map = varS.find("state3A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_s3_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_s3_%i not found", i);
			}

			z3::expr constraint1( !(var_m_ip == i));

			for(int ipp=1; ipp<ip; ++ip){
				it_map = varM.find("M_"+std::to_string(ipp));
				if(it_map != varM.end()) {
					z3::expr var = it_map->second;
					constraint1 = constraint1 || (var == i);
				}
				else {
					logger->log_error(name(), "var_m_%i not found", ipp);
				}
			}

			z3::expr constraint2(var_s1_ip = var_s1_i && var_s2_ip = var_s2_i && var_s3_ip = var_s3_i);

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Constraint: machine states are inductively consistent
	for(int i=1; i<plan_horizon+1; ++i){
		z3::expr var_m_i(_z3_context);
		z3::expr var_t_i(_z3_context);
		z3::expr var_d_i(_z3_context);
		z3::expr var_s1_i(_z3_context);
		z3::expr var_s2_i(_z3_context);
		z3::expr var_s3_i(_z3_context);

		it_map = varM.find("M_"+std::to_string(i));
		if(it_map != varM.end()) {
			var_m_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_M_%i not found", i);
		}
		it_map = varStartTime.find("t_"+std::to_string(i));
		if(it_map != varStartTime.end()) {
			var_t_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_t_%i not found", i);
		}
		it_map = varMachineDuration.find("md_"+std::to_string(i));
		if(it_map != varMachineDuration.end()) {
			var_d_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_md_%i not found", i);
		}
		it_map = varS.find("state1B_"+std::to_string(i));
		if(it_map != varS.end()) {
			var_s1_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_s1_%i not found", i);
		}
		it_map = varS.find("state2B_"+std::to_string(i));
		if(it_map != varS.end()) {
			var_s2_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_s2_%i not found", i);
		}
		it_map = varS.find("state3B_"+std::to_string(i));
		if(it_map != varS.end()) {
			var_s3_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_s3_%i not found", i);
		}

		for(int ip=i+1; ip<plan_horizon+1; ++ip){
			z3::expr var_m_ip(_z3_context);
			z3::expr var_t_ip(_z3_context);
			z3::expr var_s1_ip(_z3_context);
			z3::expr var_s2_ip(_z3_context);
			z3::expr var_s3_ip(_z3_context);

			it_map = varM.find("M_"+std::to_string(ip));
			if(it_map != varM.end()) {
				var_m_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_m_%i not found", ip);
			}
			it_map = varStartTime.find("t_"+std::to_string(ip));
			if(it_map != varStartTime.end()) {
				var_t_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_t_%i not found", ip);
			}
			it_map = varS.find("state1A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_s1_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_s1_%i not found", ip);
			}
			it_map = varS.find("state2A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_s2_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_s2_%i not found", ip);
			}
			it_map = varS.find("state3A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_s3_ip = it_map->second;
			}
			else {
				logger->log_error(name(), "var_s3_%i not found", i);
			}

			z3::expr constraint1( !(var_m_ip == var_m_i));

			for(int ipp=i+1; ipp<ip; ++ip){
				it_map = varM.find("M_"+std::to_string(ipp));
				if(it_map != varM.end()) {
					z3::expr var = it_map->second;
					constraint1 = constraint1 || (var == var_m_i);
				}
				else {
					logger->log_error(name(), "var_mp_%i not found", ipp);
				}
			}

			z3::expr constraint2(var_t_ip>=var_t_i+var_d_i && var_s1_ip = var_s1_i && var_s2_ip = var_s2_i && var_s3_ip = var_s3_i);

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Action stuff for every order
	for(int o=0; o<data.orders().size()/2; ++o){

		// Actions for orders of complexity C0-C1 (all, thus we have to make no check)
		// Determine base, ring and cap color
		std::string bi = "B";
		bi += std::to_string(data.orders(o+data.orders().size()/2).base_color());
		std::string ci = "C";
		ci += std::to_string(data.orders(o+data.orders().size()/2).cap_color());
		std::string ri_1 = "R1";
		std::string bi_ci = bi;
		bi_ci += ci;
		std::string bi_ri = bi;
		bi_ri += ri_1;
		std::string bi_ri_ci = bi_ri;
		bi_ri_ci += ci;

		std::string br_ci = "BR";
		br_ci += ci;
		std::string retrieve_ci = "retrieve_";
		retrieve_ci += ci;
		std::string has_ci = "has_";
		has_ci += ci;
		std::string mount_ci = "mount_";
		mount_ci += ci;

		std::string prep_bi = "prep_";
		prep_bi += bi;
		std::string slide_one_prep_bi_ci = "slide_one_prep_";
		slide_one_prep_bi_ci += bi_ci;

		std::string has_ri = "has_";
		has_ri += ri_1;
		std::string mount_ri = "mount_";
		mount_ri += ri_1;
		std::string slide_one_prep_bi_ri_ci = "slide_one_prep_";
		slide_one_prep_bi_ri_ci += bi_ri_ci;

		for(int i=1; i<plan_horizon+1; ++i){
			// Determine variables needed
			z3::expr var_a_i(_z3_context);
			z3::expr var_m_i(_z3_context);
			z3::expr var_state1A_i(_z3_context);
			z3::expr var_state1B_i(_z3_context);
			z3::expr var_state2A_i(_z3_context);
			z3::expr var_state2B_i(_z3_context);
			z3::expr var_state3A_i(_z3_context);
			z3::expr var_state3B_i(_z3_context);
			z3::expr var_md_i(_z3_context);
			z3::expr var_pos_i(_z3_context);
			z3::expr var_holdA_i(_z3_context);
			z3::expr var_holdB_i(_z3_context);
			z3::expr var_rd_i(_z3_context);

			it_map = varA.find("A_"+std::to_string(i));
			if(it_map != varA.end()) {
				var_a_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_a_%i not found", i);
			}
			it_map = varM.find("M_"+std::to_string(i));
			if(it_map != varM.end()) {
				var_m_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_m_%i not found", i);
			}
			it_map = varS.find("state1A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_state1A_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_state1A_%i not found", i);
			}
			it_map = varS.find("state1B_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_state1B_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_state1B_%i not found", i);
			}
			it_map = varS.find("state2A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_state2A_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_state2A_%i not found", i);
			}
			it_map = varS.find("state2B_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_state2B_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_state2B_%i not found", i);
			}
			it_map = varS.find("state3A_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_state3A_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_state3A_%i not found", i);
			}
			it_map = varS.find("state3B_"+std::to_string(i));
			if(it_map != varS.end()) {
				var_state3B_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_state3B_%i not found", i);
			}
			it_map = varMachineDuration.find("md_"+std::to_string(i));
			if(it_map != varMachineDuration.end()) {
				var_md_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_md_%i not found", i);
			}
			it_map = varRobotPosition.find("pos_"+std::to_string(i));
			if(it_map != varRobotPosition.end()) {
				var_pos_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_pos_%i not found", i);
			}
			it_map = varHold.find("holdA_"+std::to_string(i));
			if(it_map != varHold.end()) {
				var_holdA_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_holdA_%i not found", i);
			}
			it_map = varHold.find("holdB_"+std::to_string(i));
			if(it_map != varHold.end()) {
				var_holdB_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_holdB_%i not found", i);
			}
			it_map = varRobotDuration.find("rd_"+std::to_string(i));
			if(it_map != varRobotDuration.end()) {
				var_rd_i = it_map->second;
			}
			else {
				logger->log_error(name(), "var_rd_%i not found", i);
			}

			// 1.Action : retrieve base with cap from shelf at CS
			z3::expr constraint_action1((var_m_i == machine_groups["CS"])
										&& (var_state1B_i == var_state1A_i)
										&& (var_state2B_i == var_state2A_i)
										&& (var_state3B_i == var_state3A_i)
										&& (var_md_i == time_to_fetch)
										&& (var_pos_i == node_names_inverted["C-CS1-I"])
										&& (var_holdA_i == products["nothing"])
										&& (var_holdB_i == products[br_ci])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 1) || constraint_action1);

			// 2.Action : prepare CS to retrieve cap
			z3::expr constraint_action2((var_m_i == machine_groups["CS"])
										&& (var_state1A_i == state1_machines["not_prep"])
										&& (var_state1B_i == state1_machines[retrieve_ci])
										&& (var_state2B_i == var_state2A_i)
										&& (var_state3B_i == var_state3A_i)
										&& (var_md_i == time_to_prep)
										&& (var_pos_i == node_names_inverted["C-CS1-I"])
										&& (var_holdA_i == var_holdB_i)
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 2) || constraint_action2);

			// 3.Action : feed base with cap into CS
			z3::expr constraint_action3((var_m_i == machine_groups["CS"])
										&& (var_state1A_i == state1_machines[retrieve_ci])
										&& (var_state1B_i == state1_machines["not_prep"])
										&& (var_state2A_i == state2_machines["empty"])
										&& (var_state2B_i == state2_machines[has_ci])
										&& (var_state3A_i == state3_machines["empty"])
										&& (var_state3B_i == state3_machines["full"])
										&& (var_md_i == time_to_feed)
										&& (var_pos_i == node_names_inverted["C-CS1-I"])
										&& (var_holdA_i == products[br_ci])
										&& (var_holdB_i == products["nothing"])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 3) || constraint_action3);

			// 4.Action : prepare CS to mount cap
			z3::expr constraint_action4((var_m_i == machine_groups["CS"])
										&& (var_state1A_i == state1_machines["not_prep"])
										&& (var_state1B_i == state1_machines[mount_ci])
										&& (var_state2A_i == state2_machines[has_ci])
										&& (var_state2B_i == state2_machines[has_ci])
										&& (var_state3A_i == state3_machines["empty"])
										&& (var_state3B_i == state3_machines["empty"])
										&& (var_md_i == time_to_prep)
										&& (var_pos_i == node_names_inverted["C-CS1-I"])
										&& (var_holdA_i == var_holdB_i)
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 4) || constraint_action4);

			// 5.Action : feed base to CS to mount cap
			z3::expr constraint_action5((var_m_i == machine_groups["CS"])
										&& (var_state1A_i == state1_machines[mount_ci])
										&& (var_state1B_i == state1_machines["not_prep"])
										&& (var_state2A_i == state2_machines[has_ci])
										&& (var_state2B_i == state2_machines["empty"])
										&& (var_state3A_i == state3_machines["empty"])
										&& (var_state3B_i == state3_machines[bi_ci])
										&& (var_md_i == time_to_feed)
										&& (var_pos_i == node_names_inverted["C-CS1-I"])
										&& (var_holdA_i == products[bi])
										&& (var_holdB_i == products["nothing"])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 5) || constraint_action5);

			// 6.Action : retrieve base from BS
			z3::expr constraint_action6((var_m_i == machine_groups["BS"])
										&& (var_state1A_i == state1_machines[prep_bi])
										&& (var_state1B_i == state1_machines["not_prep"])
										&& (var_state2B_i == var_state2A_i)
										&& (var_state3B_i == var_state3A_i)
										&& (var_md_i == time_to_fetch)
										&& (var_pos_i == node_names_inverted["C-BS-I"])
										&& (var_holdA_i == products["nothing"])
										&& (var_holdB_i == products[bi])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 6) || constraint_action6);

			// 7.Action : prepare BS to provide base
			z3::expr constraint_action7((var_m_i == machine_groups["BS"])
										&& (var_state1A_i == state1_machines["not_prep"])
										&& (var_state1B_i == state1_machines[prep_bi])
										&& (var_state2B_i == var_state2A_i)
										&& (var_state3B_i == var_state3A_i)
										&& (var_md_i == time_to_prep)
										&& (var_pos_i == node_names_inverted["C-BS-I"])
										&& (var_holdA_i == var_holdB_i)
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 7) || constraint_action7);

			// 8.Action : discard capless base from CS
			z3::expr constraint_action8((var_m_i == machine_groups["CS"])
										&& (var_state1A_i == var_state1B_i)
										&& (var_state2A_i == state2_machines[has_ci])
										&& (var_state2B_i == state2_machines[has_ci])
										&& (var_state3A_i == state3_machines["full"])
										&& (var_state3B_i == state3_machines["empty"])
										&& (var_md_i == time_to_disc)
										&& (var_pos_i == node_names_inverted["C-CS1-O"])
										&& (var_holdA_i == products["nothing"])
										&& (var_holdB_i == products["nothing"])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 8) || constraint_action8);

			// 9.Action : retrieve base with cap from CS
			z3::expr constraint_action9((var_m_i == machine_groups["CS"])
										&& (var_state1A_i == var_state1B_i)
										&& (var_state2A_i == state2_machines["empty"])
										&& (var_state2B_i == state2_machines["empty"])
										&& (var_state3A_i == state3_machines[bi_ci])
										&& (var_state3B_i == state3_machines["empty"])
										&& (var_md_i == time_to_fetch)
										&& (var_pos_i == node_names_inverted["C-CS1-O"])
										&& (var_holdA_i == products["nothing"])
										&& (var_holdB_i == products[bi_ci])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 9) || constraint_action9);

			// 10.Action : prepare DS for slide specified order
			z3::expr constraint_action10((var_m_i == machine_groups["DS"])
										&& (var_state1A_i == state1_machines["not_prep"])
										&& (var_state1B_i == state1_machines[slide_one_prep_bi_ci])
										&& (var_state2B_i == var_state2A_i)
										&& (var_state3B_i == var_state3A_i)
										&& (var_md_i == time_to_prep)
										&& (var_pos_i == node_names_inverted["C-DS-I"])
										&& (var_holdA_i == products[bi_ci])
										&& (var_holdB_i == var_holdA_i)
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 10) || constraint_action10);

			// 11.Action : deliver to DS
			z3::expr constraint_action11((var_m_i == machine_groups["DS"])
										&& (var_state1A_i == state1_machines[slide_one_prep_bi_ci])
										&& (var_state1B_i == state1_machines["not_prep"])
										&& (var_state2B_i == var_state2A_i)
										&& (var_state3B_i == var_state3A_i)
										&& (var_md_i == time_to_prep)
										&& (var_pos_i == node_names_inverted["C-DS-I"])
										&& (var_holdA_i == products[bi_ci])
										&& (var_holdB_i == products["nothing"])
										&& (var_rd_i == 0));
			constraints.push_back(!(var_a_i == 11) || constraint_action11);

			// Actions for orders of complexity C1
			if(data.orders(o+data.orders().size()/2).ring_colors().size()>0){
				// 12.Action : prepare RS to mount ring
				z3::expr constraint_action12((var_m_i == machine_groups["RS"])
											&& (var_state1A_i == state1_machines["not_prep"])
											&& (var_state1B_i == state1_machines[mount_ri])
											&& (var_state2A_i == state2_machines[has_ri])
											&& (var_state2B_i == state2_machines[has_ri])
											&& (var_state3A_i == state3_machines["empty"])
											&& (var_state3B_i == state3_machines["empty"])
											&& (var_md_i == time_to_prep)
											&& (var_pos_i == node_names_inverted["C-RS1-I"])
											&& (var_holdA_i == var_holdB_i)
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 12) || constraint_action12);

				// 13.Action : feed base to RS to mount ring
				z3::expr constraint_action13((var_m_i == machine_groups["RS"])
											&& (var_state1A_i == state1_machines[mount_ri])
											&& (var_state1B_i == state1_machines["not_prep"])
											&& (var_state2B_i == var_state2A_i)
											&& (var_state3A_i == state3_machines["empty"])
											&& (var_state3B_i == state3_machines[bi_ri])
											&& (var_md_i == time_to_feed)
											&& (var_pos_i == node_names_inverted["C-RS1-I"])
											&& (var_holdA_i == products[bi])
											&& (var_holdB_i == products["nothing"])
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 13) || constraint_action13);

				// 14.Action : retrieve base with ring from RS
				z3::expr constraint_action14((var_m_i == machine_groups["RS"])
											&& (var_state1B_i == var_state1A_i)
											&& (var_state2B_i == var_state2A_i)
											&& (var_state3A_i == state3_machines[bi_ri])
											&& (var_state3B_i == state3_machines["empty"])
											&& (var_md_i == time_to_fetch)
											&& (var_pos_i == node_names_inverted["C-RS1-O"])
											&& (var_holdA_i == products["nothing"])
											&& (var_holdB_i == products[bi_ri])
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 14) || constraint_action14);

				// 15.Action : feed base with one ring to CS to mount cap
				z3::expr constraint_action15((var_m_i == machine_groups["CS"])
											&& (var_state1A_i == state1_machines[mount_ci])
											&& (var_state1B_i == state1_machines["not_prep"])
											&& (var_state2A_i == state2_machines[has_ci])
											&& (var_state2B_i == state2_machines["empty"])
											&& (var_state3A_i == state3_machines["empty"])
											&& (var_state3B_i == state3_machines[bi_ri_ci])
											&& (var_md_i == time_to_feed)
											&& (var_pos_i == node_names_inverted["C-CS1-I"])
											&& (var_holdA_i == products[bi_ri])
											&& (var_holdB_i == products["nothing"])
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 15) || constraint_action15);

				// 16.Action : retrieve base with one ring and cap from CS
				z3::expr constraint_action16((var_m_i == machine_groups["CS"])
											&& (var_state1B_i == var_state1A_i)
											&& (var_state2A_i == state2_machines["empty"])
											&& (var_state2B_i == state2_machines["empty"])
											&& (var_state3A_i == state3_machines[bi_ri_ci])
											&& (var_state3B_i == state3_machines["empty"])
											&& (var_md_i == time_to_fetch)
											&& (var_pos_i == node_names_inverted["C-CS1-O"])
											&& (var_holdA_i == products["nothing"])
											&& (var_holdB_i == products[bi_ri_ci])
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 16) || constraint_action16);

				// 17.Action : prepare DS for slide specified order
				z3::expr constraint_action17((var_m_i == machine_groups["DS"])
											&& (var_state1A_i == state1_machines["not_prep"])
											&& (var_state1B_i == state1_machines[slide_one_prep_bi_ri_ci])
											&& (var_state2B_i == var_state2A_i)
											&& (var_state3B_i == var_state3A_i)
											&& (var_md_i == time_to_prep)
											&& (var_pos_i == node_names_inverted["C-DS-I"])
											&& (var_holdA_i == products[bi_ri_ci])
											&& (var_holdB_i == var_holdA_i)
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 17) || constraint_action17);

				// 18.Action : deliver base with one ring and cap to DS
				z3::expr constraint_action18((var_m_i == machine_groups["DS"])
											&& (var_state1A_i == state1_machines[slide_one_prep_bi_ri_ci])
											&& (var_state1B_i == state1_machines["not_prep"])
											&& (var_state2B_i == var_state2A_i)
											&& (var_state3B_i == var_state3A_i)
											&& (var_md_i == time_to_prep)
											&& (var_pos_i == node_names_inverted["C-DS-I"])
											&& (var_holdA_i == products[bi_ri_ci])
											&& (var_holdB_i == products["nothing"])
											&& (var_rd_i == 0));
				constraints.push_back(!(var_a_i == 18) || constraint_action18);
			}
		}

		// TODO Add actions for orders of higher complexity
	}

	// Specify goal state
	for(int i=1; i<plan_horizon+1; ++i){
		z3::expr var_a_i(_z3_context);
		z3::expr var_rew_i(_z3_context);
		z3::expr var_t_i(_z3_context);
		z3::expr var_md_i(_z3_context);
		z3::expr var_t_plan_horizon(_z3_context);
		z3::expr var_md_plan_horizon(_z3_context);

		it_map = varA.find("A_"+std::to_string(i));
		if(it_map != varA.end()) {
			var_a_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_a_%i not found", i);
		}
		it_map = varRew.find("rew_"+std::to_string(i));
		if(it_map != varRew.end()) {
			var_rew_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_rew_%i not found", i);
		}
		it_map = varStartTime.find("t_"+std::to_string(i));
		if(it_map != varStartTime.end()) {
			var_t_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_state1A_%i not found", i);
		}
		it_map = varMachineDuration.find("md_"+std::to_string(i));
		if(it_map != varMachineDuration.end()) {
			var_md_i = it_map->second;
		}
		else {
			logger->log_error(name(), "var_md_%i not found", i);
		}
		it_map = varStartTime.find("t_"+std::to_string(plan_horizon));
		if(it_map != varStartTime.end()) {
			var_t_plan_horizon = it_map->second;
		}
		else {
			logger->log_error(name(), "var_t_%i not found", plan_horizon);
		}
		it_map = varMachineDuration.find("state2B_"+std::to_string(plan_horizon));
		if(it_map != varMachineDuration.end()) {
			var_md_plan_horizon = it_map->second;
		}
		else {
			logger->log_error(name(), "var_md_%i not found", plan_horizon);
		}

		if(i==1){
			constraints.push_back((var_a_i == 18 && var_rew_i == (deadline-var_t_i-var_md_i))
									|| (!(var_a_i==18) && var_rew_i==0));
		}
		else {
			z3::expr var_rew_i_1(_z3_context);

			it_map = varRew.find("rew_"+std::to_string(i-1));
			if(it_map != varRew.end()) {
				var_rew_i_1 = it_map->second;
			}
			else {
				logger->log_error(name(), "var_rew_%i not found", i);
			}

			constraints.push_back((var_a_i == 18 && var_rew_i == (var_rew_i_1+deadline-var_t_i-var_md_i))
									|| (!(var_a_i==18) && var_rew_i==var_rew_i_1));
		}
	}

	return constraints;
}

GameData
ClipsSmtThread::clips_smt_convert_protobuf_to_gamedata()
{
	GameData gD = GameData();
	gamedata_robots.clear();
	gamedata_basestations.clear();
	gamedata_ringstations.clear();
	gamedata_capstations.clear();
	gamedata_deliverystations.clear();

	// Machines
	for (int i = 1; i < number_machines+1; i++){
		std::string name_machine = node_names_[i];
		if(name_machine[2] == 'B') { // BaseStation
			auto bs_temp = std::make_shared<BaseStation>(i);
			bs_temp->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER}); // TODO Is this constant behavior
			bs_temp->setDispenseBaseTime(1);
			gamedata_basestations.push_back(bs_temp);
			gD.addMachine(bs_temp);
		}
		else if(name_machine[2] == 'R') { // RingStation
			auto rs_temp = std::make_shared<RingStation>(i);

			/**
			for(int j=0; j<data.machines(i-1).ring_colors().size(); ++j) {
				rs_temp->addPossibleRingColor(static_cast<Workpiece::Color>(data.machines(i-1).ring_colors(j)+1), 1); // TODO Add all possibleRingColors with number of bases required for production ADD NUMBERS
				// rs_temp->setRingColorSetup(static_cast<Workpiece::Color>(data.machines(i-1).ring_colors(j)+1)); // TODO Add ringColorSetup BACK
			}
			**/

			if(name_machine[4] == '1'){
				rs_temp->addPossibleRingColor(Workpiece::ORANGE, 1);
				rs_temp->addPossibleRingColor(Workpiece::BLUE, 1);
			}
			else if(name_machine[4] == '2'){
				rs_temp->addPossibleRingColor(Workpiece::GREEN, 1);
				rs_temp->addPossibleRingColor(Workpiece::YELLOW, 1);
			}

			rs_temp->setFeedBaseTime(1);
			rs_temp->setMountRingTime(10);
			rs_temp->setAdditinalBasesFed(data.machines(i-1).loaded_with());
			gamedata_ringstations.push_back(rs_temp);
			gD.addMachine(rs_temp);
		}
		else if(name_machine[2] == 'C') { // CapStation
			auto cs_temp = std::make_shared<CapStation>(i);
			if(name_machine[4] == '1'){
				cs_temp->addPossibleCapColor(Workpiece::GREY);
				cs_temp->setFedCapColor(Workpiece::GREY); // TODO Add fedCapColor BACK ONLY BUFFER
			}
			else if(name_machine[4] == '2'){
				cs_temp->addPossibleCapColor(Workpiece::BLACK);
				cs_temp->setFedCapColor(Workpiece::BLACK); // TODO Add fedCapColor BACK ONLY BUFFER
			}
			cs_temp->setFeedCapTime(1);
			cs_temp->setMountCapTime(5);
			gamedata_capstations.push_back(cs_temp);
			gD.addMachine(cs_temp);
		}
		else if(name_machine[2] == 'D') { // DeliveryStation
			auto ds_temp = std::make_shared<DeliveryStation>(i);
			gamedata_deliverystations.push_back(ds_temp);
			gD.addMachine(ds_temp);
		}
	}

	// Robots
	for (int i = 0; i < number_robots; i++)
	{
		std::string name_robot = robot_names_[i];
		auto r_temp = std::make_shared<Robot>(i);
		Workpiece pr0 = Workpiece(Workpiece::BLACK,{}, Workpiece::NONE);
		r_temp->setWorkpiece(pr0);
		// TODO Add Workpiece corresponding to robot
		gamedata_robots.push_back(r_temp);
		gD.addMachine(r_temp);
	}

	// TODO Test if correct machines are mapped to each other
	// Note that index m will go over all types of stations
	// Robot machine distance

	int limit_basestation = gamedata_basestations.size();
	int limit_ringstation = limit_basestation + gamedata_ringstations.size();
	int limit_capstation = limit_ringstation + gamedata_capstations.size();
	// int limit_deliverystation = limit_capstation + gamedata_deliverystations.size();

	for(int r=0; r<number_robots; ++r) {
		// logger->log_info(name(), "Watch robot %i", r);
		for(int m=0; m<number_machines; ++m) {
			// logger->log_info(name(), "Watch machine %i named %s", m, node_names_[m+1].c_str());
			float distance = distances_[std::make_pair(robot_names_[r], node_names_[m+1])];

			// logger->log_info(name(), "Distance computed %f", distance);

			if(m<limit_basestation) {
				// Inside gamedata_basestations
				// logger->log_info(name(), "Inside gamedata_basestations");

				Machine::addMovingTime(*gamedata_robots[r], *gamedata_basestations[m], distance);
			}
			else if(m<limit_ringstation) {
				// Inside gamedata_ringstations
				int m_temp = m-gamedata_basestations.size();
				// logger->log_info(name(), "Inside gamedata_ringstations with m_temp %i", m_temp);
				Machine::addMovingTime(*gamedata_robots[r], *gamedata_ringstations[m_temp], distance);
			}
			else if(m<limit_capstation) {
				// Inside gamedata_capstations
				int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
				// logger->log_info(name(), "Inside gamedata_capstations with m_temp %i", m_temp);
				Machine::addMovingTime(*gamedata_robots[r], *gamedata_capstations[m_temp], distance);
			}
			else {
				// Inside gamedata_deliverystations
				int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
				// logger->log_info(name(), "Inside gamedata_deliverystations with m_temp %i", m_temp);
				Machine::addMovingTime(*gamedata_robots[r], *gamedata_deliverystations[m_temp], distance);
			}

			// logger->log_info(name(), "Distance added");
		}
	}

	// Machine machine distance
	for(int n=0; n<number_machines; ++n) {
		for(int m=n+1; m<number_machines; ++m) {
			float distance = distances_[std::make_pair(node_names_[n+1], node_names_[m+1])];
			if(n<limit_basestation) {
				// n is inside gamedata_basestations
				if(m<limit_basestation) {
					// m is inside gamedata_basestations
					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_basestations[m], distance);
				}
				else if(m<limit_ringstation) {
					// m is inside gamedata_ringstations
					int m_temp = m-gamedata_basestations.size();
					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_ringstations[m_temp], distance);
				}
				else if(m<limit_capstation) {
					// m is inside gamedata_capstations
					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_capstations[m_temp], distance);
				}
				else {
					// m is inside gamedata_deliverystations
					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_deliverystations[m_temp], distance);
				}
			}
			else if(n<limit_ringstation) {
				// n is inside gamedata_ringstations
				int n_temp = n-gamedata_basestations.size();
				if(m<limit_ringstation) {
					// m is inside gamedata_ringstations
					int m_temp = m-gamedata_basestations.size();
					Machine::addMovingTime(*gamedata_ringstations[n_temp], *gamedata_ringstations[m_temp], distance);
				}
				else if(m<limit_capstation) {
					// m is inside gamedata_capstations
					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
					Machine::addMovingTime(*gamedata_ringstations[n_temp], *gamedata_capstations[m_temp], distance);
				}
				else {
					// m is inside gamedata_deliverystations
					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
					Machine::addMovingTime(*gamedata_ringstations[n_temp], *gamedata_deliverystations[m_temp], distance);
				}
			}
			else if(n<limit_capstation) {
				// n is inside gamedata_capstations
				int n_temp = n-gamedata_basestations.size()-gamedata_ringstations.size();
				if(m<limit_capstation) {
					// m is inside gamedata_capstations
					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
					Machine::addMovingTime(*gamedata_capstations[n_temp], *gamedata_capstations[m_temp], distance);
				}
				else {
					// m is inside gamedata_deliverystations
					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
					Machine::addMovingTime(*gamedata_capstations[n_temp], *gamedata_deliverystations[m_temp], distance);
				}
			}
			else {
				// n and m are inside gamedata_deliverystations
				int n_temp = n-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
				int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
				Machine::addMovingTime(*gamedata_deliverystations[n_temp], *gamedata_deliverystations[m_temp], distance);
			}
		}
	}

	// Orders
	for (int i = 0; i < data.orders().size()/2; i++)
	{
		//Color conversions
		Workpiece::Color bc_temp = Workpiece::NONE;
		switch(data.orders(i+data.orders().size()/2).base_color()){
			case 1: bc_temp = static_cast<Workpiece::Color>(Workpiece::RED);
					break;
			case 2: bc_temp = static_cast<Workpiece::Color>(Workpiece::BLACK);
					break;
			case 3: bc_temp = static_cast<Workpiece::Color>(Workpiece::SILVER);
					break;
			default: break;
		}

		std::vector<Workpiece::Color> rc_temps;
		for (int j = 0; j < data.orders(i+data.orders().size()/2).ring_colors().size(); j++){
			switch(data.orders(i+data.orders().size()/2).ring_colors(j)){
				case 1: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::BLUE));
						break;
				case 2: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::GREEN));
						break;
				case 3: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::ORANGE));
						break;
				case 4: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::YELLOW));
						break;
				default: break;
			}

		}

		Workpiece::Color cc_temp = Workpiece::NONE;
		switch(data.orders(i+data.orders().size()/2).cap_color()){
			case 1: cc_temp = static_cast<Workpiece::Color>(Workpiece::BLACK);
					break;
			case 2: cc_temp = static_cast<Workpiece::Color>(Workpiece::GREY);
					break;
			default: break;
		}

		Workpiece p_temp = Workpiece(bc_temp, rc_temps, cc_temp);
		auto o_temp = std::make_shared<Order>(i, p_temp, data.orders(i).delivery_period_end()*100); // TODO Fix third parameter
		gD.addOrder(o_temp);
	}


	logger->log_info(name(), "Create fg with gD");
	FormulaGenerator fg = FormulaGenerator(1, gD);
	// logger->log_info(name(), "Display fg.createFormula()");
	// cout << fg.createFormula() << std::endl << std::endl;
	logger->log_info(name(), "Display gD.toString()");
	cout << gD.toString() << std::endl;

	logger->log_info(name(), "Export GameData formula to file gD_fg_formula.smt");
	std::ofstream outputFile("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/gD_fg_formula.smt"); // TODO (Igor) Exchange path with config value
	outputFile << carl::outputSMTLIB(carl::Logic::QF_NIRA, {fg.createFormula()});
	outputFile.close();

	clips_smt_solve_formula_from_fg_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/gD_fg_formula.smt", fg);


	logger->log_info(name(), "Finish extracting");

	return gD;
}

/*
 * Solve encoding of exploration phase
 *	- Solve from z3 built formula
 *	- Solve from smt file
 *	- Optimize from smt file
 *	- Solve from smt file and export actions according to GameData gD and FormulaGenerator fG
 */

void
 ClipsSmtThread::clips_smt_solve_formula(z3::expr_vector formula)
{
	logger->log_info(name(), "Solve z3 formula");

	z3::optimize z3Optimizer(_z3_context);

	std::map<std::string, z3::expr>::iterator it_map;

	for (unsigned i = 0; i < formula.size(); i++) {
		z3Optimizer.add(formula[i]);
	}

	// // Add objective functions
	// z3::expr d_1_M = _z3_context.real_const(("d_"+std::to_string(1)+"_"+std::to_string(number_machines)).c_str());
	// z3::expr d_2_M = _z3_context.real_const(("d_"+std::to_string(2)+"_"+std::to_string(number_machines)).c_str());
	// z3::expr d_3_M = _z3_context.real_const(("d_"+std::to_string(3)+"_"+std::to_string(number_machines)).c_str());
	// z3::expr m_1 = _z3_context.bool_const(("m_"+std::to_string(1)).c_str());
	// z3::expr m_2 = _z3_context.bool_const(("m_"+std::to_string(2)).c_str());
	// z3::expr m_3 = _z3_context.bool_const(("m_"+std::to_string(3)).c_str());
	//
	// //z3Optimizer.minimize(m_1*d_1_M + m_2*d_2_M + m_3*d_3_M);
	// z3Optimizer.minimize(d_1_M + d_2_M + d_3_M);

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
			std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;

			// std::string function_name = function.name().str();
			// z3::expr expr = model.get_const_interp(function);
			// int interp;
			// Z3_get_numeral_int(_z3_context, expr, &interp);
			//
			// for(int j=1; j<number_machines+1; ++j){
			// 	if(interp>0) {
			// 		std::string compare_with_pos_1 = "pos_1_";
			// 		compare_with_pos_1 += std::to_string(j);
			// 		std::string compare_with_pos_2 = "pos_2_";
			// 		compare_with_pos_2 += std::to_string(j);
			// 		std::string compare_with_pos_3 = "pos_3_";
			// 		compare_with_pos_3 += std::to_string(j);
			//
			// 		if(function_name.compare(compare_with_pos_1)==0) {
			// 			actions_robot_1[j] = node_names_[interp];
			// 		}
			// 		else if(function_name.compare(compare_with_pos_2)==0) {
			// 			actions_robot_2[j] = node_names_[interp];
			// 		}
			// 		else if(function_name.compare(compare_with_pos_3)==0) {
			// 			actions_robot_3[j] = node_names_[interp];
			// 		}
			// 	}
			// }
		}

		// // Add plan specified by the model to stats
		// outputFile << std::endl << "Begin of plan" << std::endl << "R-1 goes to "<< actions_robot_1.size() << " machines:";
		// std::map<int, std::string>::iterator it_actions_1;
		// for(it_actions_1 = actions_robot_1.begin(); it_actions_1!=actions_robot_1.end(); ++it_actions_1) {
		// 	outputFile << " [" << it_actions_1->first << "] " << it_actions_1->second;
		// }
		// outputFile << std::endl << "R-2 goes to "<< actions_robot_2.size() << " machines:";
		// std::map<int, std::string>::iterator it_actions_2;
		// for(it_actions_2 = actions_robot_2.begin(); it_actions_2!=actions_robot_2.end(); ++it_actions_2) {
		// 	outputFile << " [" << it_actions_2->first << "] " << it_actions_2->second;
		// }
		// outputFile << std::endl << "R-3 goes to "<< actions_robot_3.size() << " machines:";
		// std::map<int, std::string>::iterator it_actions_3;
		// for(it_actions_3 = actions_robot_3.begin(); it_actions_3!=actions_robot_3.end(); ++it_actions_3) {
		// 	outputFile << " [" << it_actions_3->first << "] " << it_actions_3->second;
		// }
		// outputFile << std::endl << "End of plan"<< std::endl << std::endl;

	} else {

		// End measuring solving time in case of unsat
		end = std::chrono::high_resolution_clock::now();

		// Add unsat to stats
		outputFile << "z3Optimizer.check() returns unsat" << std::endl;

		logger->log_info(name(), "Finished solving and optimizing formula with UNSAT");
	}

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	// logger->log_info(name(), "Time difference is %f ms", diff); // Measure time in nanoseconds but display in milliseconds for convenience

	outputFile << "Time used for solving: " << diff_ms << " ms, " << diff_m << " m" << std::endl << "__________ __________ __________";
	outputFile.close();
}

void ClipsSmtThread::clips_smt_solve_formula_from_smt_file(std::string path) {
	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	z3::solver s(_z3_context);
	s.add(e);

	// Start measuring sovling time
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
	if(s.check() == z3::sat) {
		// Stop measuring sovling time
		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

		// Compute time for solving
		double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
		double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

		logger->log_info(name(), "Test of import .smt file into z3 constraint did work (SAT) [%f ms, %f m]", diff_ms, diff_m);

		z3::model model = s.get_model();
		logger->log_info(name(), "Display model with %i entries", model.size());
		for(unsigned i=0; i<model.size(); ++i) {
			z3::func_decl function = model[i];
			std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;
		}
	}
	else logger->log_info(name(), "Test of import .smt file into z3 constraint did NOT work (UNSAT)");
}

void ClipsSmtThread::clips_smt_optimize_formula_from_smt_file(std::string path, std::string var) {
	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	z3::optimize o(_z3_context);
	o.add(e);
	z3::expr v = _z3_context.real_const(var.c_str());
	o.minimize(v);

	// Start measuring sovling time
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
	if(o.check() == z3::sat) {
		// Stop measuring sovling time
		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

		// Compute time for solving
		double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
		double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

		int actions_robot_1_discard=12;
		int actions_robot_2_discard=12;
		int actions_robot_3_discard=12;

		logger->log_info(name(), "Test of import .smt file into z3 constraint did work (SAT) [%f ms, %f m]", diff_ms, diff_m);

		z3::model model = o.get_model();
		logger->log_info(name(), "Display model with %i entries", model.size());
		for(unsigned i=0; i<model.size(); ++i) {
			z3::func_decl function = model[i];
			std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;

			// Extract for move actions
			std::string function_name = function.name().str();
			z3::expr expr = model.get_const_interp(function);
			int interp;
			Z3_get_numeral_int(_z3_context, expr, &interp);

			for(int j=1; j<number_machines+1; ++j){
				if(interp>0) {
					std::string compare_with_pos_1 = "pos_1_";
					compare_with_pos_1 += std::to_string(j);
					std::string compare_with_pos_2 = "pos_2_";
					compare_with_pos_2 += std::to_string(j);
					std::string compare_with_pos_3 = "pos_3_";
					compare_with_pos_3 += std::to_string(j);

					std::string compare_with_n_1_12 = "n_1_12";
					std::string compare_with_n_2_12 = "n_2_12";
					std::string compare_with_n_3_12 = "n_3_12";

					if(function_name.compare(compare_with_pos_1)==0) {
						actions_robot_1[j] = node_names_[interp];
					}
					else if(function_name.compare(compare_with_pos_2)==0) {
						actions_robot_2[j] = node_names_[interp];
					}
					else if(function_name.compare(compare_with_pos_3)==0) {
						actions_robot_3[j] = node_names_[interp];
					}
					else if(function_name.compare(compare_with_n_1_12)==0) {
						actions_robot_1_discard = interp;
					}
					else if(function_name.compare(compare_with_n_2_12)==0) {
						actions_robot_2_discard = interp;
					}
					else if(function_name.compare(compare_with_n_3_12)==0) {
						actions_robot_3_discard = interp;
					}
				}
			}
		}

		// Erase not needed information
		for(int i=actions_robot_1_discard+1; i<13; ++i) {
			actions_robot_1.erase(i);
		}
		for(int i=actions_robot_2_discard+1; i<13; ++i) {
			actions_robot_2.erase(i);
		}
		for(int i=actions_robot_3_discard+1; i<13; ++i) {
			actions_robot_3.erase(i);
		}

		// Export stats into clips_smt_thread_stats.txt
		std::ofstream outputFile;
		outputFile.open("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats.txt", std::ofstream::out | std::ofstream::app);

		time_t now = time(0);
		char* dt = ctime(&now);
		outputFile << std::endl << dt << std::endl;

		// Add plan specified by the model to stats
		outputFile << "Begin of plan" << std::endl << "R-1 goes to "<< actions_robot_1.size() << " machines:";
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
		outputFile << std::endl << "End of plan"<< std::endl << "__________ __________ __________" << std::endl;
	}
	else logger->log_info(name(), "Test of import .smt file into z3 constraint did NOT work (UNSAT)");
}

void ClipsSmtThread::clips_smt_solve_formula_from_fg_smt_file(std::string path, FormulaGenerator fg) {
	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	z3::solver s(_z3_context);
	s.add(e);

	// Start measuring sovling time
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
	if(s.check() == z3::sat) {
		// Stop measuring sovling time
		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

		// Compute time for solving
		double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
		double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

		logger->log_info(name(), "Test of import .smt file into z3 constraint did work (SAT) [%f ms, %f m]", diff_ms, diff_m);

		z3::model model = s.get_model();
		logger->log_info(name(), "Display model with %i entries", model.size());
		for(unsigned i=0; i<model.size(); ++i) {
			z3::func_decl function = model[i];
			std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;
		}

		logger->log_info(name(), "Create actions with fg and model");
		std::vector<Action> actions = fg.getActions(model);
		logger->log_info(name(), "Print actions");
		for(auto action: actions) {
			logger->log_info(name(), "Action: %s", action.toString().c_str());

			logger->log_info(name(), "Get station_id");
			int station_id = action.getStation()->getId();

			// Extract move from actions
			switch(action.getRobot()->getId()){
				case 0: if(actions_robot_fg_1.empty() || actions_robot_fg_1.back() != station_id) {
							actions_robot_fg_1.push_back(station_id);
						}
						break;
				case 1: if(actions_robot_fg_2.empty() || actions_robot_fg_2.back() != station_id) {
							actions_robot_fg_2.push_back(station_id);
						}
						break;
				case 2: if(actions_robot_fg_3.empty() || actions_robot_fg_3.back() != station_id) {
							actions_robot_fg_3.push_back(station_id);
						}
						break;
				default: break;
			}
		}
	}
	else logger->log_info(name(), "Test of import .smt file into z3 constraint did NOT work (UNSAT)");
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

	clips_smt_solve_formula_from_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/carl_formula.smt");
}

void ClipsSmtThread::clips_smt_test_formulaGenerator()
{
	logger->log_info(name(), "Test FormulaGenerator extern binary");

	GameData gD = FormulaGeneratorTest::createGameDataTestCase();
	FormulaGenerator fg = FormulaGenerator(1, gD);

	logger->log_info(name(), "Export FormulaGenerator formula to file fg_formula.smt");
	std::ofstream outputFile("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt"); // TODO (Igor) Exchange path with config value
	outputFile << carl::outputSMTLIB(carl::Logic::QF_NIRA, {fg.createFormula()});
	outputFile.close();

	logger->log_info(name(), "Import FormulaGenerator formula from file fg_formula.smt into z3 formula");


	clips_smt_solve_formula_from_fg_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt", fg);
}
