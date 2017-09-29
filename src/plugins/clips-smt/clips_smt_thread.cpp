
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
// using Rational = mpq_class;

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
	//clips_smt_test_formulaGenerator();

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

	state2_machines["empty"]=0;
	state2_machines["has_C1"]=1;
	state2_machines["has_C2"]=2;

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
	products_inverted[0]="null";
	products_inverted[1]="B1";
	products_inverted[2]="B2";
	products_inverted[3]="B3";
	products_inverted[4]="B1C1";
	products_inverted[5]="B1C2";
	products_inverted[6]="B2C1";
	products_inverted[7]="B2C2";
	products_inverted[8]="B3C1";
	products_inverted[9]="B3C2";
	products_inverted[10]="BRC1";
	products_inverted[11]="BRC2";

	machine_groups["CS"]=0;
	machine_groups["BS"]=1;
	machine_groups["DS"]=2;

	actions[1] = "|R|CSS|>>BRC|";
	actions[2] = "|P|CS|BRC>>|";
	actions[3] = "|F|CS|>BRC>|";
	actions[4] = "|P|CS|B>C>|";
	actions[5] = "|F|CS|>B+C>";
	actions[6] = "|R|BS|>>B|";
	actions[7] = "|P|BS|B>>|";
	actions[8] = "|R|CS|>C>BR|";
	actions[9] = "|R|CS|>>BC|";
	actions[10] = "|P|DS|BC>>|";
	actions[11] = "|F|DS|>BC>|";

	lower_bounds_c0[0] = 0;
	lower_bounds_c0[1] = 29;
	lower_bounds_c0[2] = 246;
	upper_bounds_c0[0] = lower_bounds_c0[0]+900;
	upper_bounds_c0[1] = lower_bounds_c0[1]+103;
	upper_bounds_c0[2] = lower_bounds_c0[1]+150;
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
	llsf_msgs::ActorSpecificPlan *actor_plan = agplan->add_plans();
	actor_plan->set_actor_name("R-1");
	llsf_msgs::SequentialPlan *plan = actor_plan->mutable_sequential_plan();
	llsf_msgs::PlanAction *action;
	llsf_msgs::PlanActionParameter *param;

	// action = plan->add_actions();
	// action->set_name("enter-field");


	// Francesco's C0 Scenario
	// Loop through model_actions
	for(unsigned int i=0; i<model_actions.size(); ++i){
		switch(model_actions[i]){
			case 1:		// Get black cap with cap carrier from shelf of corresponding C-CS
					// Move action and get from shelf?
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-I");

					action = plan->add_actions();
					action->set_name("retrieve_shelf");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-I");
					param = action->add_params();
					param->set_key("shelf");
					param->set_value("TRUE");

					break;
			case 2:		// Prepare corresponding C-CS to hold black cap with cap carrier
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-I");

					action = plan->add_actions();
					action->set_name("prepare");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-I");
					param = action->add_params();
					param->set_key("operation");
					param->set_value("RETRIEVE_CAP");
					break;
			case 3:		// Put black cap with cap carrier into corresponding C-CS
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-I");

					action = plan->add_actions();
					action->set_name("feed");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-I");
					break;
			case 4:		// Prepare corresponding C-CS to mount red base with black cap
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-I");

					action = plan->add_actions();
					action->set_name("prepare");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-I");
					param = action->add_params();
					param->set_key("operation");
					param->set_value("MOUNT_CAP");
					break;
			case 5:		// Put red base into corresponding C-CS
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-I");

					action = plan->add_actions();
					action->set_name("feed");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-I");
					break;
			case 6:		// Get red base from C-BS-O
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-BS-O");

					action = plan->add_actions();
					action->set_name("retrieve");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-BS-O");
					break;
			case 7:		// Prepare C-BS to output red base
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-BS-O");

					action = plan->add_actions();
					action->set_name("prepare");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-BS-O");
					param = action->add_params();
					param->set_key("color");
					param->set_value("BASE_RED");
					break;
			case 8:		// Discard cap carrier from corresponding C-CS output
					// Move to output side, get cap carrier and discard via opening gripper
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-O");

					action = plan->add_actions();
					action->set_name("retrieve");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-O");

					action = plan->add_actions();
					action->set_name("discard");
					break;
			case 9:		// Get product red_base_black_cap at corresponding C-CS output
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-CS1-O");

					action = plan->add_actions();
					action->set_name("retrieve");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-CS1-O");
					break;
			case 10:	// Prepare C-DS to receive product red_base_black_cap
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-DS-I");

					action = plan->add_actions();
					action->set_name("prepare");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-DS-I");
					param = action->add_params();
					param->set_key("gate");
					param->set_value("1");
					break;
			case 11:	// Put product red_base_black_cap into C-DS-I
					action = plan->add_actions();
					action->set_name("move");
					param = action->add_params();
					param->set_key("to");
					param->set_value("C-DS-I");

					action = plan->add_actions();
					action->set_name("feed");
					param = action->add_params();
					param->set_key("mps");
					param->set_value("C-DS-I");
					break;
			default:	break;
		}
       	}



	// for (const auto &r : std::vector<std::string>{"R-1", "R-2", "R-3"}) {
	// 	llsf_msgs::ActorSpecificPlan *actor_plan = agplan->add_plans();
	// 	actor_plan->set_actor_name(r);
	// 	llsf_msgs::SequentialPlan *plan = actor_plan->mutable_sequential_plan();
	// 	llsf_msgs::PlanAction *action;
	// 	llsf_msgs::PlanActionParameter *param;
	//
	// 	action = plan->add_actions();
	// 	action->set_name("enter-field");
	//
	// 	// Francesco's Move Scenario
	// 	if(r.compare("R-1")==0){
	// 		std::map<int, std::string>::iterator it_actions;
	// 		for(it_actions = actions_robot_1.begin(); it_actions!=actions_robot_1.end(); ++it_actions) {
	// 			action = plan->add_actions();
	// 			action->set_name("move");
	// 			param = action->add_params();
	// 			param->set_key("to");
	// 			param->set_value(it_actions->second);
	// 		}
	// 	}
	// 	else if(r.compare("R-2")==0){
	// 		std::map<int, std::string>::iterator it_actions;
	// 		for(it_actions = actions_robot_2.begin(); it_actions!=actions_robot_2.end(); ++it_actions) {
	// 			action = plan->add_actions();
	// 			action->set_name("move");
	// 			param = action->add_params();
	// 			param->set_key("to");
	// 			param->set_value(it_actions->second);
	// 		}
	// 	}
	// 	else if(r.compare("R-3")==0){
	// 		std::map<int, std::string>::iterator it_actions;
	// 		for(it_actions = actions_robot_3.begin(); it_actions!=actions_robot_3.end(); ++it_actions) {
	// 			action = plan->add_actions();
	// 			action->set_name("move");
	// 			param = action->add_params();
	// 			param->set_key("to");
	// 			param->set_value(it_actions->second);
	// 		}
	// 	}
	//
	// 	// // Leonard
	// 	// if(r.compare("R-1")==0){
	// 	// 	std::map<int, std::string>::iterator it_actions;
	// 	// 	for(int move_to: actions_robot_fg_1) {
	// 	// 		action = plan->add_actions();
	// 	// 		action->set_name("move");
	// 	// 		param = action->add_params();
	// 	// 		param->set_key("to");
	// 	// 		param->set_value(node_names_[move_to]);
	// 	// 	}
	// 	// }
	// 	// else if(r.compare("R-2")==0){
	// 	// 	std::map<int, std::string>::iterator it_actions;
	// 	// 	for(int move_to: actions_robot_fg_2) {
	// 	// 		action = plan->add_actions();
	// 	// 		action->set_name("move");
	// 	// 		param = action->add_params();
	// 	// 		param->set_key("to");
	// 	// 		param->set_value(node_names_[move_to]);
	// 	// 	}
	// 	// }
	// 	// else if(r.compare("R-3")==0){
	// 	// 	std::map<int, std::string>::iterator it_actions;
	// 	// 	for(int move_to: actions_robot_fg_3) {
	// 	// 		action = plan->add_actions();
	// 	// 		action->set_name("move");
	// 	// 		param = action->add_params();
	// 	// 		param->set_key("to");
	// 	// 		param->set_value(node_names_[move_to]);
	// 	// 	}
	// 	// }
	// }


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

	// actions_robot_fg_1.clear();
	// actions_robot_fg_2.clear();
	// actions_robot_fg_3.clear();

	number_robots = 1; // data.robots().size()-1;
	number_machines = 4;// data.machines().size();
	number_orders_protobuf = data.orders().size()/2;
	if(number_orders_protobuf==0){
		logger->log_info(name(), "Protobuf orders is empty, thus use predefined number of orders");
	}

	// number_bits = ceil(log2(number_machines));

	number_orders = number_orders_c0+number_orders_c1; // data.robots().size()/2;
	plan_horizon = number_orders_c0*number_max_required_actions_c0 + number_orders_c1*number_required_actions_c1;
	// if(number_orders_c1) number_actions = number_required_actions_c1;
	// else number_actions = number_required_actions_c0;
	number_actions = number_orders_c0*number_total_action_c0;

	logger->log_info(name(), "Create plan with %i robot(s) (fix), %i machine(s) (fix), %i order(s) (dynamic) with plan_horizon %i and %i actions",
							number_robots,
							number_machines,
							number_orders,
							plan_horizon,
							number_actions);

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
	z3::expr_vector formula = clips_smt_encoder(varStartTime, varRobotDuration, varRobotPosition,
												varMachineDuration, varR, varA,
												varM, varHold, varS,
												varRew, varInit);

	// Pass it to z3 solver
	clips_smt_solve_formula(formula);

	// Test formulaGenerator
	// logger->log_info(name(), "Convert protobuf to gamedata");
	// clips_smt_convert_protobuf_to_gamedata();

	// Test precomputed .smt2 files
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
	// wakeup();
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

	// Set only required names of machines fix
	node_names_[1] = "C-BS-I";
	node_names_[2] = "C-CS1-I";
	node_names_[3] = "C-CS1-O";
	node_names_[4] = "C-DS-I";
	// node_names_[5] = "C-RS1-I";
	// node_names_[6] = "C-RS1-O";

	node_names_inverted["C-ins-in"] = 0;
	node_names_inverted["C-BS-I"] = 1;
	node_names_inverted["C-CS1-I"] = 2;
	node_names_inverted["C-CS1-O"] = 3;
	node_names_inverted["C-DS-I"] = 4;
	// node_names_inverted["C-RS1-I"] = 5;
	// node_names_inverted["C-RS1-O"] = 6;
}

void
ClipsSmtThread::clips_smt_fill_robot_names()
{
	logger->log_info(name(), "Get name of robots using protobuf data");
	robot_names_.clear();

	// // Read names of robots automatically
	// int i_true=0;
	// for(int i=0; i<number_robots+1; ++i){
	// 	std::string robot_name = data.robots(i).name().c_str();
	//
	// 	if(!robot_name.compare("RefBox")==0) {
	// 		// Not hitting 'RefBox'
	// 		// logger->log_info(name(), "Add %s to robot_names_", robot_name.c_str());
	// 		robot_names_[i_true] = robot_name;
	// 		i_true++;
	// 	}
	// }

	// Set names of robots fix
	robot_names_[0] = "R-1";
	robot_names_[1] = "R-2";
	robot_names_[2] = "R-3";
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
	std::ofstream of_distances;
	of_distances.open ("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/navgraph-costs-000.csv");

	// Compute distances between unconnected C-ins-in and all other machines
	NavGraphNode ins_node(navgraph->node("C-ins-in"));
	NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		std::pair<std::string, std::string> nodes_pair(ins_node.name(), node_names_[i]);

		NavGraphNode to = navgraph->node(node_names_[i]);
		// logger->log_info(name(), "Position of machine %s is (%f,%f)", to.name().c_str(), to.x(), to.y());

		NavGraphPath p = navgraph->search_path(from, to);

		// logger->log_info(name(), "Distance between node %s and node %s is %f", from.name().c_str(), node_names_[i].c_str(), p.cost());
		of_distances << "C-ins-in;" << node_names_[i].c_str() <<";" << p.cost()+navgraph->cost(from, ins_node) << "\n";
		distances_[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
	}

	// Compute distances between machines
	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		for (unsigned int j = 1; j < node_names_.size(); ++j) {
			if (i == j) continue;
			std::pair<std::string, std::string> nodes_pair(node_names_[i], node_names_[j]);

			NavGraphPath p = navgraph->search_path(node_names_[i], node_names_[j]);
			// logger->log_info(name(), "Distance between node %s and node %s is %f", node_names_[i].c_str(), node_names_[j].c_str(), p.cost());
			of_distances << node_names_[i].c_str() << ";" << node_names_[j].c_str() <<";" << p.cost() << "\n";
			distances_[nodes_pair] = p.cost();
		}
	}

	of_distances.close();
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
									std::map<std::string, z3::expr>& varInit)
{
	logger->log_info(name(), "Create z3 encoder");

	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);
	// Init variable true and false
	z3::expr var_false(_z3_context.bool_val(false));
	z3::expr var_true(_z3_context.bool_val(true));


	// VARIABLES
	logger->log_info(name(), "Add variables");

	// Variables initDist_i_j
	for(int i = 0; i < number_machines+1; ++i){
		for(int j = i+1; j < number_machines+1; ++j) {
			varInit.insert(std::make_pair("initDist_" + std::to_string(i) + "_" + std::to_string(j), _z3_context.real_const(("initDist_" + std::to_string(i) + "_" + std::to_string(j)).c_str())));
		}
	}

	// Variables initPos and initHold
	for(int i = 1; i < number_robots+1; ++i){
		varInit.insert(std::make_pair("initPos_" + std::to_string(i), _z3_context.int_const(("initPos_" + std::to_string(i)).c_str())));
		varInit.insert(std::make_pair("initHold_" + std::to_string(i), _z3_context.int_const(("initHold_" + std::to_string(i)).c_str())));
	}

	// Variables initState1_i, initState2_i and initState3_i
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		varInit.insert(std::make_pair("initState1_" + std::to_string(i), _z3_context.int_const(("initState1_" + std::to_string(i)).c_str())));
		varInit.insert(std::make_pair("initState2_" + std::to_string(i), _z3_context.int_const(("initState2_" + std::to_string(i)).c_str())));
		varInit.insert(std::make_pair("initState3_" + std::to_string(i), _z3_context.int_const(("initState3_" + std::to_string(i)).c_str())));
	}

	// Variables depending on plan_horizon
	for(int i=1; i<plan_horizon+1; ++i){
		varStartTime.insert(std::make_pair("t_" + std::to_string(i), _z3_context.real_const(("t_" + std::to_string(i)).c_str())));
		varRobotDuration.insert(std::make_pair("rd_" + std::to_string(i), _z3_context.real_const(("rd_" + std::to_string(i)).c_str())));
		varRobotPosition.insert(std::make_pair("pos_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(i)).c_str())));
		varMachineDuration.insert(std::make_pair("md_" + std::to_string(i), _z3_context.real_const(("md_" + std::to_string(i)).c_str())));
		varR.insert(std::make_pair("R_" + std::to_string(i), _z3_context.int_const(("R_" + std::to_string(i)).c_str())));
		varA.insert(std::make_pair("A_" + std::to_string(i), _z3_context.int_const(("A_" + std::to_string(i)).c_str())));
		varM.insert(std::make_pair("M_" + std::to_string(i), _z3_context.int_const(("M_" + std::to_string(i)).c_str())));
		varHold.insert(std::make_pair("holdA_" + std::to_string(i), _z3_context.int_const(("holdA_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state1A_" + std::to_string(i), _z3_context.int_const(("state1A_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state2A_" + std::to_string(i), _z3_context.int_const(("state2A_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state3A_" + std::to_string(i), _z3_context.int_const(("state3A_" + std::to_string(i)).c_str())));
		varHold.insert(std::make_pair("holdB_" + std::to_string(i), _z3_context.int_const(("holdB_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state1B_" + std::to_string(i), _z3_context.int_const(("state1B_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state2B_" + std::to_string(i), _z3_context.int_const(("state2B_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state3B_" + std::to_string(i), _z3_context.int_const(("state3B_" + std::to_string(i)).c_str())));
		varRew.insert(std::make_pair("rew_" + std::to_string(i), _z3_context.real_const(("rew_" + std::to_string(i)).c_str())));
	}


	// CONSTRAINTS
	logger->log_info(name(), "Add constraints");

	// Constraints depending on plan_horizon
	logger->log_info(name(), "Add constraints depending on plan_horizon");

	for(int i = 1; i < plan_horizon+1; ++i){

		// VarStartTime
		if(i==1){
			constraints.push_back(0 <= getVar(varStartTime, "t_"+std::to_string(i)));
		}
		else if(i==plan_horizon) {
			constraints.push_back(getVar(varStartTime, "t_"+std::to_string(i)) <= 900);
		}
		else {
			constraints.push_back(getVar(varStartTime, "t_"+std::to_string(i-1)) <= getVar(varStartTime, "t_"+std::to_string(i)));
		}

		constraints.push_back(0 <= getVar(varRobotDuration, "rd_"+std::to_string(i))); // VarRobotDuration
		constraints.push_back(1 <= getVar(varRobotPosition, "pos_"+std::to_string(i)) && getVar(varRobotPosition, "pos_"+std::to_string(i)) <= number_machines); // VarRobotPosition
		constraints.push_back(0 <= getVar(varMachineDuration, "md_"+std::to_string(i))); // VarMachineDuration
		constraints.push_back(1 <= getVar(varR, "R_"+std::to_string(i)) && getVar(varR, "R_"+std::to_string(i)) <= number_robots); // VarR
		constraints.push_back(1 <= getVar(varA, "A_"+std::to_string(i)) && getVar(varA, "A_"+std::to_string(i)) <= number_actions); // VarA
		constraints.push_back(min_machine_groups <= getVar(varM, "M_"+std::to_string(i)) && getVar(varM, "M_"+std::to_string(i)) <= max_machine_groups); // VarM
		constraints.push_back(min_products <= getVar(varHold, "holdA_"+std::to_string(i)) && getVar(varHold, "holdA_"+std::to_string(i)) <= max_products); // VarHoldA
		constraints.push_back(min_state1_machines <= getVar(varS, "state1A_"+std::to_string(i)) && getVar(varS, "state1A_"+std::to_string(i)) <= max_state1_machines); // VarState1A
		constraints.push_back(min_state2_machines <= getVar(varS, "state2A_"+std::to_string(i)) && getVar(varS, "state2A_"+std::to_string(i)) <= max_state2_machines); // VarState2A
		constraints.push_back(min_state3_machines <= getVar(varS, "state3A_"+std::to_string(i)) && getVar(varS, "state3A_"+std::to_string(i)) <= max_state3_machines); // VarState3A
		constraints.push_back(min_products <= getVar(varHold, "holdB_"+std::to_string(i)) && getVar(varHold, "holdB_"+std::to_string(i)) <= max_products); // VarHoldB
		constraints.push_back(min_state1_machines <= getVar(varS, "state1B_"+std::to_string(i)) && getVar(varS, "state1B_"+std::to_string(i)) <= max_state1_machines); // VarState1B
		constraints.push_back(min_state2_machines <= getVar(varS, "state2B_"+std::to_string(i)) && getVar(varS, "state2B_"+std::to_string(i)) <= max_state2_machines); // VarState2B
		constraints.push_back(min_state3_machines <= getVar(varS, "state3B_"+std::to_string(i)) && getVar(varS, "state3B_"+std::to_string(i)) <= max_state3_machines); // VarState3B

	}

	// Constraint: robot states are initially consistent
	logger->log_info(name(), "Add constraints stating robot states are initially consistent");

	for(int i=1; i<number_robots+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(varR, "R_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(varR, "R_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(var_false);
			for(int k=0; k<number_machines+1; ++k){
				for(int l=1; l<number_machines+1; ++l){
					if(k<l){
						constraint2 = constraint2 || (getVar(varInit, "initPos_"+std::to_string(i))==k
														&& getVar(varRobotPosition, "pos_"+std::to_string(ip))==l
														&& getVar(varStartTime, "t_"+std::to_string(ip))>=
															getVar(varInit, "initDist_"+std::to_string(k)+"_"+std::to_string(l)));
					}
					else if(l<k){
						constraint2 = constraint2 || (getVar(varInit, "initPos_"+std::to_string(i))==k
														&& getVar(varRobotPosition, "pos_"+std::to_string(ip))==l
														&& getVar(varStartTime, "t_"+std::to_string(ip))>=
															getVar(varInit, "initDist_"+std::to_string(l)+"_"+std::to_string(k)));
					}
					else {
						constraint2 = constraint2 || (getVar(varInit, "initPos_"+std::to_string(i))==k
														&& getVar(varRobotPosition, "pos_"+std::to_string(ip))==l
														&& getVar(varStartTime, "t_"+std::to_string(ip))>=0);
					}
				}
			}

			constraints.push_back(constraint1 || (getVar(varHold, "holdA_"+std::to_string(ip))==getVar(varInit, "initHold_"+std::to_string(i)) && constraint2));
		}
	}

	// Constraint: robot states are inductively consistent
	logger->log_info(name(), "Add constraints stating robot states are inductively consistent");

	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(varR, "R_"+std::to_string(ip)) == getVar(varR, "R_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(varR, "R_"+std::to_string(ipp))==getVar(varR, "R_"+std::to_string(i));
			}

			z3::expr constraint2(var_false);
			for(int k=1; k<number_machines+1; ++k){
				for(int l=1; l<number_machines+1; ++l){
					if(k<l){
						constraint2 = constraint2 || (getVar(varRobotPosition, "pos_"+std::to_string(i))==k
														&& getVar(varRobotPosition, "pos_"+std::to_string(ip))==l
														&& getVar(varStartTime, "t_"+std::to_string(ip))>=
															getVar(varStartTime, "t_"+std::to_string(i))+getVar(varRobotDuration, "rd_"+std::to_string(ip))+getVar(varInit, "initDist_"+std::to_string(k)+"_"+std::to_string(l)));
					}
					else if(l<k){
						constraint2 = constraint2 || (getVar(varRobotPosition, "pos_"+std::to_string(i))==k
														&& getVar(varRobotPosition, "pos_"+std::to_string(ip))==l
														&& getVar(varStartTime, "t_"+std::to_string(ip))>=
															getVar(varStartTime, "t_"+std::to_string(i))+getVar(varRobotDuration, "rd_"+std::to_string(ip))+getVar(varInit, "initDist_"+std::to_string(l)+"_"+std::to_string(k)));
					}
					else {
						constraint2 = constraint2 || (getVar(varRobotPosition, "pos_"+std::to_string(i))==k
														&& getVar(varRobotPosition, "pos_"+std::to_string(ip))==l
														&& getVar(varStartTime, "t_"+std::to_string(ip))>=
															getVar(varStartTime, "t_"+std::to_string(i))+getVar(varRobotDuration, "rd_"+std::to_string(ip)));
					}
				}
			}

			constraints.push_back(constraint1 || (getVar(varHold, "holdA_"+std::to_string(ip))==getVar(varHold, "holdB_"+std::to_string(i)) && constraint2));
		}
	}

	// Constraint: machine states are initially consistent
	logger->log_info(name(), "Add constraints stating machine states are initially consistent");

	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(varM, "M_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(varM, "M_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(getVar(varS, "state1A_"+std::to_string(ip))==getVar(varInit, "initState1_"+std::to_string(i))
								&& getVar(varS, "state2A_"+std::to_string(ip))==getVar(varInit, "initState2_"+std::to_string(i))
								&& getVar(varS, "state3A_"+std::to_string(ip))==getVar(varInit, "initState3_"+std::to_string(i)));

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Constraint: machine states are inductively consistent
	logger->log_info(name(), "Add constraints stating robot states are inductively consistent");

	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(varM, "M_"+std::to_string(ip)) == getVar(varM, "M_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || (getVar(varM, "M_"+std::to_string(ipp)) == getVar(varM, "M_"+std::to_string(i)));
			}

			z3::expr constraint2(getVar(varStartTime, "t_"+std::to_string(ip))>=getVar(varStartTime, "t_"+std::to_string(i))+getVar(varMachineDuration, "md_"+std::to_string(i))
									&& getVar(varS, "state1B_"+std::to_string(i))==getVar(varS, "state1A_"+std::to_string(ip))
									&& getVar(varS, "state2B_"+std::to_string(i))==getVar(varS, "state2A_"+std::to_string(ip))
									&& getVar(varS, "state3B_"+std::to_string(i))==getVar(varS, "state3A_"+std::to_string(ip)));

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Action stuff for every order
	logger->log_info(name(), "Add constraints for actions for %i orders", number_orders);

	for(int i=1; i<plan_horizon+1; ++i){

		// // 0.Action: Empty action (Including a one time step back?)
		// z3::expr constraint_action0((getVar(varS, "state1B_"+std::to_string(i)) == getVar(varS, "state1A_"+std::to_string(i)))
		// 							&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
		// 							&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
		// 							&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == 0)
		// 							&& (getVar(varHold, "holdA_"+std::to_string(i)) == getVar(varHold, "holdB_"+std::to_string(i)))
		// 							&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
		// constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 0) || constraint_action0);

		for(int o=0; o<number_orders; ++o){

			std::string bi = "B";
			std::string ci = "C";

			if(data.orders().size()/2>=1){
				// Determine base, ring and cap color via protobuf
				bi += std::to_string(data.orders(o+data.orders().size()/2).base_color());
				ci += std::to_string(data.orders(o+data.orders().size()/2).cap_color());
			}
			else {
				// Fix colors of orders of complexity c0
				switch(o){
					case 0: bi = "B1";
							ci = "C1";
							break;
					case 1: bi = "B3";
							ci = "C2";
							break;
					case 2: bi = "B2";
							ci = "C1";
							break;
				}
			}

			// Save information for Visualization
			orders_base[o] = bi;
			orders_cap[o] = ci;

			// Determine required strings
			std::string bi_ci = bi;
			bi_ci += ci;
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


			// Macroactions to combine pure actions

			// // 1.Macroaction: Prepare CapStation for Base input [1,2,3,8,4]
			// z3::expr constraint_macroaction1((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
			// 							&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines["not_prep"])
			// 							&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines[mount_ci])
			// 							&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
			// 							&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
			// 							&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
			// 							&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
			// 							&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed+time_to_prep+time_to_disc)
			// 							&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
			// 							&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
			// 							&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
			// 							&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			// constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (12 + o*number_total_action_c0)) || constraint_macroaction1);
			//
			// // 2.Macroaction : Get Base from BaseStation [7,6]
			// z3::expr constraint_macroaction2((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
			// 							&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines["not_prep"])
			// 							&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines["not_prep"])
			// 							&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
			// 							&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
			// 							&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
			// 							&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-BS-I"])
			// 							&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
			// 							&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi])
			// 							&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			// constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (13 + o*number_total_action_c0)) || constraint_macroaction2);
			//
			// // 3.Macroaction : Construct product [5,9]
			// z3::expr constraint_macroaction3((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
			// 							&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines[mount_ci])
			// 							&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines["not_prep"])
			// 							&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
			// 							&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
			// 							&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
			// 							&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
			// 							&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed+time_to_fetch)
			// 							&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
			// 							&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi])
			// 							&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_ci])
			// 							&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			// constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (14 + o*number_total_action_c0)) || constraint_macroaction3);
			//
			// // 4.Macroaction : Deliver product [10,11]
			// z3::expr constraint_macroaction4((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
			// 							&& (getVar(varS, "state1B_"+std::to_string(i)) == getVar(varS, "state1A_"+std::to_string(i)))
			// 							&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
			// 							&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
			// 							&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
			// 							&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-DS-I"])
			// 							&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_ci])
			// 							&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
			// 							&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			// constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (15 + o*number_total_action_c0)) || constraint_macroaction4);

			// // 5.Macroaction: Operate at CapStation for Base input [3,8]
			// z3::expr constraint_macroaction5((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
			// 							&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines[retrieve_ci])
			// 							&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines[mount_ci])
			// 							&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
			//							&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
			// 							&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
			// 							&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
			// 							&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed+time_to_disc)
			// 							&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
			// 							&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[br_ci])
			// 							&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
			// 							&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			// constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (16 + o*number_total_action_c0)) || constraint_macroaction5);

			// 1.Action : retrieve base with cap from shelf at CS
			z3::expr constraint_action1((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == getVar(varS, "state1A_"+std::to_string(i)))
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[br_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (1 + o*number_total_action_c0)) || constraint_action1);

			// 2.Action : prepare CS to retrieve cap
			z3::expr constraint_action2((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines[retrieve_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == getVar(varHold, "holdB_"+std::to_string(i)))
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (2 + o*number_total_action_c0)) || constraint_action2);

			// 3.Action : feed base with cap into CS
			z3::expr constraint_action3((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines[retrieve_ci])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[br_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (3 + o*number_total_action_c0)) || constraint_action3);

			// 4.Action : prepare CS to mount cap
			z3::expr constraint_action4((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines[mount_ci])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == getVar(varHold, "holdB_"+std::to_string(i)))
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (4 + o*number_total_action_c0)) || constraint_action4);

			// 5.Action : feed base to CS to mount cap
			z3::expr constraint_action5((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines[mount_ci])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_ci])
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (5 + o*number_total_action_c0)) || constraint_action5);

			// 6.Action : retrieve base from BS
			z3::expr constraint_action6((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines[prep_bi])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-BS-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (6 + o*number_total_action_c0)) || constraint_action6);

			// 7.Action : prepare BS to provide base
			z3::expr constraint_action7((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines[prep_bi])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-BS-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == getVar(varHold, "holdB_"+std::to_string(i)))
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (7 + o*number_total_action_c0)) || constraint_action7);

			// 8.Action : discard capless base from CS
			z3::expr constraint_action8((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == getVar(varS, "state1B_"+std::to_string(i)))
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-O"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (8 + o*number_total_action_c0)) || constraint_action8);

			// 9.Action : retrieve base with cap from CS
			z3::expr constraint_action9((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == getVar(varS, "state1B_"+std::to_string(i)))
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_ci])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-CS1-O"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (9 + o*number_total_action_c0)) || constraint_action9);

			// 10.Action : prepare DS for slide specified order
			z3::expr constraint_action10((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines[slide_one_prep_bi_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-DS-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (10 + o*number_total_action_c0)) || constraint_action10);

			// 11.Action : deliver to DS
			z3::expr constraint_action11((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(varS, "state1A_"+std::to_string(i)) == state1_machines[slide_one_prep_bi_ci])
										&& (getVar(varS, "state1B_"+std::to_string(i)) == state1_machines["not_prep"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted["C-DS-I"])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == (11 + o*number_total_action_c0)) || constraint_action11);
		}
	}

	// logger->log_info(name(), "Add constraints for goal state");
	//
	// // Specify goal state
	// for(int i=1; i<plan_horizon+1; ++i){
	// 	if(i==1){
	// 		constraints.push_back(((getVar(varA, "A_"+std::to_string(i)) == number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 2*number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 3*number_final_action_c0)
	// 									&& getVar(varRew, "rew_"+std::to_string(i)) == (deadline-getVar(varStartTime, "t_"+std::to_string(i))-getVar(varMachineDuration, "md_"+std::to_string(i))))
	// 								|| (!(getVar(varA, "A_"+std::to_string(i)) == number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 2*number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 3*number_final_action_c0)
	// 									&& getVar(varRew, "rew_"+std::to_string(i))==0)); // TODO adapt to "last" action
	// 	}
	// 	else {
	// 		constraints.push_back(((getVar(varA, "A_"+std::to_string(i)) == number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 2*number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 3*number_final_action_c0)
	// 									&& getVar(varRew, "rew_"+std::to_string(i)) == (getVar(varRew, "rew_"+std::to_string(i-1))+deadline-getVar(varStartTime, "t_"+std::to_string(i))-getVar(varMachineDuration, "md_"+std::to_string(i))))
	// 								|| (!(getVar(varA, "A_"+std::to_string(i)) == number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 2*number_final_action_c0
	// 										|| getVar(varA, "A_"+std::to_string(i)) == 3*number_final_action_c0)
	// 									&& getVar(varRew, "rew_"+std::to_string(i))==getVar(varRew, "rew_"+std::to_string(i-1))));
	// 	}
	// }

	logger->log_info(name(), "Add constraints for initial situation");

	// Specify initial situation for robots
	for(int i=1; i<number_robots+1; ++i){
		constraints.push_back(getVar(varInit, "initHold_"+std::to_string(i))==products["nothing"]);
		constraints.push_back(getVar(varInit, "initPos_"+std::to_string(i))==0);
	}

	// Specify initial situation for machines
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		constraints.push_back(getVar(varInit, "initState1_"+std::to_string(i))==state1_machines["not_prep"]);
		if(i==3) {
			constraints.push_back(getVar(varInit, "initState2_"+std::to_string(i))==state2_machines["has_R1"]); // TODO Adapt to the dynamic case
		}
		else {
			constraints.push_back(getVar(varInit, "initState2_"+std::to_string(i))==state2_machines["empty"]);
		}
		constraints.push_back(getVar(varInit, "initState3_"+std::to_string(i))==state3_machines["empty"]);
	}

	logger->log_info(name(), "Add constraints for distances between machines");

	// Specify distances between machines
	for(int k=0; k<number_machines+1; ++k){
		for(int l=k+1; l<number_machines+1; ++l){
			float distance = distances_[std::make_pair(node_names_[k], node_names_[l])];
			z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
			constraints.push_back(getVar(varInit, "initDist_"+std::to_string(k)+"_"+std::to_string(l)) == distance_z3);
		}
	}

	// logger->log_info(name(), "Add constraints to fix execution of actions"); // TODO Use to test specific sequence of actions

	// Constraints to fix a sequence of actions
	// constraints.push_back(getVar(varA, "A_1") == 1);
	// constraints.push_back(getVar(varA, "A_2") == 7);
	// constraints.push_back(getVar(varA, "A_3") == 2);
	// constraints.push_back(getVar(varA, "A_4") == 3);
	// constraints.push_back(getVar(varA, "A_5") == 8);
	// constraints.push_back(getVar(varA, "A_6") == 6);
	// constraints.push_back(getVar(varA, "A_7") == 4);
	// constraints.push_back(getVar(varA, "A_8") == 5);
	// constraints.push_back(getVar(varA, "A_9") == 9);
	// constraints.push_back(getVar(varA, "A_10") == 10);
	// constraints.push_back(getVar(varA, "A_11") == 11);

	// logger->log_info(name(), "Add constraints for unique actions and null sequences");
	//
	// // Constraint that no action is chosen twice and if 0 is chosen, then all fellow actions are 0 too
	// for(int i=1; i<plan_horizon+1; ++i){
	// 	z3::expr constraint_condition(getVar(varA, "A_"+std::to_string(i))==0);
	// 	z3::expr constraint_effect_unique(var_true);
	// 	z3::expr constraint_effect_null(var_true);
	//
	// 	for(int j=i+1; j<plan_horizon+1; ++j){
	// 		constraint_effect_unique = constraint_effect_unique && !(getVar(varA, "A_"+std::to_string(i)) == getVar(varA, "A_"+std::to_string(j)));
	// 		constraint_effect_null = constraint_effect_null && getVar(varA, "A_"+std::to_string(j)) == 0;
	// 	}
	//
	// 	constraints.push_back(constraint_condition || constraint_effect_unique);
	// 	constraints.push_back(!constraint_condition || constraint_effect_null);
	// }

	// logger->log_info(name(), "Add constraints to exclude pure and macro actions");
	//
	// // Constraint that no action is chosen twice and if 0 is chosen, then all fellow actions are 0 too
	// for(int i=1; i<plan_horizon+1; ++i){
	// 	for(int o=0; o<number_orders_c0; ++o){
	//
	// 		z3::expr constraint_condition_1(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+1);
	// 		z3::expr constraint_condition_2(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+2);
	// 		z3::expr constraint_condition_3(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+3);
	// 		z3::expr constraint_condition_4(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+4);
	// 		z3::expr constraint_condition_5(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+5);
	// 		z3::expr constraint_condition_6(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+6);
	// 		z3::expr constraint_condition_7(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+7);
	// 		z3::expr constraint_condition_8(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+8);
	// 		z3::expr constraint_condition_9(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+9);
	// 		z3::expr constraint_condition_10(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+10);
	// 		z3::expr constraint_condition_11(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+11);
	// 		z3::expr constraint_condition_12(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+12);
	// 		z3::expr constraint_condition_13(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+13);
	// 		z3::expr constraint_condition_14(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+14);
	// 		z3::expr constraint_condition_15(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+15);
	// 		// z3::expr constraint_condition_16(getVar(varA, "A_"+std::to_string(i))==o*number_total_action_c0+16);
	//
	// 		z3::expr constraint_effect_macro_12(var_true);
	// 		z3::expr constraint_effect_macro_13(var_true);
	// 		z3::expr constraint_effect_macro_14(var_true);
	// 		z3::expr constraint_effect_macro_15(var_true);
	// 		// z3::expr constraint_effect_macro_16(var_true);
	// 		z3::expr constraint_effect_pure_12(var_true);
	// 		z3::expr constraint_effect_pure_13(var_true);
	// 		z3::expr constraint_effect_pure_14(var_true);
	// 		z3::expr constraint_effect_pure_15(var_true);
	// 		// z3::expr constraint_effect_pure_16(var_true);
	//
	//
	// 		for(int j=i+1; j<plan_horizon+1; ++j){
	// 			constraint_effect_macro_12 = constraint_effect_macro_12 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+12);
	// 			constraint_effect_macro_13 = constraint_effect_macro_13 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+13);
	// 			constraint_effect_macro_14 = constraint_effect_macro_14 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+14);
	// 			constraint_effect_macro_15 = constraint_effect_macro_15 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+15);
	// 			// constraint_effect_macro_16 = constraint_effect_macro_16 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+16);
	//
	// 			constraint_effect_pure_12 = constraint_effect_pure_12 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+1)
	// 																	&& !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+2)
	// 																	&& !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+3)
	// 																	&& !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+8)
	// 																	&& !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+4);
	// 			constraint_effect_pure_13 = constraint_effect_pure_13 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+7) && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+6);
	// 			constraint_effect_pure_14 = constraint_effect_pure_14 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+5) && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+9);
	// 			constraint_effect_pure_15 = constraint_effect_pure_15 && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+10) && !(getVar(varA, "A_"+std::to_string(j)) == o*number_total_action_c0+11);
	// 			// constraint_effect_pure_16 = constraint_effect_pure_16 && !(var_a_j == o*number_total_action_c0+3)
	// 			// 														&& !(var_a_j == o*number_total_action_c0+8);
	// 		}
	//
	// 		constraints.push_back(!constraint_condition_1 || constraint_effect_macro_12);
	// 		constraints.push_back(!constraint_condition_2 || constraint_effect_macro_12);
	// 		constraints.push_back(!constraint_condition_3 || constraint_effect_macro_12); // && constraint_effect_macro_16));
	// 		constraints.push_back(!constraint_condition_4 || constraint_effect_macro_12);
	// 		constraints.push_back(!constraint_condition_5 || constraint_effect_macro_14);
	// 		constraints.push_back(!constraint_condition_6 || constraint_effect_macro_13);
	// 		constraints.push_back(!constraint_condition_7 || constraint_effect_macro_13);
	// 		constraints.push_back(!constraint_condition_8 || constraint_effect_macro_12); // && constraint_effect_macro_16));
	// 		constraints.push_back(!constraint_condition_9 || constraint_effect_macro_14);
	// 		constraints.push_back(!constraint_condition_10 || constraint_effect_macro_15);
	// 		constraints.push_back(!constraint_condition_11 || constraint_effect_macro_15);
	// 		constraints.push_back(!constraint_condition_12 || constraint_effect_pure_12);
	// 		constraints.push_back(!constraint_condition_13 || constraint_effect_pure_13);
	// 		constraints.push_back(!constraint_condition_14 || constraint_effect_pure_14);
	// 		constraints.push_back(!constraint_condition_15 || constraint_effect_pure_15);
	// 		// constraints.push_back(!constraint_condition_16 || constraint_effect_pure_16);
	//
	// 	}
	// }

	logger->log_info(name(), "Add constraints for final actions");

	// Constraints encoding that final_actions for each order have to be at least executed once
	// If they are chosen, take time bounds into account
	z3::expr constraint_goal(var_true);
	for(int o=0; o<number_orders_c0; ++o){
		z3::expr constraint_subgoal(var_false);
		for(int i=number_min_required_actions_c0; i<plan_horizon+1; ++i){ // Start from here to assign the last action because number_final_action_c0=11 is the minimum number of actions

			z3::expr constraint_finalaction(getVar(varA, "A_"+std::to_string(i)) == o*number_total_action_c0+number_final_action_c0); // || var_a_i == o*number_total_action_c0+number_final_macroaction_c0);

			if(add_temporal_constraint){
				constraints.push_back(!constraint_finalaction || (getVar(varStartTime, "t_"+std::to_string(i)) < upper_bounds_c0[o]-upper_bound_offset
																		&& getVar(varStartTime, "t_"+std::to_string(i)) >lower_bounds_c0[o]));
			}

			constraint_subgoal = constraint_subgoal || constraint_finalaction;
		}

		constraint_goal = constraint_goal && constraint_subgoal;
	}
	constraints.push_back(constraint_goal);

	return constraints;
}

// GameData
// ClipsSmtThread::clips_smt_convert_protobuf_to_gamedata()
// {
// 	GameData gD = GameData();
// 	gamedata_robots.clear();
// 	gamedata_basestations.clear();
// 	gamedata_ringstations.clear();
// 	gamedata_capstations.clear();
// 	gamedata_deliverystations.clear();
//
// 	// Machines
// 	for (int i = 1; i < number_machines+1; i++){
// 		std::string name_machine = node_names_[i];
// 		if(name_machine[2] == 'B') { // BaseStation
// 			auto bs_temp = std::make_shared<BaseStation>(i);
// 			bs_temp->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER}); // TODO Is this constant behavior
// 			bs_temp->setDispenseBaseTime(1);
// 			gamedata_basestations.push_back(bs_temp);
// 			gD.addMachine(bs_temp);
// 		}
// 		else if(name_machine[2] == 'R') { // RingStation
// 			auto rs_temp = std::make_shared<RingStation>(i);
//
// 			/**
// 			for(int j=0; j<data.machines(i-1).ring_colors().size(); ++j) {
// 				rs_temp->addPossibleRingColor(static_cast<Workpiece::Color>(data.machines(i-1).ring_colors(j)+1), 1); // TODO Add all possibleRingColors with number of bases required for production ADD NUMBERS
// 				// rs_temp->setRingColorSetup(static_cast<Workpiece::Color>(data.machines(i-1).ring_colors(j)+1)); // TODO Add ringColorSetup BACK
// 			}
// 			**/
//
// 			if(name_machine[4] == '1'){
// 				rs_temp->addPossibleRingColor(Workpiece::ORANGE, 1);
// 				rs_temp->addPossibleRingColor(Workpiece::BLUE, 1);
// 			}
// 			else if(name_machine[4] == '2'){
// 				rs_temp->addPossibleRingColor(Workpiece::GREEN, 1);
// 				rs_temp->addPossibleRingColor(Workpiece::YELLOW, 1);
// 			}
//
// 			rs_temp->setFeedBaseTime(1);
// 			rs_temp->setMountRingTime(10);
// 			rs_temp->setAdditinalBasesFed(data.machines(i-1).loaded_with());
// 			gamedata_ringstations.push_back(rs_temp);
// 			gD.addMachine(rs_temp);
// 		}
// 		else if(name_machine[2] == 'C') { // CapStation
// 			auto cs_temp = std::make_shared<CapStation>(i);
// 			if(name_machine[4] == '1'){
// 				cs_temp->addPossibleCapColor(Workpiece::GREY);
// 				cs_temp->setFedCapColor(Workpiece::GREY); // TODO Add fedCapColor BACK ONLY BUFFER
// 			}
// 			else if(name_machine[4] == '2'){
// 				cs_temp->addPossibleCapColor(Workpiece::BLACK);
// 				cs_temp->setFedCapColor(Workpiece::BLACK); // TODO Add fedCapColor BACK ONLY BUFFER
// 			}
// 			cs_temp->setFeedCapTime(1);
// 			cs_temp->setMountCapTime(5);
// 			gamedata_capstations.push_back(cs_temp);
// 			gD.addMachine(cs_temp);
// 		}
// 		else if(name_machine[2] == 'D') { // DeliveryStation
// 			auto ds_temp = std::make_shared<DeliveryStation>(i);
// 			gamedata_deliverystations.push_back(ds_temp);
// 			gD.addMachine(ds_temp);
// 		}
// 	}
//
// 	// Robots
// 	for (int i = 0; i < number_robots; i++)
// 	{
// 		std::string name_robot = robot_names_[i];
// 		auto r_temp = std::make_shared<Robot>(i);
// 		Workpiece pr0 = Workpiece(Workpiece::BLACK,{}, Workpiece::NONE);
// 		r_temp->setWorkpiece(pr0);
// 		// TODO Add Workpiece corresponding to robot
// 		gamedata_robots.push_back(r_temp);
// 		gD.addMachine(r_temp);
// 	}
//
// 	// TODO Test if correct machines are mapped to each other
// 	// Note that index m will go over all types of stations
// 	// Robot machine distance
//
// 	int limit_basestation = gamedata_basestations.size();
// 	int limit_ringstation = limit_basestation + gamedata_ringstations.size();
// 	int limit_capstation = limit_ringstation + gamedata_capstations.size();
// 	// int limit_deliverystation = limit_capstation + gamedata_deliverystations.size();
//
// 	for(int r=0; r<number_robots; ++r) {
// 		// logger->log_info(name(), "Watch robot %i", r);
// 		for(int m=0; m<number_machines; ++m) {
// 			// logger->log_info(name(), "Watch machine %i named %s", m, node_names_[m+1].c_str());
// 			float distance = distances_[std::make_pair(robot_names_[r], node_names_[m+1])];
//
// 			// logger->log_info(name(), "Distance computed %f", distance);
//
// 			if(m<limit_basestation) {
// 				// Inside gamedata_basestations
// 				// logger->log_info(name(), "Inside gamedata_basestations");
//
// 				Machine::addMovingTime(*gamedata_robots[r], *gamedata_basestations[m], distance);
// 			}
// 			else if(m<limit_ringstation) {
// 				// Inside gamedata_ringstations
// 				int m_temp = m-gamedata_basestations.size();
// 				// logger->log_info(name(), "Inside gamedata_ringstations with m_temp %i", m_temp);
// 				Machine::addMovingTime(*gamedata_robots[r], *gamedata_ringstations[m_temp], distance);
// 			}
// 			else if(m<limit_capstation) {
// 				// Inside gamedata_capstations
// 				int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
// 				// logger->log_info(name(), "Inside gamedata_capstations with m_temp %i", m_temp);
// 				Machine::addMovingTime(*gamedata_robots[r], *gamedata_capstations[m_temp], distance);
// 			}
// 			else {
// 				// Inside gamedata_deliverystations
// 				int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
// 				// logger->log_info(name(), "Inside gamedata_deliverystations with m_temp %i", m_temp);
// 				Machine::addMovingTime(*gamedata_robots[r], *gamedata_deliverystations[m_temp], distance);
// 			}
//
// 			// logger->log_info(name(), "Distance added");
// 		}
// 	}
//
// 	// Machine machine distance
// 	for(int n=0; n<number_machines; ++n) {
// 		for(int m=n+1; m<number_machines; ++m) {
// 			float distance = distances_[std::make_pair(node_names_[n+1], node_names_[m+1])];
// 			if(n<limit_basestation) {
// 				// n is inside gamedata_basestations
// 				if(m<limit_basestation) {
// 					// m is inside gamedata_basestations
// 					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_basestations[m], distance);
// 				}
// 				else if(m<limit_ringstation) {
// 					// m is inside gamedata_ringstations
// 					int m_temp = m-gamedata_basestations.size();
// 					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_ringstations[m_temp], distance);
// 				}
// 				else if(m<limit_capstation) {
// 					// m is inside gamedata_capstations
// 					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
// 					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_capstations[m_temp], distance);
// 				}
// 				else {
// 					// m is inside gamedata_deliverystations
// 					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
// 					Machine::addMovingTime(*gamedata_basestations[n], *gamedata_deliverystations[m_temp], distance);
// 				}
// 			}
// 			else if(n<limit_ringstation) {
// 				// n is inside gamedata_ringstations
// 				int n_temp = n-gamedata_basestations.size();
// 				if(m<limit_ringstation) {
// 					// m is inside gamedata_ringstations
// 					int m_temp = m-gamedata_basestations.size();
// 					Machine::addMovingTime(*gamedata_ringstations[n_temp], *gamedata_ringstations[m_temp], distance);
// 				}
// 				else if(m<limit_capstation) {
// 					// m is inside gamedata_capstations
// 					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
// 					Machine::addMovingTime(*gamedata_ringstations[n_temp], *gamedata_capstations[m_temp], distance);
// 				}
// 				else {
// 					// m is inside gamedata_deliverystations
// 					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
// 					Machine::addMovingTime(*gamedata_ringstations[n_temp], *gamedata_deliverystations[m_temp], distance);
// 				}
// 			}
// 			else if(n<limit_capstation) {
// 				// n is inside gamedata_capstations
// 				int n_temp = n-gamedata_basestations.size()-gamedata_ringstations.size();
// 				if(m<limit_capstation) {
// 					// m is inside gamedata_capstations
// 					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size();
// 					Machine::addMovingTime(*gamedata_capstations[n_temp], *gamedata_capstations[m_temp], distance);
// 				}
// 				else {
// 					// m is inside gamedata_deliverystations
// 					int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
// 					Machine::addMovingTime(*gamedata_capstations[n_temp], *gamedata_deliverystations[m_temp], distance);
// 				}
// 			}
// 			else {
// 				// n and m are inside gamedata_deliverystations
// 				int n_temp = n-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
// 				int m_temp = m-gamedata_basestations.size()-gamedata_ringstations.size()-gamedata_capstations.size();
// 				Machine::addMovingTime(*gamedata_deliverystations[n_temp], *gamedata_deliverystations[m_temp], distance);
// 			}
// 		}
// 	}
//
// 	// Orders
// 	for (int i = 0; i < data.orders().size()/2; i++)
// 	{
// 		//Color conversions
// 		Workpiece::Color bc_temp = Workpiece::NONE;
// 		switch(data.orders(i+data.orders().size()/2).base_color()){
// 			case 1: bc_temp = static_cast<Workpiece::Color>(Workpiece::RED);
// 					break;
// 			case 2: bc_temp = static_cast<Workpiece::Color>(Workpiece::BLACK);
// 					break;
// 			case 3: bc_temp = static_cast<Workpiece::Color>(Workpiece::SILVER);
// 					break;
// 			default: break;
// 		}
//
// 		std::vector<Workpiece::Color> rc_temps;
// 		for (int j = 0; j < data.orders(i+data.orders().size()/2).ring_colors().size(); j++){
// 			switch(data.orders(i+data.orders().size()/2).ring_colors(j)){
// 				case 1: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::BLUE));
// 						break;
// 				case 2: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::GREEN));
// 						break;
// 				case 3: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::ORANGE));
// 						break;
// 				case 4: rc_temps.push_back(static_cast<Workpiece::Color>(Workpiece::YELLOW));
// 						break;
// 				default: break;
// 			}
//
// 		}
//
// 		Workpiece::Color cc_temp = Workpiece::NONE;
// 		switch(data.orders(i+data.orders().size()/2).cap_color()){
// 			case 1: cc_temp = static_cast<Workpiece::Color>(Workpiece::BLACK);
// 					break;
// 			case 2: cc_temp = static_cast<Workpiece::Color>(Workpiece::GREY);
// 					break;
// 			default: break;
// 		}
//
// 		Workpiece p_temp = Workpiece(bc_temp, rc_temps, cc_temp);
// 		auto o_temp = std::make_shared<Order>(i, p_temp, data.orders(i).delivery_period_end()*100); // TODO Fix third parameter
// 		gD.addOrder(o_temp);
// 	}
//
//
// 	logger->log_info(name(), "Create fg with gD");
// 	FormulaGenerator fg = FormulaGenerator(1, gD);
// 	// logger->log_info(name(), "Display fg.createFormula()");
// 	// cout << fg.createFormula() << std::endl << std::endl;
// 	logger->log_info(name(), "Display gD.toString()");
// 	cout << gD.toString() << std::endl;
//
// 	logger->log_info(name(), "Export GameData formula to file gD_fg_formula.smt");
// 	std::ofstream of_fg_formula("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/gD_fg_formula.smt"); // TODO (Igor) Exchange path with config value
// 	of_fg_formula << carl::outputSMTLIB(carl::Logic::QF_NIRA, {fg.createFormula()});
// 	of_fg_formula.close();
//
// 	clips_smt_solve_formula_from_fg_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/gD_fg_formula.smt", fg);
//
//
// 	logger->log_info(name(), "Finish extracting");
//
// 	return gD;
// }

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

	z3::solver z3Optimizer(_z3_context);
	// z3::optimize z3Optimizer(_z3_context);

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

	z3::expr rew_planhorizon = _z3_context.real_const(("rew_"+std::to_string(plan_horizon)).c_str());

	// z3Optimizer.maximize(rew_planhorizon); // TODO

	// Export stats into clips_smt_thread_stats.txt
	std::ofstream of_stats;
	of_stats.open("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats.txt", std::ofstream::out | std::ofstream::app);

	// Add time stamp to stats
	time_t now = time(0);
	char* dt = ctime(&now);
	of_stats << std::endl << dt << std::endl;

	// Begin measuring solving time
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	z3::set_param("pp.decimal", true);

	// Export formula into .smt file
	std::ofstream of_c0_formula("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/encoder_c0_formula.smt"); // TODO (Igor) Exchange path with config value
	// of_c0_formula << Z3_optimize_to_string(_z3_context, z3Optimizer);
	of_c0_formula << z3Optimizer.to_smt2() << std::endl;
	of_c0_formula.close();

	if (z3Optimizer.check() == z3::sat){
		// End measuring solving time in case of sat
		end = std::chrono::high_resolution_clock::now();

		// Add sat to stats
		of_stats << "SAT" << std::endl;

		logger->log_info(name(), "Finished solving and optimizing formula with SAT");

		z3::model model = z3Optimizer.get_model();

		for(unsigned i=0; i<model.size(); ++i) {
			z3::func_decl function = model[i];
			// std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;

			std::string function_name = function.name().str();
			z3::expr expr = model.get_const_interp(function);
			float interp;
			std::string s_interp;
			s_interp = Z3_get_numeral_decimal_string(_z3_context, expr, 6);
			interp = std::stof(s_interp);

			for(int j=1; j<plan_horizon+1; ++j){
				if(interp>0) {
					std::string cw_time = "t_";
					cw_time += std::to_string(j);
					std::string cw_pos = "pos_";
					cw_pos += std::to_string(j);
					std::string cw_robot = "R_";
					cw_robot += std::to_string(j);
					std::string cw_action = "A_";
					cw_action += std::to_string(j);
					std::string cw_holdA = "holdA_";
					cw_holdA += std::to_string(j);
					std::string cw_state1A = "state1A_";
					cw_state1A += std::to_string(j);
					std::string cw_state2A = "state2A_";
					cw_state2A += std::to_string(j);
					std::string cw_state3A = "state3A_";
					cw_state3A += std::to_string(j);
					std::string cw_holdB = "holdB_";
					cw_holdB += std::to_string(j);
					std::string cw_state1B = "state1B_";
					cw_state1B += std::to_string(j);
					std::string cw_state2B = "state2B_";
					cw_state2B += std::to_string(j);
					std::string cw_state3B = "state3B_";
					cw_state3B += std::to_string(j);

					if(function_name.compare(cw_time)==0) {
						model_times[j] = interp;
					}
					else if(function_name.compare(cw_pos)==0) {
						model_positions[j] = (int) interp;
					}
					else if(function_name.compare(cw_robot)==0) {
						model_robots[j] = (int) interp;
					}
					else if(function_name.compare(cw_action)==0) {
						model_actions[j] = (int) interp;
					}
					else if(function_name.compare(cw_holdA)==0) {
						model_holdA[j] = (int) interp;
					}
					else if(function_name.compare(cw_state1A)==0) {
						model_state1A[j] = (int) interp;
					}
					else if(function_name.compare(cw_state2A)==0) {
						model_state2A[j] = (int) interp;
					}
					else if(function_name.compare(cw_state3A)==0) {
						model_state3A[j] = (int) interp;
					}
					else if(function_name.compare(cw_holdB)==0) {
						model_holdB[j] = (int) interp;
					}
					else if(function_name.compare(cw_state1B)==0) {
						model_state1B[j] = (int) interp;
					}
					else if(function_name.compare(cw_state2B)==0) {
						model_state2B[j] = (int) interp;
					}
					else if(function_name.compare(cw_state3B)==0) {
						model_state3B[j] = (int) interp;
					}
				}
			}
		}

		// Add plan specified by the model to stats
		// of_stats << "number_orders_c0: " << number_orders_c0 << std::endl;
		// of_stats << "add_temporal_constraint: " << add_temporal_constraint << std::endl;
		for(int o=0; o<number_orders; ++o){
			of_stats << "O" << o+1 << ": " << orders_base[o] << orders_cap[o];
			if(add_temporal_constraint){
				of_stats << " with bounds " << lower_bounds_c0[o] << "s < o0 < " << upper_bounds_c0[o] << "s-" << upper_bound_offset << "s";
			}
			of_stats << std::endl;
		}

		of_stats << std::endl;

		for(int j=1; j<plan_horizon+1; ++j){
			of_stats << j <<". ";
			of_stats << "R" << model_robots[j] << " for O" << ((model_actions[j]-1)/number_max_required_actions_c0)+1;
			of_stats << " does " << actions[model_actions[j]]; //A" << model_actions[j];
			of_stats  << " and holds " << products_inverted[model_holdB[j]] << " at "<< node_names_[model_positions[j]];
			// ":[H(" << model_holdA[j] << "-" << model_holdB[j] <<
			// "), S1(" << model_state1A[j] << "-" << model_state1B[j] <<
			// "), S2(" << model_state2A[j] << "-" << model_state2B[j] <<
			// "), S3(" << model_state3A[j] << "-" << model_state3B[j] <<"]";
			of_stats << " [t = " << model_times[j] << "s]" << std::endl;
		}
	} else {

		// End measuring solving time in case of unsat
		end = std::chrono::high_resolution_clock::now();

		// Add unsat to stats
		of_stats << "UNSAT" << std::endl;

		logger->log_info(name(), "Finished solving and optimizing formula with UNSAT");
	}

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	// logger->log_info(name(), "Time difference is %f ms", diff); // Measure time in nanoseconds but display in milliseconds for convenience

	of_stats << "Time used for solving: " << diff_ms << " ms, " << diff_m << " m" << std::endl << "__________ __________ __________";
	of_stats.close();
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
		std::ofstream of_stats;
		of_stats.open("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats.txt", std::ofstream::out | std::ofstream::app);

		time_t now = time(0);
		char* dt = ctime(&now);
		of_stats << std::endl << dt << std::endl;

		// Add plan specified by the model to stats
		of_stats << "Begin of plan" << std::endl << "R-1 goes to "<< actions_robot_1.size() << " machines:";
		std::map<int, std::string>::iterator it_actions_1;
		for(it_actions_1 = actions_robot_1.begin(); it_actions_1!=actions_robot_1.end(); ++it_actions_1) {
			of_stats << " [" << it_actions_1->first << "] " << it_actions_1->second;
		}
		of_stats << std::endl << "R-2 goes to "<< actions_robot_2.size() << " machines:";
		std::map<int, std::string>::iterator it_actions_2;
		for(it_actions_2 = actions_robot_2.begin(); it_actions_2!=actions_robot_2.end(); ++it_actions_2) {
			of_stats << " [" << it_actions_2->first << "] " << it_actions_2->second;
		}
		of_stats << std::endl << "R-3 goes to "<< actions_robot_3.size() << " machines:";
		std::map<int, std::string>::iterator it_actions_3;
		for(it_actions_3 = actions_robot_3.begin(); it_actions_3!=actions_robot_3.end(); ++it_actions_3) {
			of_stats << " [" << it_actions_3->first << "] " << it_actions_3->second;
		}
		of_stats << std::endl << "End of plan"<< std::endl << "__________ __________ __________" << std::endl;
	}
	else logger->log_info(name(), "Test of import .smt file into z3 constraint did NOT work (UNSAT)");
}

// void ClipsSmtThread::clips_smt_solve_formula_from_fg_smt_file(std::string path, FormulaGenerator fg) {
// 	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
// 	z3::expr e(_z3_context, a);
//
// 	z3::solver s(_z3_context);
// 	s.add(e);
//
// 	// Start measuring sovling time
// 	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
// 	if(s.check() == z3::sat) {
// 		// Stop measuring sovling time
// 		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//
// 		// Compute time for solving
// 		double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
// 		double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;
//
// 		logger->log_info(name(), "Test of import .smt file into z3 constraint did work (SAT) [%f ms, %f m]", diff_ms, diff_m);
//
// 		z3::model model = s.get_model();
// 		logger->log_info(name(), "Display model with %i entries", model.size());
// 		for(unsigned i=0; i<model.size(); ++i) {
// 			z3::func_decl function = model[i];
// 			std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;
// 		}
//
// 		logger->log_info(name(), "Create actions with fg and model");
// 		std::vector<Action> actions = fg.getActions(model);
// 		logger->log_info(name(), "Print actions");
// 		for(auto action: actions) {
// 			logger->log_info(name(), "Action: %s", action.toString().c_str());
//
// 			logger->log_info(name(), "Get station_id");
// 			int station_id = action.getStation()->getId();
//
// 			// Extract move from actions
// 			switch(action.getRobot()->getId()){
// 				case 0: if(actions_robot_fg_1.empty() || actions_robot_fg_1.back() != station_id) {
// 							actions_robot_fg_1.push_back(station_id);
// 						}
// 						break;
// 				case 1: if(actions_robot_fg_2.empty() || actions_robot_fg_2.back() != station_id) {
// 							actions_robot_fg_2.push_back(station_id);
// 						}
// 						break;
// 				case 2: if(actions_robot_fg_3.empty() || actions_robot_fg_3.back() != station_id) {
// 							actions_robot_fg_3.push_back(station_id);
// 						}
// 						break;
// 				default: break;
// 			}
// 		}
// 	}
// 	else logger->log_info(name(), "Test of import .smt file into z3 constraint did NOT work (UNSAT)");
// }

/**
 * Test methods
 * - z3: Export carl formula into .smt file and import .smt file back into z3 formula in order to call the z3 solver
 * - carl: Call a carl method and compare computed to known output
 * - formulaGenerator: Similar to z3 test method, but with test case of formulaGenerator
 **/

// void
// ClipsSmtThread::clips_smt_test_z3()
// {
//
// 	logger->log_info(name(), "Test z3 extern binary");
//
// 	logger->log_info(name(), "Setup carl test formula");
// 	carl::Variable x = carl::freshRealVariable("x");
// 	Rational r = 4;
// 	carl::MultivariatePolynomial<Rational> mp = Rational(r*r)*x*x + r*x + r;
// 	carl::Formula<carl::MultivariatePolynomial<Rational>> f(mp, carl::Relation::GEQ);
//
// 	logger->log_info(name(), "Export carl test formula to file carl_formula.smt");
// 	std::ofstream of_carl_formula("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/carl_formula.smt"); // TODO (Igor) Exchange path with config value
// 	of_carl_formula << carl::outputSMTLIB(carl::Logic::QF_NRA, {f});
// 	of_carl_formula.close();
//
// 	logger->log_info(name(), "Import carl test formula from file carl_formula.smt into z3 formula");
//
// 	clips_smt_solve_formula_from_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/carl_formula.smt");
// }

// void ClipsSmtThread::clips_smt_test_formulaGenerator()
// {
// 	logger->log_info(name(), "Test FormulaGenerator extern binary");
//
// 	GameData gD = FormulaGeneratorTest::createGameDataTestCase();
// 	FormulaGenerator fg = FormulaGenerator(1, gD);
//
// 	logger->log_info(name(), "Export FormulaGenerator formula to file fg_formula.smt");
// 	std::ofstream of_fg_formula("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt"); // TODO (Igor) Exchange path with config value
// 	of_fg_formula << carl::outputSMTLIB(carl::Logic::QF_NIRA, {fg.createFormula()});
// 	of_fg_formula.close();
//
// 	logger->log_info(name(), "Import FormulaGenerator formula from file fg_formula.smt into z3 formula");
//
//
// 	clips_smt_solve_formula_from_fg_smt_file("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/fg_formula.smt", fg);
// }

z3::expr ClipsSmtThread::getVar(std::map<std::string, z3::expr>& vars, std::string var_id)
{
	// Define iterator
	std::map<std::string, z3::expr>::iterator it;

	// Look for var
	it = vars.find(var_id);
	if(it != vars.end()) {
		// Found
		return it->second;
	}
	else {
		// Not found
		logger->log_error(name(), "Variable with id %s not found", var_id.c_str());
	}

	// Return default value false
	return _z3_context.bool_val(false);
}

std::string ClipsSmtThread::getCapColor(int product_id)
{
	if(product_id==4 || product_id==6 || product_id==8 || product_id==10){
		return "CAP_BLACK";
	}
	else if(product_id==5 || product_id==7 || product_id==9 || product_id==11){
		return "CAP_GREY";
	}

	return "empty";
}

std::string ClipsSmtThread::getBaseColor(int product_id)
{
	if(product_id==1 || product_id==4 || product_id==5){
		return "BASE_RED";
	}
	else if(product_id==2 || product_id==6 || product_id==7){
		return "BASE_BLACK";
	}
	else if(product_id==3 || product_id==8 || product_id==9){
		return "BASE_SILVER";
	}

	return "empty";
}
