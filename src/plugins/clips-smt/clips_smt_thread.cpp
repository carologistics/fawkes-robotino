
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

	// initialize maps for Francesco Leofante's encoder for C0-C1
	// state1_machines["not_prep"]=0;
	// state1_machines["retrieve_C1"]=1;
	// state1_machines["retrieve_C2"]=2;
	// state1_machines["mount_C1"]=3;
	// state1_machines["mount_C2"]=4;
	// state1_machines["prep_B1"]=5;
	// state1_machines["prep_B2"]=6;
	// state1_machines["prep_B3"]=7;
	// state1_machines["slide_one_prep_B1C1"]=8;
	// state1_machines["slide_one_prep_B1C2"]=9;
	// state1_machines["slide_one_prep_B2C1"]=10;
	// state1_machines["slide_one_prep_B2C2"]=11;
	// state1_machines["slide_one_prep_B3C1"]=12;
	// state1_machines["slide_one_prep_B3C2"]=13;

	state2_machines["empty"]=0;
	state2_machines["has_C1"]=1;
	state2_machines["has_C2"]=2;
	// state2_machines["has_R1"]=3;
	// state2_machines["has_R2"]=4;
	// state2_machines["has_R3"]=5;
	// state2_machines["has_R4"]=6;

	// Add product names by iterating over all possible products
	// unsigned int ctr = 0;
	// for(unsigned int b=1; b<4; ++b){
	//     for(unsigned int r1=0; r1<5; ++r1) {
	//         for(unsigned int r2=0; r2<5; ++r2) {
	//             for(unsigned int r3=0; r3<5; ++r3) {
	//                 for(unsigned int c=0; c<3; ++c) {
					
	//                 std::string name = "B"+std::to_string(b);
	//                 if(r1>0) name += "R"+std::to_string(r1);
	//                 if(r1>0 && r2>0) name += "R"+std::to_string(r2);
	//                 else if(r1==0 && r2>0) continue;
	//                 if(r1>0 && r2>0 && r3>0) name += "R"+std::to_string(r3);
	//                 else if(r1==0 && r2==0 && r3>0) continue;
	//                 if(c>0) name += "C"+std::to_string(c);

	//                 ctr++;

	//                 std::cout << ctr << "." << name.c_str() << std::endl;

	//                 state3_machines[name] = ctr;
	//                 products[name] = ctr;
	//                 products_inverted[ctr] = name;
	//                 }
	//             }
	//         }
	//     }
	// }

	// Add product names by iterating over all possible products
	unsigned int ctr = 0;

	// Only bases
	for(unsigned int b=1; b<4; ++b){
		std::string name = "B"+std::to_string(b);

		ctr++;
		state3_machines[name] = ctr;
		products[name] = ctr;
		products_inverted[ctr] = name;

		std::cout << ctr << "." << name.c_str() << std::endl;
	}

	/*
	 * C0
	 */
	
	// Only bases and caps
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int c=1; c<3; ++c) {
			std::string name = "B"+std::to_string(b)+"C"+std::to_string(c);

			ctr++;
			state3_machines[name] = ctr;
			products[name] = ctr;
			products_inverted[ctr] = name;

			std::cout << ctr << "." << name.c_str() << std::endl;
		}
	}

	// Only bases and ring1 
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int r1=1; r1<5; ++r1) {
			std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1);

			ctr++;
			state3_machines[name] = ctr;
			products[name] = ctr;
			products_inverted[ctr] = name;

			std::cout << ctr << "." << name.c_str() << std::endl;
		}
	}

	/*
	 * C1
	 */

	// Only bases, ring1 and cap 
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int r1=1; r1<5; ++r1) {
			for(unsigned int c=1; c<3; ++c) {
				std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"C"+std::to_string(c);

				ctr++;
				state3_machines[name] = ctr;
				products[name] = ctr;
				products_inverted[ctr] = name;

				std::cout << ctr << "." << name.c_str() << std::endl;
			}
		}
	}

	// Only bases, ring1 and ring2
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int r1=1; r1<5; ++r1) {
			for(unsigned int r2=1; r2<5; ++r2) {
				std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2);

				ctr++;
				state3_machines[name] = ctr;
				products[name] = ctr;
				products_inverted[ctr] = name;

				std::cout << ctr << "." << name.c_str() << std::endl;
			}
		}
	}

	/*
	 * C2
	 */

	// Only bases, ring1, ring2 and cap 
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int r1=1; r1<5; ++r1) {
			for(unsigned int r2=1; r2<5; ++r2) {
				for(unsigned int c=1; c<3; ++c) {
					std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2)+"C"+std::to_string(c);

					ctr++;
					state3_machines[name] = ctr;
					products[name] = ctr;
					products_inverted[ctr] = name;

					std::cout << ctr << "." << name.c_str() << std::endl;
				}
			}
		}
	}

	// Only bases, ring1, ring2 and ring3
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int r1=1; r1<5; ++r1) {
			for(unsigned int r2=1; r2<5; ++r2) {
				for(unsigned int r3=1; r3<5; ++r3) {
					std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2)+"R"+std::to_string(r3);

					ctr++;
					state3_machines[name] = ctr;
					products[name] = ctr;
					products_inverted[ctr] = name;

					std::cout << ctr << "." << name.c_str() << std::endl;
				}
			}
		}
	}

	/*
	 * C3
	 */

	// Only bases, ring1, ring2, ring3 and cap 
	for(unsigned int b=1; b<4; ++b){
		for(unsigned int r1=1; r1<5; ++r1) {
			for(unsigned int r2=1; r2<5; ++r2) {
				for(unsigned int r3=1; r3<5; ++r3) {
					for(unsigned int c=1; c<3; ++c) {
						std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2)+"R"+std::to_string(r3)+"C"+std::to_string(c);

						ctr++;
						state3_machines[name] = ctr;
						products[name] = ctr;
						products_inverted[ctr] = name;

						std::cout << ctr << "." << name.c_str() << std::endl;
					}
				}
			}
		}
	}

	state3_machines["full"]=-1;
	state3_machines["empty"]=0;

	products["nothing"]=0;
	products_inverted[0]="nothing";

	machine_groups["CS"]=0;
	machine_groups["BS"]=1;
	machine_groups["RS"]=2;
	machine_groups["DS"]=3;

	// Visualize MACRO-actions
	actions[1] = "|R|CSS|>>BRC| |P|CS|BRC>>| |F|CS|>BRC>|";
	actions[2] = "|R|CS|>C>BR|";
	actions[3] = "|P|BS|B>>| |R|BS|>>B|";
	actions[4] = "|P|CS|B>C>| |F|CS|>B+C>|";
	actions[5] = "|R|CS|>>BC|";
	actions[6] = "|P|DS|BC>>| |F|DS|>BC>|";
	actions[7] = "|F|RS|>B>";
	actions[8] = "|P|RS|B>R>| |F|RS|>B+R>|";
	actions[9] = "|R|RS|>>BR";
	actions[10] = "|P|RS|BR>R>| |F|RS|>BR+R>|";
	actions[11] = "|R|RS|>>BRR";
	actions[12] = "|P|RS|BRR>R>| |F|RS|>BRR+R>|";
	actions[13] = "|R|RS|>>BRRR";

	// Initialize shelf positions // TODO Use all shelf-positions in protobuf
	shelf_position.push_back(true);
	shelf_position.push_back(true);

	// Initialize robot_permutation
	robot_permutation_[1]=1;
	robot_permutation_[2]=2;
	robot_permutation_[3]=3;

	// Initialize number_required_bases TODO read dynamically
	number_required_bases.push_back(0);
	number_required_bases.push_back(1);
	number_required_bases.push_back(2);
	number_required_bases.push_back(3);

	// Initialize rings_order TOOD read dynamically
	rings_order.push_back(2);
	rings_order.push_back(0);
	rings_order.push_back(1);
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
	// llsf_msgs::uint32 *uint32;

	// action = plan->add_actions();
	// action->set_name("enter-field");

	// Francesco Leofante's approach
	std::map<uint32_t,uint32_t> action_id_last;
	uint32_t action_id=0;

	for(unsigned int i=0; i<model_actions.size(); ++i){
		switch(model_actions[i]) { // }((model_actions[i]-1)%index_upper_bound_actions)+1){
			case 1:	// Actions 1,2,3

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve_shelf");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("shelf");
					param->set_value("TRUE"); // TODO Replace TRUE by getNextShelf() once implemented

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("operation");
					param->set_value("RETRIEVE_CAP");

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 2:	// Action 8

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[1]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("discard");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);

					break;
			case 3:	// Action 7,6

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[1]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[1]);
					param = action->add_params();
					param->set_key("color");
					param->set_value(getBaseColor(data.orders((model_actions[i]-1)/number_total_actions).base_color()));

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[1]);

					break;
			case 4:	// Action 4,5

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					if(number_orders_c0) {
						action->add_parent_id(action_id_last[3]);
					}
					else if(number_orders_c1) {
						action->add_parent_id(action_id_last[9]);
					}
					else if(number_orders_c2) {
						action->add_parent_id(action_id_last[11]);
					}
					else if(number_orders_c3) {
						action->add_parent_id(action_id_last[13]);
					}
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[1]);
					action->add_parent_id(action_id_last[2]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("operation");
					param->set_value("MOUNT_CAP");

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;
			case 5:	// Action 9

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[2]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[4]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 6:	// Action 10,11

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[5]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[6]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[6]);
					param = action->add_params();
					param->set_key("gate");
					param->set_value(std::to_string(data.orders((model_actions[i]-1)/index_upper_bound_actions).delivery_gate()));

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[6]);

					break;

			case 7: // Feed base into RS as payment

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[3]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					// param = action->add_params();
					// param->set_key("operation");
					// param->set_value("PAY_RING"); // TODO add param

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 8: // Action prepare RS and feed base for first ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[3]);
					action->add_parent_id(action_id_last[7]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					// param = action->add_params();
					// param->set_key("operation");
					// param->set_value("MOUNT_RING"); // TODO add param

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 9: // Action retrieve base_ring at RS for first ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[8]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 10: // Action prepare RS and feed base for second ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[3]);
					action->add_parent_id(action_id_last[7]);
					action->add_parent_id(action_id_last[9]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					// param = action->add_params();
					// param->set_key("operation");
					// param->set_value("MOUNT_RING"); // TODO add param

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

			case 11: // Action retrieve base_ring at RS for second ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[10]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 12: // Action prepare RS and feed base for third ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[3]);
					action->add_parent_id(action_id_last[7]);
					action->add_parent_id(action_id_last[11]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					// param = action->add_params();
					// param->set_key("operation");
					// param->set_value("MOUNT_RING"); // TODO add param

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

			case 13: // Action retrieve base_ring at RS for third ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(robot_permutation_[model_robots[i]]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[12]);
					action->set_goal_id(0);
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			default:	break;
		}

		// Save action_id of last action inside MACRO-actions for dependencies
		action_id_last[model_actions[i]]=action_id;

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

	// Set global variables
	number_robots = data.robots().size()-1;
	number_machines = 10; // data.machines().size();
	
	number_orders_protobuf = data.orders().size();
	number_orders = number_orders_c0 + number_orders_c1 + number_orders_c2 + number_orders_c3; 

	base_order = data.orders(0).base_color();
	// TODO read rings_order
	cap_order = data.orders(0).cap_color();

	number_required_actions_c1 += 2*number_required_bases[rings_order[0]];
	number_required_actions_c2 += 2*number_required_bases[rings_order[0]] + 2*number_required_bases[rings_order[1]];
	number_required_actions_c3 += 2*number_required_bases[rings_order[0]] + 2*number_required_bases[rings_order[1]] + 2*number_required_bases[rings_order[2]];

	plan_horizon = number_orders_c0*number_required_actions_c0 + number_orders_c1*number_required_actions_c1 + number_orders_c2*number_required_actions_c2 + number_orders_c3*number_required_actions_c3;

	if(number_orders_c3) {
		index_upper_bound_actions = index_upper_bound_actions_c3;
		number_required_actions = number_required_actions_c3;
	}
	else if (number_orders_c2) {
		index_upper_bound_actions = index_upper_bound_actions_c2;
		number_required_actions = number_required_actions_c2;
	}
	else if(number_orders_c1) {
		index_upper_bound_actions = index_upper_bound_actions_c1;
		number_required_actions = number_required_actions_c1;
	}
	else {
		index_upper_bound_actions = index_upper_bound_actions_c0;
		number_required_actions = number_required_actions_c0;
	}

	number_total_actions = number_orders*index_upper_bound_actions;

	// Compute distances between nodes and robots using navgraph
	clips_smt_fill_node_names();
	clips_smt_fill_robot_names();
	clips_smt_compute_distances_robots();
	clips_smt_compute_distances_machines();

	/*
	 * Francesco Leofante's approach
	 */

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
	clips_smt_solve_formula(formula, "rew_");

	/*
	 * Leonard Korp's approach
	 */

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
 *	- Set names of nodes
 *	- Set names of robots
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

	// Extract team from protobuf
	std::string team = "M";
	if(data.robots(0).team_color() == 0){
		team = "C";
	}

	// // Read names of machines automatically
	// C-ins-in is never in the list of machines, therefore add it here
	// node_names_[0] = "C-ins-in";

	// for(int i=0; i<number_machines; ++i){
	// 	std::string machine_name = data.machines(i).name().c_str();
	// 	machine_name += "-I";
	// 	// logger->log_info(name(), "Add %s to node_names_", machine_name.c_str());
	// 	node_names_[i+1] = machine_name;
	// }

	// Set only required names of machines fix
	node_names_[0] = team+"-ins-in";
	node_names_[1] = team+"-BS-O";
	node_names_[2] = team+"-CS1-I";
	node_names_[3] = team+"-CS1-O";
	node_names_[4] = team+"-CS2-I";
	node_names_[5] = team+"-CS2-O";
	node_names_[6] = team+"-DS-I";
	node_names_[7] = team+"-RS1-I";
	node_names_[8] = team+"-RS1-O";
	node_names_[9] = team+"-RS2-I";
	node_names_[10] = team+"-RS2-O";

	node_names_inverted[team+"-ins-in"] = 0;
	node_names_inverted[team+"-BS-O"] = 1;
	node_names_inverted[team+"-CS1-I"] = 2;
	node_names_inverted[team+"-CS1-O"] = 3;
	node_names_inverted[team+"-CS2-I"] = 4;
	node_names_inverted[team+"-CS2-O"] = 5;
	node_names_inverted[team+"-DS-I"] = 6;
	node_names_inverted[team+"-RS1-I"] = 7;
	node_names_inverted[team+"-RS1-O"] = 8;
	node_names_inverted[team+"-RS2-I"] = 9;
	node_names_inverted[team+"-RS2-O"] = 10;

	if(config->get_string("/clips-agent/rcll2016/cap-station/assigned-color/"+team+"-CS1").compare("BLACK")==0){
		// team-CS1 contains black c1 caps and C-CS2 grey ones
		colors_input["C1"] = team+"-CS1-I";
		colors_input["C2"] = team+"-CS2-I";
		colors_output["C1"] = team+"-CS1-O";
		colors_output["C2"] = team+"-CS2-O";
	}
	else {
		// team-CS1 contains grey c1 caps and C-CS2 black ones
		colors_input["C2"] = team+"-CS1-I";
		colors_input["C1"] = team+"-CS2-I";
		colors_output["C2"] = team+"-CS1-O";
		colors_output["C1"] = team+"-CS2-O";
	}

	if(true) { //config->get_string("/clips-agent/rcll2016/ring-station/assigned-color/"+team+"-RS1").compare("BLUE")==0){
		// team-RS1 contains blue r1 rings
		colors_input["R1"] = team+"-RS1-I";
		colors_output["R1"] = team+"-RS1-O";
	}
	else {
		// team-RS2 contains blue r1 rings
		colors_input["R1"] = team+"-RS2-I";
		colors_output["R1"] = team+"-RS2-O";
	}
	if(true) { // config->get_string("/clips-agent/rcll2016/ring-station/assigned-color/"+team+"-RS1").compare("GREEN")==0){
		// team-RS1 contains green r2 rings
		colors_input["R2"] = team+"-RS1-I";
		colors_output["R2"] = team+"-RS1-O";
	}
	else {
		// team-RS2 contains green r2 rings
		colors_input["R2"] = team+"-RS2-I";
		colors_output["R2"] = team+"-RS2-O";
	}
	if(false) { // config->get_string("/clips-agent/rcll2016/ring-station/assigned-color/"+team+"-RS1").compare("ORANGE")==0){
		// team-RS1 contains orange r3 rings
		colors_input["R3"] = team+"-RS1-I";
		colors_output["R3"] = team+"-RS1-O";
	}
	else {
		// team-RS2 contains orange r3 rings
		colors_input["R3"] = team+"-RS2-I";
		colors_output["R3"] = team+"-RS2-O";
	}
	if(false) { // config->get_string("/clips-agent/rcll2016/ring-station/assigned-color/"+team+"-RS1").compare("YELLOW")==0){
		// team-RS1 contains yellow r4 rings
		colors_input["R4"] = team+"-RS1-I";
		colors_output["R4"] = team+"-RS1-O";
	}
	else {
		// team-RS2 contains yellow r4 rings
		colors_input["R4"] = team+"-RS2-I";
		colors_output["R4"] = team+"-RS2-O";
	}
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
		if(data.robots(r).name().compare("R-1")==0 || data.robots(r).name().compare("R-2")==0 || data.robots(r).name().compare("R-3")==0) { // Avoid robot 'RefBox'
			NavGraphNode robot_node(data.robots(r).name().c_str(), data.robots(r).pose().x(), data.robots(r).pose().y());
			NavGraphNode from = navgraph->closest_node(robot_node.x(), robot_node.y());

			for (unsigned int i = 1; i < node_names_.size(); ++i) {
				std::pair<std::string, std::string> nodes_pair(robot_node.name(), node_names_[i]);

				NavGraphNode to = navgraph->node(node_names_[i]);
				NavGraphPath p = navgraph->search_path(from, to);

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

	// // Prepare file stream to export the distances between machines
	// std::ofstream of_distances;
	// of_distances.open ("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/navgraph-costs-000.csv");

	// Compute distances between unconnected C-ins-in and all other machines
	NavGraphNode ins_node(navgraph->node(node_names_[0]));
	NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		std::pair<std::string, std::string> nodes_pair(ins_node.name(), node_names_[i]);

		NavGraphNode to = navgraph->node(node_names_[i]);
		NavGraphPath p = navgraph->search_path(from, to);

		// of_distances << node_names_[0] << ";" << node_names_[i].c_str() <<";" << p.cost()+navgraph->cost(from, ins_node) << "\n";
		distances_[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
	}

	// Compute distances between machines
	for (unsigned int i = 1; i < node_names_.size(); ++i) {
		for (unsigned int j = 1; j < node_names_.size(); ++j) {
			if (i == j) continue;
			std::pair<std::string, std::string> nodes_pair(node_names_[i], node_names_[j]);

			NavGraphPath p = navgraph->search_path(node_names_[i], node_names_[j]);
			//
			// of_distances << node_names_[i].c_str() << ";" << node_names_[j].c_str() <<";" << p.cost() << "\n";
			distances_[nodes_pair] = p.cost();
		}
	}
}

/*
 * Methods for encoding the given protobuf data in formulas
 *	- Encode formula by Francesco Leofante
 *	- Encode formula by Leonard Kopp
 */

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
		// varInit.insert(std::make_pair("initState1_" + std::to_string(i), _z3_context.int_const(("initState1_" + std::to_string(i)).c_str())));
		varInit.insert(std::make_pair("initState2_" + std::to_string(i), _z3_context.int_const(("initState2_" + std::to_string(i)).c_str())));
		varInit.insert(std::make_pair("initState3_" + std::to_string(i), _z3_context.int_const(("initState3_" + std::to_string(i)).c_str())));
	}
	varInit.insert(std::make_pair("initState4", _z3_context.int_const(("initState4"))));
	varInit.insert(std::make_pair("initState5", _z3_context.int_const(("initState5"))));

	// Variables depending on plan_horizon
	for(int i=1; i<plan_horizon+1; ++i){
		varStartTime.insert(std::make_pair("t_" + std::to_string(i), _z3_context.real_const(("t_" + std::to_string(i)).c_str())));
		varRobotDuration.insert(std::make_pair("rd_" + std::to_string(i), _z3_context.real_const(("rd_" + std::to_string(i)).c_str())));
		varRobotPosition.insert(std::make_pair("pos_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(i)).c_str())));
		for(int r=1; r<number_robots+1; ++r){
			varRobotPosition.insert(std::make_pair("pos_" + std::to_string(r) + "_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(r) + "_" + std::to_string(i)).c_str())));
		}
		varMachineDuration.insert(std::make_pair("md_" + std::to_string(i), _z3_context.real_const(("md_" + std::to_string(i)).c_str())));
		varR.insert(std::make_pair("R_" + std::to_string(i), _z3_context.int_const(("R_" + std::to_string(i)).c_str())));
		varA.insert(std::make_pair("A_" + std::to_string(i), _z3_context.int_const(("A_" + std::to_string(i)).c_str())));
		varM.insert(std::make_pair("M_" + std::to_string(i), _z3_context.int_const(("M_" + std::to_string(i)).c_str())));
		varHold.insert(std::make_pair("holdA_" + std::to_string(i), _z3_context.int_const(("holdA_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state2A_" + std::to_string(i), _z3_context.int_const(("state2A_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state3A_" + std::to_string(i), _z3_context.int_const(("state3A_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state3A_" + std::to_string(i), _z3_context.int_const(("state3A_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state4A_" + std::to_string(i), _z3_context.int_const(("state4A_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state5A_" + std::to_string(i), _z3_context.int_const(("state5A_" + std::to_string(i)).c_str())));
		varHold.insert(std::make_pair("holdB_" + std::to_string(i), _z3_context.int_const(("holdB_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state2B_" + std::to_string(i), _z3_context.int_const(("state2B_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state3B_" + std::to_string(i), _z3_context.int_const(("state3B_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state4B_" + std::to_string(i), _z3_context.int_const(("state4B_" + std::to_string(i)).c_str())));
		varS.insert(std::make_pair("state5B_" + std::to_string(i), _z3_context.int_const(("state5B_" + std::to_string(i)).c_str())));
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
		for(int r=1; r<number_robots+1; ++r){
			constraints.push_back(0 <= getVar(varRobotPosition, "pos_"+std::to_string(r)+"_"+std::to_string(i)) && getVar(varRobotPosition, "pos_"+std::to_string(r)+"_"+std::to_string(i)) <= number_machines); // VarRobotPosition
		}
		constraints.push_back(0 <= getVar(varMachineDuration, "md_"+std::to_string(i))); // VarMachineDuration
		constraints.push_back(1 <= getVar(varR, "R_"+std::to_string(i)) && getVar(varR, "R_"+std::to_string(i)) <= number_robots); // VarR
		constraints.push_back(1 <= getVar(varA, "A_"+std::to_string(i)) && getVar(varA, "A_"+std::to_string(i)) <= number_total_actions); // VarA
		constraints.push_back(min_machine_groups <= getVar(varM, "M_"+std::to_string(i)) && getVar(varM, "M_"+std::to_string(i)) <= max_machine_groups); // VarM
		constraints.push_back(min_products <= getVar(varHold, "holdA_"+std::to_string(i)) && getVar(varHold, "holdA_"+std::to_string(i)) <= max_products); // VarHoldA
		constraints.push_back(min_state2_machines <= getVar(varS, "state2A_"+std::to_string(i)) && getVar(varS, "state2A_"+std::to_string(i)) <= max_state2_machines); // VarState2A
		constraints.push_back(min_state3_machines <= getVar(varS, "state3A_"+std::to_string(i)) && getVar(varS, "state3A_"+std::to_string(i)) <= max_state3_machines); // VarState3A
		constraints.push_back(min_state45_machines <= getVar(varS, "state4A_"+std::to_string(i)) && getVar(varS, "state4A_"+std::to_string(i)) <= max_state45_machines); // VarState4A
		constraints.push_back(min_state45_machines <= getVar(varS, "state5A_"+std::to_string(i)) && getVar(varS, "state5A_"+std::to_string(i)) <= max_state45_machines); // VarState5A
		constraints.push_back(min_products <= getVar(varHold, "holdB_"+std::to_string(i)) && getVar(varHold, "holdB_"+std::to_string(i)) <= max_products); // VarHoldB
		constraints.push_back(min_state2_machines <= getVar(varS, "state2B_"+std::to_string(i)) && getVar(varS, "state2B_"+std::to_string(i)) <= max_state2_machines); // VarState2B
		constraints.push_back(min_state3_machines <= getVar(varS, "state3B_"+std::to_string(i)) && getVar(varS, "state3B_"+std::to_string(i)) <= max_state3_machines); // VarState3B
		constraints.push_back(min_state45_machines <= getVar(varS, "state4B_"+std::to_string(i)) && getVar(varS, "state4B_"+std::to_string(i)) <= max_state45_machines); // VarState4A
		constraints.push_back(min_state45_machines <= getVar(varS, "state5B_"+std::to_string(i)) && getVar(varS, "state5B_"+std::to_string(i)) <= max_state45_machines); // VarState5A

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

	// Constraint: robots can not occupy the same position in the same action step
	for(int i=1; i<plan_horizon+1; ++i){
		for(int r=1; r<number_robots+1; ++r){

			// If robot r is acting in step i then set his position to the position in step i
			z3::expr constraint_precondition( getVar(varR, "R_"+std::to_string(i))==r );
			z3::expr constraint_effect( getVar(varRobotPosition, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(varRobotPosition, "pos_"+std::to_string(i)) );

			for(int rp=1; rp<number_robots+1; ++rp){
				if(r!=rp){
					if(i==1){
						constraint_effect = constraint_effect && getVar(varRobotPosition, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == getVar(varInit, "initPos_"+std::to_string(rp));
					}
					else {
						constraint_effect = constraint_effect && getVar(varRobotPosition, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == getVar(varRobotPosition, "pos_"+std::to_string(rp)+"_"+std::to_string(i-1));
					}
				}
			}

			constraints.push_back( !(constraint_precondition) || constraint_effect);

			// All robot's positions must exclude each other
			for(int rp=r+1; rp<number_robots+1; ++rp){
				constraints.push_back( !(getVar(varRobotPosition, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(varRobotPosition, "pos_"+std::to_string(rp)+"_"+std::to_string(i)))
			 							|| ( getVar(varRobotPosition, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == 0
											&& getVar(varRobotPosition, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == 0) );
			}
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

			z3::expr constraint2(getVar(varS, "state2A_"+std::to_string(ip))==getVar(varInit, "initState2_"+std::to_string(i))
								&& getVar(varS, "state3A_"+std::to_string(ip))==getVar(varInit, "initState3_"+std::to_string(i)));

			constraints.push_back(constraint1 || constraint2);
		}
	}
	constraints.push_back(getVar(varS, "state4A_"+std::to_string(1))==getVar(varInit, "initState4")
						&& getVar(varS, "state5A_"+std::to_string(1))==getVar(varInit, "initState5"));

	// Constraint: machine states are inductively consistent
	logger->log_info(name(), "Add constraints stating robot states are inductively consistent");

	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(varM, "M_"+std::to_string(ip)) == getVar(varM, "M_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || (getVar(varM, "M_"+std::to_string(ipp)) == getVar(varM, "M_"+std::to_string(i)));
			}

			z3::expr constraint2(getVar(varStartTime, "t_"+std::to_string(ip))>=getVar(varStartTime, "t_"+std::to_string(i))+getVar(varMachineDuration, "md_"+std::to_string(i))
									&& getVar(varS, "state2B_"+std::to_string(i))==getVar(varS, "state2A_"+std::to_string(ip))
									&& getVar(varS, "state3B_"+std::to_string(i))==getVar(varS, "state3A_"+std::to_string(ip)));

			constraints.push_back(constraint1 || constraint2);

		}

		if(i<plan_horizon){
			constraints.push_back(getVar(varS, "state4B_"+std::to_string(i))==getVar(varS, "state4A_"+std::to_string(i+1))
									&& getVar(varS, "state5B_"+std::to_string(i))==getVar(varS, "state5A_"+std::to_string(i+1)));
		}
	}

	// Constraint: every action is encoded for every order
	logger->log_info(name(), "Add constraints for actions for %i orders (%i inside protobuf)", number_orders, number_orders_protobuf);

	// Save ids of order_colors as strings
	std::string base_order_str = std::to_string(base_order);
	std::string ring1_order_str = std::to_string(rings_order[0]);
	std::string ring2_order_str = std::to_string(rings_order[1]);
	std::string ring3_order_str = std::to_string(rings_order[2]);
	std::string cap_order_str = std::to_string(cap_order);

	// Init string identifiers for state maps
	std::string bi = "B";
	bi += base_order_str;
	std::string r1i = "R";
	r1i += ring1_order_str;
	std::string r2i = "R";
	r2i += ring2_order_str;
	std::string r3i = "R";
	r3i += ring3_order_str;
	std::string ci = "C";
	ci +=cap_order_str;


	// BR represents the random base or so-called cap carrier
	std::string br_ci = "BR";
	br_ci += ci;

	// Construct combined string identifiers
	std::string bi_ci = bi;
	bi_ci += ci;
	std::string bi_r1i = bi;
	bi_r1i += r1i;
	std::string bi_r1i_ci = bi_r1i;
	bi_r1i_ci += ci;
	std::string bi_r1i_r2i = bi_r1i;
	bi_r1i_r2i += r2i;
	std::string bi_r1i_r2i_ci = bi_r1i_r2i;
	bi_r1i_r2i_ci += ci;
	std::string bi_r1i_r2i_r3i = bi_r1i_r2i;
	bi_r1i_r2i_r3i += r3i;
	std::string bi_r1i_r2i_r3i_ci = bi_r1i_r2i_r3i;
	bi_r1i_r2i_r3i_ci += ci;

	std::string has_ci = "has_";
	has_ci += ci;
	std::string has_r1i = "has_";
	has_r1i += r1i;
	std::string has_r2i = "has_";
	has_r2i += r2i;
	std::string has_r3i = "has_";
	has_r3i += r3i;

	// For every step up to the plan_horizon add all required actions depending on the order complexity
	for(int i=1; i<plan_horizon+1; ++i){

		z3::expr constraint_rs1( getVar(varRobotPosition, "pos_"+std::to_string(i))==7 );
		z3::expr constraint_rs2( getVar(varRobotPosition, "pos_"+std::to_string(i))==9 );

		if(number_orders_c0) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 1)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_ci])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_ci])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 6)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);
		}
		else if(number_orders_c1) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 1)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i_ci])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i_ci])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 6)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);

			// 7.Macroaction : Feed base into ringstation
			z3::expr constraint_macroaction7((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))+1 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))+1
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["B1"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 7) || constraint_macroaction7);

			// 8.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction8((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i])
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))-number_required_bases[rings_order[0]] 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))-number_required_bases[rings_order[0]]
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 8) || constraint_macroaction8);

			// 9.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction9((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										// && (getVar(varS, "state1B_"+std::to_string(i)) == getVar(varS, "state1A_"+std::to_string(i)))
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 9) || constraint_macroaction9);
		}
		else if(number_orders_c2) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 1)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i_r2i_ci])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i_r2i_ci])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 6)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);

			// 7.Macroaction : Feed base into ringstation
			z3::expr constraint_macroaction7((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))+1 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))+1
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["B1"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 7) || constraint_macroaction7);

			// 8.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction8((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i])
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))-number_required_bases[rings_order[0]] 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))-number_required_bases[rings_order[0]]
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 8) || constraint_macroaction8);

			// 9.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction9((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 9) || constraint_macroaction9);

			// 10.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction10((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i_r2i])
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))-number_required_bases[rings_order[1]] 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))-number_required_bases[rings_order[1]]
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r2i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 10) || constraint_macroaction10);

			// 11.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction11((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i_r2i])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r2i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 11) || constraint_macroaction11);
		}
		else if(number_orders_c3) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["full"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(varM, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 1)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines[has_ci])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i_r2i_r3i_ci])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(varM, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(varS, "state2A_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == state2_machines["empty"])
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i_r2i_r3i_ci])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i_ci])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(varM, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == 6)
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i_ci])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);

			// 7.Macroaction : Feed base into ringstation
			z3::expr constraint_macroaction7((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3B_"+std::to_string(i)) == getVar(varS, "state3A_"+std::to_string(i)))
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))+1 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))+1
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["B1"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 7) || constraint_macroaction7);

			// 8.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction8((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i])
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))-number_required_bases[rings_order[0]] 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))-number_required_bases[rings_order[0]]
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 8) || constraint_macroaction8);

			// 9.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction9((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r1i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 9) || constraint_macroaction9);

			// 10.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction10((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i_r2i])
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))-number_required_bases[rings_order[1]] 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))-number_required_bases[rings_order[1]]
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r2i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 10) || constraint_macroaction10);

			// 11.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction11((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i_r2i])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r2i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 11) || constraint_macroaction11);

			// 12.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction12((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines[bi_r1i_r2i_r3i])
										&& ( !constraint_rs1 || ( getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))-number_required_bases[rings_order[2]] 
										   && getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i))-number_required_bases[rings_order[2]]
											&& getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i))) )
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r3i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 12) || constraint_macroaction12);

			// 13.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction13((getVar(varM, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(varS, "state2B_"+std::to_string(i)) == getVar(varS, "state2A_"+std::to_string(i)))
										&& (getVar(varS, "state3A_"+std::to_string(i)) == state3_machines[bi_r1i_r2i_r3i])
										&& (getVar(varS, "state3B_"+std::to_string(i)) == state3_machines["empty"])
										&& (getVar(varS, "state4B_"+std::to_string(i)) == getVar(varS, "state4A_"+std::to_string(i)))
										&& (getVar(varS, "state5B_"+std::to_string(i)) == getVar(varS, "state5A_"+std::to_string(i)))
										&& (getVar(varMachineDuration, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(varRobotPosition, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r3i]])
										&& (getVar(varHold, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(varHold, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(varRobotDuration, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(varA, "A_"+std::to_string(i)) == 13) || constraint_macroaction13);
		}
	}

	logger->log_info(name(), "Add constraints for goal state");

	// Specify goal state
	for(int o=0; o<number_orders; ++o) {
		

		for(int i=1; i<plan_horizon+1; ++i){
			if(i==1){
				constraints.push_back((getVar(varA, "A_"+std::to_string(i)) == o*index_upper_bound_actions+index_delivery_action
											&& getVar(varRew, "rew_"+std::to_string(i)) == (deadline-getVar(varStartTime, "t_"+std::to_string(i))-getVar(varMachineDuration, "md_"+std::to_string(i))))
										|| (!(getVar(varA, "A_"+std::to_string(i)) == o*index_upper_bound_actions+index_delivery_action)
											&& getVar(varRew, "rew_"+std::to_string(i))==0));
			}
			else {
					constraints.push_back((getVar(varA, "A_"+std::to_string(i)) == o*index_upper_bound_actions+index_delivery_action
												&& getVar(varRew, "rew_"+std::to_string(i)) == (getVar(varRew, "rew_"+std::to_string(i-1))+deadline-getVar(varStartTime, "t_"+std::to_string(i))-getVar(varMachineDuration, "md_"+std::to_string(i))))
											|| (!(getVar(varA, "A_"+std::to_string(i)) == o*index_upper_bound_actions+index_delivery_action)
												&& getVar(varRew, "rew_"+std::to_string(i))==getVar(varRew, "rew_"+std::to_string(i-1))));
			}
		}
	}

	logger->log_info(name(), "Add constraints for initial situation");

	// Specify initial situation for robots
	for(int i=1; i<number_robots+1; ++i){
		constraints.push_back(getVar(varInit, "initHold_"+std::to_string(i))==products["nothing"]);
		constraints.push_back(getVar(varInit, "initPos_"+std::to_string(i))==0);
	}

	// Specify initial situation for machines
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		if(i==3) {
			constraints.push_back(getVar(varInit, "initState2_"+std::to_string(i))==state2_machines["has_R1"]); // TODO Adapt to the dynamic case
		}
		else {
			constraints.push_back(getVar(varInit, "initState2_"+std::to_string(i))==state2_machines["empty"]);
		}
		constraints.push_back(getVar(varInit, "initState3_"+std::to_string(i))==state3_machines["empty"]);
	}
	constraints.push_back(getVar(varInit, "initState4")==0);
	constraints.push_back(getVar(varInit, "initState5")==0);

	logger->log_info(name(), "Add constraints for distances between machines");

	// Specify distances between machines
	for(int k=0; k<number_machines+1; ++k){
		for(int l=k+1; l<number_machines+1; ++l){
			float distance = velocity_scaling_ * distances_[std::make_pair(node_names_[k], node_names_[l])];
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

	constraints.push_back(getVar(varR, "R_1") == 1);

	logger->log_info(name(), "Add constraints for final actions");

	// Constraints encoding that final_actions for each order have to be at least executed once (if desiered, during the delivery window)
	z3::expr constraint_goal(var_true);
	for(int o=0; o<number_orders; ++o){

		z3::expr constraint_subgoal(var_false);
		for(int i=number_required_actions; i<plan_horizon+1; ++i){

			z3::expr constraint_finalaction(getVar(varA, "A_"+std::to_string(i)) == o*index_upper_bound_actions+index_delivery_action);

			if(add_temporal_constraint){
				constraints.push_back(!constraint_finalaction || (getVar(varStartTime, "t_"+std::to_string(i)) < (int) data.orders(o).delivery_period_end()
																		&& getVar(varStartTime, "t_"+std::to_string(i)) > (int) data.orders(o).delivery_period_begin()));
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
 ClipsSmtThread::clips_smt_solve_formula(z3::expr_vector formula, std::string var)
{
	logger->log_info(name(), "Solve z3 formula");

	// z3::solver z3Optimizer(_z3_context); // Use for solving
	z3::optimize z3Optimizer(_z3_context); // Use for optimizing

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

	z3::expr rew_planhorizon = _z3_context.real_const((var.c_str()+std::to_string(plan_horizon)).c_str());

	z3Optimizer.maximize(rew_planhorizon);

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
	of_c0_formula << Z3_optimize_to_string(_z3_context, z3Optimizer);
	// of_c0_formula << z3Optimizer.to_smt2() << std::endl;
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
					std::string cw_pos_R1 = "pos_1_";
					cw_pos_R1 += std::to_string(j); std::string cw_pos_R2 = "pos_2_";
					cw_pos_R2 += std::to_string(j);
					std::string cw_pos_R3 = "pos_3_";
					cw_pos_R3 += std::to_string(j);
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
					std::string cw_state4A = "state4A_";
					cw_state4A += std::to_string(j);
					std::string cw_state5A = "state5A_";
					cw_state5A += std::to_string(j);
					std::string cw_holdB = "holdB_";
					cw_holdB += std::to_string(j);
					std::string cw_state1B = "state1B_";
					cw_state1B += std::to_string(j);
					std::string cw_state2B = "state2B_";
					cw_state2B += std::to_string(j);
					std::string cw_state3B = "state3B_";
					cw_state3B += std::to_string(j);
					std::string cw_state4B = "state4B_";
					cw_state4B += std::to_string(j);
					std::string cw_state5B = "state5B_";
					cw_state5B += std::to_string(j);

					if(function_name.compare(cw_time)==0) {
						model_times[j] = interp;
					}
					else if(function_name.compare(cw_pos)==0) {
						model_positions[j] = (int) interp;
					}
					else if(function_name.compare(cw_pos_R1)==0) {
						model_positions_R1[j] = (int) interp;
					}
					else if(function_name.compare(cw_pos_R2)==0) {
						model_positions_R2[j] = (int) interp;
					}
					else if(function_name.compare(cw_pos_R3)==0) {
						model_positions_R3[j] = (int) interp;
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
					else if(function_name.compare(cw_state4A)==0) {
						model_state4A[j] = (int) interp;
					}
					else if(function_name.compare(cw_state5A)==0) {
						model_state5A[j] = (int) interp;
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
					else if(function_name.compare(cw_state4B)==0) {
						model_state4B[j] = (int) interp;
					}
					else if(function_name.compare(cw_state5B)==0) {
						model_state5B[j] = (int) interp;
					}

				}
			}
		}

		bool set_robot_permutation = true;
		for(unsigned int i = 1; i < model_robots.size(); ++i){
			if(set_robot_permutation){
				if(i==1){
					robot_permutation_[model_robots[1]]=1;	
				}
				else if(model_robots[i] != model_robots[1]){
					robot_permutation_[model_robots[i]] = 2;
					std::cout << "Try to access robot_permutation_ at position " << 6-(model_robots[1]+model_robots[i]) << std::endl;
					robot_permutation_[6-(model_robots[1]+model_robots[i])] = 3;
					set_robot_permutation = false;
				}
			}
		}

		// Add plan specified by the model to stats
		// of_stats << "number_orders_c0: " << number_orders_c0 << std::endl;
		// of_stats << "add_temporal_constraint: " << add_temporal_constraint << std::endl;
		for(int o=0; o<number_orders; ++o){
			of_stats << "O" << o+1 << ": B" << data.orders(o).base_color() << "C" << data.orders(o).cap_color() ;
			if(add_temporal_constraint){
				of_stats << " with bounds " << data.orders(o).delivery_period_begin() << "s < o0 < " << data.orders(o).delivery_period_end() << "s";
			}
			of_stats << std::endl;
		}

		of_stats << std::endl;

		for(int j=1; j<plan_horizon+1; ++j){
			of_stats << j <<". ";
			of_stats << "R" << model_robots[j] << " for O" << ((model_actions[j]-1)/index_upper_bound_actions)+1;
			of_stats << " does " << actions[((model_actions[j]-1)%index_upper_bound_actions)+1]  << " (A" << model_actions[j] << ")"; //of_stats  << " and holds " << products_inverted[model_holdB[j]] << " at "<< node_names_[model_positions[j]];
			// ":[H(" << model_holdA[j] << "-" << model_holdB[j] <<
			// "), S1(" << model_state1A[j] << "-" << model_state1B[j] <<
			// "), S2(" << model_state2A[j] << "-" << model_state2B[j] <<
			// "), S3(" << model_state3A[j] << "-" << model_state3B[j] <<"]";
			of_stats << " [" << model_state4B[j] << ", " << model_state5B[j] << "]"; 
			of_stats << " [t = " << model_times[j] << "s]" << std::endl; // [R1: " << node_names_[model_positions_R1[j]] <<", R2: " << node_names_[model_positions_R2[j]] << ", R3: " << node_names_[model_positions_R3[j]] << "]" << std::endl;
		}
		// of_stats << " robot_permutation_ has following values:  " << robot_permutation_[1] <<", " <<  robot_permutation_[2] << ", " <<  robot_permutation_[3] << std::endl;
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

z3::expr ClipsSmtThread::clips_smt_extract_formula_from_smt_file(std::string path) {

	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	return e;
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

	std::string product_color;

	switch(product_id){
		case 1: product_color = "CAP_BLACK";
				break;
		case 2: product_color = "CAP_GREY";
				break;
		default: product_color = "EMPTY";
				break;
	}

	return product_color;
}

std::string ClipsSmtThread::getBaseColor(int product_id)
{

	std::string product_color;

	switch(product_id){
		case 1: product_color = "BASE_RED";
				break;
		case 2: product_color = "BASE_BLACK";
				break;
		case 3: product_color = "BASE_SILVER";
				break;
		default: product_color = "EMPTY";
				break;
	}

	return product_color;
}

void ClipsSmtThread::initShelf()
{
	shelf_position[0] = true;
	shelf_position[1] = true;
}

std::string ClipsSmtThread::getNextShelf()
{
	if(shelf_position[0]){
		shelf_position[0] = false;
		return "LEFT";
	}
	else if(shelf_position[1]){
		shelf_position[1] = false;
		return "MIDDLE";
	}

	initShelf();
	return "LEFT";
}
