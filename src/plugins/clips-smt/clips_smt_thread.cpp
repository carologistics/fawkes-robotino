
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

#include <core/threading/mutex_locker.h>
#include <navgraph/navgraph.h>
#include <navgraph/yaml_navgraph.h>
#include <navgraph/constraints/static_list_edge_cost_constraint.h>
#include <navgraph/constraints/constraint_repo.h>
#include <llsf_msgs/Plan.pb.h>

using namespace fawkes;

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
	logger->log_info(name(), "clips_smt_init_pre");

	init_game_once = true;

	// Order
	base_colors["BASE_NONE"] = 0;
	base_colors["BASE_RED"] = 1;
	base_colors["BASE_BLACK"] = 2;
	base_colors["BASE_SILVER"] = 3;
	base_colors_inverted[0] = "BASE_NONE";
	base_colors_inverted[1] = "BASE_RED";
	base_colors_inverted[2] = "BASE_BLACK";
	base_colors_inverted[3] = "BASE_SILVER";
	rings_colors["RING_NONE"] = 0;
	rings_colors["RING_BLUE"] = 1;
	rings_colors["RING_GREEN"] = 2;
	rings_colors["RING_ORANGE"] = 3;
	rings_colors["RING_YELLOW"] = 4;
	rings_colors_inverted[0] = "RING_NONE";
	rings_colors_inverted[1] = "RING_BLUE";
	rings_colors_inverted[2] = "RING_GREEN";
	rings_colors_inverted[3] = "RING_ORANGE";
	rings_colors_inverted[4] = "RING_YELLOW";
	cap_colors["CAP_NONE"] = 0;
	cap_colors["CAP_BLACK"] = 1;
	cap_colors["CAP_GREY"] = 2;
	cap_colors_inverted[0] = "CAP_NONE";
	cap_colors_inverted[1] = "CAP_BLACK";
	cap_colors_inverted[2] = "CAP_GREY";
	add_bases_description[-1] = "ERROR";
	add_bases_description[0] = "ZERO";
	add_bases_description[1] = "ONE";
	add_bases_description[2] = "TWO";
	add_bases_description[3] = "THREE";
	add_bases_description_inverted["ERROR"] = -1;
	add_bases_description_inverted["ZERO"] = 0;
	add_bases_description_inverted["ONE"] = 1;
	add_bases_description_inverted["TWO"] = 2;
	add_bases_description_inverted["THREE"] = 3;
	cap_carrier_colors["CAP_GREY"] = "CCG";
	cap_carrier_colors["CAP_BLACK"] = "CCB";
	cap_carrier_index = 0;

	// Prepare PlanHorizon
	// MACRO ACTIONS
	amount_min_req_actions.clear();
	amount_min_req_actions.push_back(8);
	amount_min_req_actions.push_back(10);
	amount_min_req_actions.push_back(12);
	amount_min_req_actions.push_back(14);
	index_upper_bound_actions.push_back(7);
	index_upper_bound_actions.push_back(9);
	index_upper_bound_actions.push_back(11);
	index_upper_bound_actions.push_back(13);
	amount_req_actions_add_bases = 2;

	// Inside encodes if the CS has the cap retrieved in order to mount it with some subproduct.
	inside_capstation["nothing"]=0;
	inside_capstation["has_C1"]=1;
	inside_capstation["has_C2"]=2;

	// For each relevant machine we assign a group
	machine_groups["BS"]=0;
	machine_groups["RS1"]=1;
	machine_groups["RS2"]=2;
	machine_groups["CS1"]=3;
	machine_groups["CS2"]=4;

	// Products encodes the output of an station which can be any product (at the CS) and subproduct (at the BS and RS) OR the product a robot is holding
	products["nothing"]=0;
	products["BRC1"]=1;
	products["BRC2"]=2;
	products["BR"]=3;

	unsigned ctr = 3;

	// B1 ... B3
	for(unsigned b=1; b<4; ++b){
		std::string name = "B"+std::to_string(b);
		ctr++;
		products[name] = ctr;
	}

	// B1C1 ... B3C2
	for(unsigned b=1; b<4; ++b){
		for(unsigned c=1; c<3; ++c) {
			std::string name = "B"+std::to_string(b)+"C"+std::to_string(c);
			ctr++;
			products[name] = ctr;
		}
	}

	// B1R1 ... B3R4
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1);
			ctr++;
			products[name] = ctr;
		}
	}

	// B1R1C1 ... B3R4C2
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			for(unsigned c=1; c<3; ++c) {
				std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"C"+std::to_string(c);
				ctr++;
				products[name] = ctr;
			}
		}
	}

	// B1R1R1 ... B3R4R4
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			for(unsigned r2=1; r2<5; ++r2) {
				std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2);
				ctr++;
				products[name] = ctr;
			}
		}
	}

	// B1R1R1C1 ... B3R4R4C2
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			for(unsigned r2=1; r2<5; ++r2) {
				for(unsigned c=1; c<3; ++c) {
					std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2)+"C"+std::to_string(c);
					ctr++;
					products[name] = ctr;
				}
			}
		}
	}

	// B1R1R1R1 ... B3R4R4R4
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			for(unsigned r2=1; r2<5; ++r2) {
				for(unsigned r3=1; r3<5; ++r3) {
					std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2)+"R"+std::to_string(r3);
					ctr++;
					products[name] = ctr;
				}
			}
		}
	}

	// B1R1R1R1C1 ... B3R4R4R4C2
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			for(unsigned r2=1; r2<5; ++r2) {
				for(unsigned r3=1; r3<5; ++r3) {
					for(unsigned c=1; c<3; ++c) {
						std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"R"+std::to_string(r2)+"R"+std::to_string(r3)+"C"+std::to_string(c);
						ctr++;
						products[name] = ctr;
					}
				}
			}
		}
	}
	// Shelf positions
	shelf_position.push_back(true);
	shelf_position.push_back(true);

	// Initialize world state fix
	world_initHold.push_back(0); // dummy
	world_initHold.push_back(products["nothing"]);
	world_initHold.push_back(products["nothing"]);
	world_initHold.push_back(products["nothing"]);

	world_initPos.push_back(0); // dummy
	world_initPos.push_back(0);
	world_initPos.push_back(0);
	world_initPos.push_back(0);

	world_initInside.push_back(0); // BS
	world_initInside.push_back(inside_capstation["nothing"]); // CS1
	world_initInside.push_back(inside_capstation["nothing"]); // CS2
	world_initInside.push_back(0); // RS1
	world_initInside.push_back(0); // RS2

	world_initOutside.push_back(products["nothing"]); // BS
	world_initOutside.push_back(products["nothing"]); // RS1
	world_initOutside.push_back(products["nothing"]); // RS2
	world_initOutside.push_back(products["nothing"]); // CS1
	world_initOutside.push_back(products["nothing"]); // CS2

	world_points = 0;
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

	data.CopyFrom(**m); // Use data with subpoint-methods, e.g. data.robots(0).name() OR amount_machines
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
	actor_plan->set_actor_name("Master");
	llsf_msgs::SequentialPlan *plan = actor_plan->mutable_sequential_plan();
	llsf_msgs::PlanAction *action;
	llsf_msgs::PlanActionParameter *param;
	// llsf_msgs::uint32 *uint32;

	// action = plan->add_actions();
	// action->set_name("enter-field");

	// Francesco Leofante's approach
	std::vector<uint32_t> action_id_last;
	action_id_last.clear();
	for(int i=0; i<=index_upper_bound_actions[desired_complexity]; ++i){
		action_id_last.push_back(0);
	}
	std::vector<uint32_t> action_id_last_bs_robot;
	action_id_last_bs_robot.clear();
	for(int i=0; i<=4; ++i){
		action_id_last_bs_robot.push_back(0);
	}
	std::vector<uint32_t> action_id_last_rs1_pay;
	action_id_last_rs1_pay.clear();
	std::vector<uint32_t> action_id_last_rs2_pay;
	action_id_last_rs2_pay.clear();
	uint32_t action_id_last_rs1_feed=0;
	uint32_t action_id_last_rs1_retr=0;
	uint32_t action_id_last_rs2_feed=0;
	uint32_t action_id_last_rs2_retr=0;

	uint32_t action_id=0;
	std::string wp;
	unsigned int j;
	bool search_for_old_position;

	for(unsigned int i=1; i<=model_actions.size(); ++i){
		switch(model_actions[i]) {

			case 1:	// 1.Action: Retrieve cap_carrier_cap from CS-Shelf

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}


					++action_id;
					action = plan->add_actions();
					action->set_name("wp-get-shelf");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("shelf");
					param->set_value(getNextShelf());
					param = action->add_params();
					param->set_key("wp");
					param->set_value(cap_carrier_colors[cap_colors_inverted[data.orders(order_id).cap_color()]]+std::to_string(cap_carrier_index));

					break;

			case 2:	// 2.Action: Prepare and feed CS for RETRIEVE with cap_carrier

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare-cs");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("operation");
					param->set_value("RETRIEVE_CAP");

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value(cap_carrier_colors[cap_colors_inverted[data.orders(order_id).cap_color()]]+std::to_string(cap_carrier_index));

					++action_id;
					action = plan->add_actions();
					action->set_name("cs-retrieve-cap");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("cap-color");
					param->set_value(cap_colors_inverted[data.orders(order_id).cap_color()]);

					break;

			case 3:	// Action 3: Retrieve cap_carrier at CS

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					if(model_outputA[i] == products["BR"]) {
						++action_id;
						action = plan->add_actions();
						action->set_name("wp-get");
						action->set_actor("R-"+std::to_string(model_robots[i]));
						action->set_id(action_id);
						action->add_parent_id(action_id-1);
						action->add_parent_id(action_id_last[1]);
						action->set_goal_id(data.orders(order_id).id());
						param = action->add_params();
						param->set_key("mps");
						param->set_value(node_names[model_positions[i]]);
						param = action->add_params();
						param->set_key("wp");
						param->set_value(cap_carrier_colors[cap_colors_inverted[data.orders(order_id).cap_color()]]+std::to_string(cap_carrier_index));
					}
					else {
						++action_id;
						action = plan->add_actions();
						action->set_name("wp-get");
						action->set_actor("R-"+std::to_string(model_robots[i]));
						action->set_id(action_id);
						action->add_parent_id(action_id-1);
						action->add_parent_id(action_id_last[4]);
						action->set_goal_id(data.orders(order_id).id());
						param = action->add_params();
						param->set_key("mps");
						param->set_value(node_names[model_positions[i]]);
						param = action->add_params();
						param->set_key("wp");
						param->set_value("WP1");
					}

					break;

			case 4:	// 4.Action : Prepare and retrieve base from BS

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Only go to the base station if last robot which got a base (mentioned in our sequential plan) finishes its action
					// if(action_id_last[3]) action->add_parent_id(action_id_last[3]);
					// Only go to the base station if the base intended to be mounted comes after all the base retrievals which are intended as payment
					// Look ahead to dermine if this instance of action 3 is the (!) one which requires additional parent_ids
					wp = "WP2"; // TODO How to determine dummy
					for(unsigned j=i+1; j<=model_actions.size(); ++j) {

						// Check if action is an instance of action 8 and the current base retrieval is performed by the same robot
						if(model_actions[j] == 8 && model_robots[i] == model_robots[j]) {

							if(model_positions[j]==7) {
								for(unsigned int k=0; k<action_id_last_rs1_pay.size(); ++k) {
									if((int) k<rings_req_add_bases[rings[0]]) {
										action->add_parent_id(action_id_last_rs1_pay[k]);
									}
									else {
										std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 1." << std::endl;
									}
								}
							}
							else if(model_positions[j]==9) {
								for(unsigned int k=0; k<action_id_last_rs2_pay.size(); ++k) {
									if((int) k<rings_req_add_bases[rings[0]]) {
										action->add_parent_id(action_id_last_rs2_pay[k]);
									}
									else {
										std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 2." << std::endl;
									}
								}
							}
							wp = "WP1";
							break;
						}
						else if(model_actions[j] == 5 && model_robots[i] == model_robots[j]) {
							wp = "WP1";
							break;
						}
						else if(model_actions[j] == 7 && model_robots[i] == model_robots[j]) {
							wp = "WP2";
							break;
						}

					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[1]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					if(model_outputA[i] == products["nothing"]) {
						++action_id;
						action = plan->add_actions();
						action->set_name("prepare-bs");
						action->set_actor("R-"+std::to_string(model_robots[i]));
						action->set_id(action_id);
						action->add_parent_id(action_id-1);
						action->set_goal_id(data.orders(order_id).id());
						param = action->add_params();
						param->set_key("mps");
						param->set_value(node_names[1]);
						param = action->add_params();
						param->set_key("color");
						param->set_value(base_colors_inverted[data.orders(order_id).base_color()]);

						++action_id;
						action = plan->add_actions();
						action->set_name("bs-dispense");
						action->set_actor("R-"+std::to_string(model_robots[i]));
						action->set_id(action_id);
						action->add_parent_id(action_id-1);
						action->set_goal_id(data.orders(order_id).id());
						param = action->add_params();
						param->set_key("mps");
						param->set_value(node_names[1]);
						param = action->add_params();
						param->set_key("color");
						param->set_value(base_colors_inverted[data.orders(order_id).base_color()]);
						param = action->add_params();
						param->set_key("wp");
						param->set_value(wp);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-get");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[1]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value(wp);

					action_id_last_bs_robot[model_robots[i]] = action_id;

					break;

			case 5: // 5.Action : Prepare and mount sub_product with cap at CS

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					switch(desired_complexity) {
						case 0:
								action->add_parent_id(action_id_last[4]);
								break;
						case 1:
								action->add_parent_id(action_id_last[9]);
								break;
						case 2:
								action->add_parent_id(action_id_last[11]);
								break;
						case 3:
								action->add_parent_id(action_id_last[13]);
								break;
						default:
								break;
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare-cs");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[1]);
					action->add_parent_id(action_id_last[2]);
					action->add_parent_id(action_id_last[3]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("operation");
					param->set_value("MOUNT_CAP");

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					++action_id;
					action = plan->add_actions();
					action->set_name("cs-mount-cap");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("cap-color");
					param->set_value(cap_colors_inverted[data.orders(order_id).cap_color()]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					break;
			case 6:	// 6.Action : Deliver at DS

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[5]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[6]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare-ds");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[6]);
					param = action->add_params();
					param->set_key("gate");
					param->set_value("GATE-"+std::to_string(data.orders(order_id).delivery_gate()));

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[6]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					++action_id;
					switch(data.orders(order_id).complexity()) {
						case 0:
								action = plan->add_actions();
								action->set_name("fulfill-order-c0");
								action->set_actor("R-"+std::to_string(model_robots[i]));
								action->set_id(action_id);
								action->add_parent_id(action_id-1);
								action->set_goal_id(data.orders(order_id).id());
								param = action->add_params();
								param->set_key("mps");
								param->set_value(node_names[6]);
								param = action->add_params();
								param->set_key("base-color");
								param->set_value(base_colors_inverted[data.orders(order_id).base_color()]);
								param = action->add_params();
								param->set_key("cap-color");
								param->set_value(cap_colors_inverted[data.orders(order_id).cap_color()]);
								param = action->add_params();
								param->set_key("wp");
								param->set_value("WP1");
								break;
						case 1:
								action = plan->add_actions();
								action->set_name("fulfill-order-c1");
								action->set_actor("R-"+std::to_string(model_robots[i]));
								action->set_id(action_id);
								action->add_parent_id(action_id-1);
								action->set_goal_id(data.orders(order_id).id());
								param = action->add_params();
								param->set_key("mps");
								param->set_value(node_names[6]);
								param = action->add_params();
								param->set_key("base-color");
								param->set_value(base_colors_inverted[data.orders(order_id).base_color()]);
								param = action->add_params();
								param->set_key("ring1-color");
								param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
								param = action->add_params();
								param->set_key("cap-color");
								param->set_value(cap_colors_inverted[data.orders(order_id).cap_color()]);
								param = action->add_params();
								param->set_key("wp");
								param->set_value("WP1");
								break;
						case 2:
								action = plan->add_actions();
								action->set_name("fulfill-order-c2");
								action->set_actor("R-"+std::to_string(model_robots[i]));
								action->set_id(action_id);
								action->add_parent_id(action_id-1);
								action->set_goal_id(data.orders(order_id).id());
								param = action->add_params();
								param->set_key("mps");
								param->set_value(node_names[6]);
								param = action->add_params();
								param->set_key("base-color");
								param->set_value(base_colors_inverted[data.orders(order_id).base_color()]);
								param = action->add_params();
								param->set_key("ring1-color");
								param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
								param = action->add_params();
								param->set_key("ring2-color");
								param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(1)]);
								param = action->add_params();
								param->set_key("cap-color");
								param->set_value(cap_colors_inverted[data.orders(order_id).cap_color()]);
								param = action->add_params();
								param->set_key("wp");
								param->set_value("WP1");
								break;
						case 3:
								action = plan->add_actions();
								action->set_name("fulfill-order-c3");
								action->set_actor("R-"+std::to_string(model_robots[i]));
								action->set_id(action_id);
								action->add_parent_id(action_id-1);
								action->set_goal_id(data.orders(order_id).id());
								param = action->add_params();
								param->set_key("mps");
								param->set_value(node_names[6]);
								param = action->add_params();
								param->set_key("base-color");
								param->set_value(base_colors_inverted[data.orders(order_id).base_color()]);
								param = action->add_params();
								param->set_key("ring1-color");
								param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
								param = action->add_params();
								param->set_key("ring2-color");
								param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(1)]);
								param = action->add_params();
								param->set_key("ring3-color");
								param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(2)]);
								param = action->add_params();
								param->set_key("cap-color");
								param->set_value(cap_colors_inverted[data.orders(order_id).cap_color()]);
								param = action->add_params();
								param->set_key("wp");
								param->set_value("WP1");
								break;
						default:
								break;
					}

					break;

			case 7: // Feed base into RS as payment

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Only go to ring station
					// ...if the base was picked up (trivial because it is the same robot)
					action->add_parent_id(action_id_last_bs_robot[model_robots[i]]);
					// TODO Here we assume that a ring station is only used at maximum two times
					// ...if the corresponding ring station is already filled with 3 bases, wait for the feed action to occur in order to empty.
					// This feed action occurs due to the sequential form of the smt generated plan.
					if(model_positions[i]==7 && action_id_last_rs1_feed) {
						if(action_id_last_rs1_pay.size() >= 3) {
							action->add_parent_id(action_id_last_rs1_feed);
						}
					}
					else if(model_positions[i]==9 && action_id_last_rs2_feed) {
						if(action_id_last_rs2_pay.size() >= 3) {
							action->add_parent_id(action_id_last_rs2_feed);
						}
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put-slide-cc");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("slide");
					param->set_value("true");
					param = action->add_params();
					param->set_key("wp");
					if(model_holdA[i]<=3) {
						param->set_value(cap_carrier_colors[cap_colors_inverted[data.orders(order_id).cap_color()]]+std::to_string(cap_carrier_index));
					}
					else {
						param->set_value("WP2");
					}

					// Save action_id_last for feed at the correpsonding ring station
					if(model_positions[i] == 7) {
						action_id_last_rs1_pay.push_back(action_id);

						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						action_id_last_rs2_pay.push_back(action_id);

						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}

					break;

			case 8: // Action prepare RS and feed base for first ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Go to the ring station
					// ...if we picked up the base (trivial)
					action->add_parent_id(action_id_last_bs_robot[model_robots[i]]);
					// ...if the corresponding ring station is not blocked by any older necessary paying
					if(model_positions[i]==7) {
						for(unsigned int j=0; j<action_id_last_rs1_pay.size(); ++j) {
							if((int) j<rings_req_add_bases[rings[0]]) {
								action->add_parent_id(action_id_last_rs1_pay[j]);
							}
							else {
								std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 1." << std::endl;
							}
						}
					}
					else if(model_positions[i]==9) {
						for(unsigned int j=0; j<action_id_last_rs2_pay.size(); ++j) {
							if((int) j<rings_req_add_bases[rings[0]]) {
								action->add_parent_id(action_id_last_rs2_pay[j]);
							}
							else {
								std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 2." << std::endl;
							}
						}
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare-rs");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring_color");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
					if(model_positions[i] == 7) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					param = action->add_params();
					param->set_key("r-req");
					param->set_value(add_bases_description[rings_req_add_bases[rings[0]]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					++action_id;
					action = plan->add_actions();
					action->set_name("rs-mount-ring1");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring-color");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
					if(model_positions[i] == 7) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					param = action->add_params();
					param->set_key("r-req");
					param->set_value(add_bases_description[rings_req_add_bases[rings[0]]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");


					if(model_positions[i]==7) {
						action_id_last_rs1_feed = action_id;
					}
					else if(model_positions[i]==9){
						action_id_last_rs2_feed = action_id;
					}

					break;

			case 9: // Action retrieve base_ring at RS for first ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-get");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[8]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					if(model_positions[i]==8) {
						action_id_last_rs1_retr = action_id;
					}
					else if(model_positions[i]==10){
						action_id_last_rs2_retr = action_id;
					}

					break;

			case 10: // Action prepare RS and feed base for second ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Goto the ring station
					// ...if the base mounted with the first ring was picked up (trivial)
					action->add_parent_id(action_id_last[9]);
					// ...if the corresponding ring station is not blocked by feeding or any older paying
					// TODO Here we assume that one ring station is only used twice,
					// therefore it is sufficient to add all payment action ids here.
					// The ones which belonged to the first ring at this ringstation have been executed already
					// and ony the latest ones are important.
					// We could ONLY add the latest one if we compare index j with j>nrb[ro[0]] and j<nrb[ro[1]].
					if(model_positions[i]==7){
						for(unsigned int j=0; j<action_id_last_rs1_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs1_pay[j]);
						}
						if(action_id_last_rs1_feed) {
							action->add_parent_id(action_id_last_rs1_feed);
						}
					}
					else if(model_positions[i]==9){
						for(unsigned int j=0; j<action_id_last_rs2_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs2_pay[j]);
						}
						if(action_id_last_rs2_feed) {
							action->add_parent_id(action_id_last_rs2_feed);
						}
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare-rs");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring_color");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(1)]);
					if(model_positions[i] == 7) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					param = action->add_params();
					param->set_key("r-req");
					param->set_value(add_bases_description[rings_req_add_bases[rings[1]]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					++action_id;
					action = plan->add_actions();
					action->set_name("rs-mount-ring2");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring-color");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(1)]);
					if(model_positions[i] == 7) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					param = action->add_params();
					param->set_key("r-req");
					param->set_value(add_bases_description[rings_req_add_bases[rings[1]]]);
					param = action->add_params();
					param->set_key("col1");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					if(model_positions[i]==7) {
						action_id_last_rs1_feed = action_id;
					}
					else if(model_positions[i]==9){
						action_id_last_rs2_feed = action_id;
					}

					break;
			case 11: // Action retrieve base_ring at RS for second ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Go to the ring station output side if the corresponding ring station is not blocked by older retr
					if(model_positions[i]==8 && action_id_last_rs1_retr) {
						action->add_parent_id(action_id_last_rs1_retr);
					}
					else if(model_positions[i]==10 && action_id_last_rs2_retr) {
						action->add_parent_id(action_id_last_rs2_retr);
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-get");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[10]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					if(model_positions[i]==8) {
						action_id_last_rs1_retr = action_id;
					}
					else if(model_positions[i]==10){
						action_id_last_rs2_retr = action_id;
					}

					break;

			case 12: // Action prepare RS and feed base for third ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Goto the ring station
					// ...if the base mounted with the first ring was picked up
					action->add_parent_id(action_id_last[11]);
					// ...if the corresponding ring station is not blocked by feeding or any older paying
					if(model_positions[i]==7){
						for(unsigned int j=0; j<action_id_last_rs1_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs1_pay[j]);
						}
						if(action_id_last_rs1_feed) {
							action->add_parent_id(action_id_last_rs1_feed);
						}
					}
					else if(model_positions[i]==9){
						for(unsigned int j=0; j<action_id_last_rs2_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs2_pay[j]);
						}
						if(action_id_last_rs2_feed) {
							action->add_parent_id(action_id_last_rs2_feed);
						}
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare-rs");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring_color");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(2)]);
					if(model_positions[i] == 7) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					param = action->add_params();
					param->set_key("r-req");
					param->set_value(add_bases_description[rings_req_add_bases[rings[2]]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-put");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					++action_id;
					action = plan->add_actions();
					action->set_name("rs-mount-ring3");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring-color");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(2)]);
					if(model_positions[i] == 7) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					else if(model_positions[i] == 9) {
						param = action->add_params();
						param->set_key("rs-before");
						param->set_value(add_bases_description[model_insideA[i]]);
						param = action->add_params();
						param->set_key("rs-after");
						param->set_value(add_bases_description[model_insideB[i]]);
					}
					param = action->add_params();
					param->set_key("r-req");
					param->set_value(add_bases_description[rings_req_add_bases[rings[2]]]);
					param = action->add_params();
					param->set_key("col1");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(0)]);
					param = action->add_params();
					param->set_key("col2");
					param->set_value(rings_colors_inverted[data.orders(order_id).ring_colors(1)]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					if(model_positions[i]==7) {
						action_id_last_rs1_feed = action_id;
					}
					else if(model_positions[i]==9){
						action_id_last_rs2_feed = action_id;
					}

					break;
			case 13: // Action retrieve base_ring at RS for third ring

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Go to the ring station output side if the corresponding ring station is not blocked by older retr
					if(model_positions[i]==8 && action_id_last_rs1_retr) {
						action->add_parent_id(action_id_last_rs1_retr);
					}
					else if(model_positions[i]==10 && action_id_last_rs2_retr) {
						action->add_parent_id(action_id_last_rs2_retr);
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names[model_positions[i]]);
					j=i-1;
					search_for_old_position = true;
					// Case that robot was already used
					while(j>0 && search_for_old_position) {
						if(model_robots[i]==model_robots[j]) {
							param = action->add_params();
							param->set_key("from");
							param->set_value(node_names[model_positions[j]]);

							search_for_old_position = false;
						}
						j--;
					}
					// Case that robot was not used yet or we are in the first action
					if(search_for_old_position) {
						param = action->add_params();
						param->set_key("from");
						param->set_value(node_names[world_initPos[model_robots[i]]]);
					}

					++action_id;
					action = plan->add_actions();
					action->set_name("wp-get");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[12]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names[model_positions[i]]);
					param = action->add_params();
					param->set_key("wp");
					param->set_value("WP1");

					if(model_positions[i]==8) {
						action_id_last_rs1_retr = action_id;
					}
					else if(model_positions[i]==10){
						action_id_last_rs2_retr = action_id;
					}

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

	// Further initialization after init() depending on protobuf data
	clips_smt_clear_maps();
	clips_smt_init_game();

	// Strategy MACRO
	if(!data.strategy()){
		clips_smt_init_post();
		logger->log_info(name(), "Plan_horizon for macro approach is set to %i", plan_horizon_max);

		bool solve = true;
		if(world_initHold[1]==0 && world_initInside[1]==0 && world_initInside[2]==0 && world_initInside[3]==0 && world_initInside[4]==0 && world_initOutside[0]==0 && world_initOutside[1]==0 && world_initOutside[2]==0 && world_initOutside[3]==0 && world_initOutside[4]==0) {
			plan_horizon = plan_horizon_max;
		}
		else {
			plan_horizon = 1;
		}

		while(solve && plan_horizon < plan_horizon_max+5) {

			logger->log_info(name(), "%i iteration for macro approach", plan_horizon);

			// Declare formulas for encoding
			z3::expr_vector formula = clips_smt_encoder();

			// Pass it to z3 solver
			if(clips_smt_solve_formula(formula)) {
				solve = false;
			}
			plan_horizon++;
		}
	}
	// Strategy WINDOW
	else {
		plan_horizon = data.window();
		logger->log_info(name(), "Plan_horizon for window approach is set to %i", plan_horizon);

		// Declare formulas for encoding
		z3::expr_vector formula = clips_smt_encoder_window();

		// Pass it ot z3 optimizer
		clips_smt_optimize_formula(formula, "rew_");
	}

	logger->log_info(name(), "Thread reached end of loop");

	envs_[data_env].lock();
	envs_[data_env]->assert_fact_f("(smt-plan-complete \"%s\")", data_handle.c_str());
	envs_[data_env].unlock();
}

/**
 * #############################################################################
 * ############################## INITIALIZATION ###############################
 * #############################################################################
 *
 * init_game() extracts game and order relevant information
 * init_navgraph() extracts the distances between machines
 * init_post() determines the plan_horizon
 */

void
ClipsSmtThread::clips_smt_init_game()
{
	logger->log_info(name(), "clips_smt_init_game");

	if(init_game_once) {
		// Extract amount of robots
		amount_robots = data.robots().size();

		// Extract team
		team = "M";
		if(data.robots(0).team_color() == 0){
			team = "C";
		}

		// Extract how many bases are required for the corresponding colors
		for( int i=0; i<data.rings().size(); ++i ) {
			rings_req_add_bases[data.rings(i).ring_color()] = data.rings(i).raw_material();
		}

		// Extract station_colors ring_stations
		std::string cw_ringStation = team+"-RS1";
		std::string cw_capStation = team+"-CS1";
		for(int i=0; i<data.machines().size(); ++i) {
			if(cw_ringStation.compare(data.machines(i).name()) == 0){
				if(data.machines(i).ring_colors(0)==1 || data.machines(i).ring_colors(1)==1) {
					station_colors["R1"] = "RS1";
				}
				else {
					station_colors["R1"] = "RS2";
				}
				if(data.machines(i).ring_colors(0)==2 || data.machines(i).ring_colors(1)==2) {
					station_colors["R2"] = "RS1";
				}
				else {
					station_colors["R2"] = "RS2";
				}
				if(data.machines(i).ring_colors(0)==3 || data.machines(i).ring_colors(1)==3) {
					station_colors["R3"] = "RS1";
				}
				else {
					station_colors["R3"] = "RS2";
				}
				if(data.machines(i).ring_colors(0)==4 || data.machines(i).ring_colors(1)==4) {
					station_colors["R4"] = "RS1";
				}
				else {
					station_colors["R4"] = "RS2";
				}
				// continue;
			}
			else if(cw_capStation.compare(data.machines(i).name()) == 0){
				if(data.machines(i).cap_color()==1) {
					std::cout << "BLACK cap for CS1" << std::endl;
					station_colors["C1"] = "CS1";
					station_colors["C2"] = "CS2";
				}
				else {
					std::cout << "GREY cap for CS1" << std::endl;
					station_colors["C1"] = "CS2";
					station_colors["C2"] = "CS1";
				}
			}
		}

		// // Extract station_colors cap_stations with information from config
		// station_colors["C1"] = "CS2";
		// station_colors["C2"] = "CS1";
		// if(config->get_string("/clips-agent/rcll2016/cap-station/assigned-color/"+team+"-CS1").compare("BLACK")==0){
		//     station_colors["C1"] = "CS1";
		//     station_colors["C2"] = "CS2";
		// }

		clips_smt_init_navgraph();
		init_game_once = false;
	}

	// Extract order details
	for(int i=0; i<data.orders().size(); ++i) {
		rings.clear();
		desired_complexity = data.orders(i).complexity(); // Cover last given order
		order_id = i;
		base = data.orders(i).base_color();
		if(desired_complexity>0){
			rings.push_back(data.orders(i).ring_colors(0));
		}
		else {
			rings.push_back(1); // DUMMY
		}
		if(desired_complexity>1){
			rings.push_back(data.orders(i).ring_colors(1));
		}
		else {
			rings.push_back(1); // DUMMY
		}
		if(desired_complexity==3){
			rings.push_back(data.orders(i).ring_colors(2));
		}
		else {
			rings.push_back(1); // DUMMY
		}
		cap = data.orders(i).cap_color();
		delivery_period_begin = data.orders(i).delivery_period_begin();
		delivery_period_end = data.orders(i).delivery_period_end();
	}


	// Extract down/break, inside and output information of machine
	for(int i=0; i<data.machines().size(); ++i){
		// Extract which machines are down
		std::string var_down = "DOWN";
		std::string var_break = "BREAK";

		if(var_down.compare(data.machines(i).state().c_str()) == 0 || var_break.compare(data.machines(i).state().c_str()) == 0) {
			std::string machine_name = data.machines(i).name().c_str();
			machine_name += "-I";

			world_machines_down.push_back(node_names_inverted[machine_name]);
		}

		std::string cs1_name = team+"-CS1";
		std::string cs2_name = team+"-CS2";
		std::string rs1_name = team+"-RS1";
		std::string rs2_name = team+"-RS2";
		std::string bs_name = team+"-BS";

		// Extract add_bases of RS
		if(rs1_name.compare(data.machines(i).name()) == 0) {
			world_initInside[1] = add_bases_description_inverted[data.machines(i).loaded_with()];
			world_initOutside[1] = clips_smt_rewrite_product(data.machines(i).wp().base_color(), data.machines(i).wp().ring_colors(0), data.machines(i).wp().ring_colors(1), data.machines(i).wp().ring_colors(2), data.machines(i).wp().cap_color());
		}
		else if(rs2_name.compare(data.machines(i).name()) == 0) {
			world_initInside[2] = add_bases_description_inverted[data.machines(i).loaded_with()];
			world_initOutside[2] = clips_smt_rewrite_product(data.machines(i).wp().base_color(), data.machines(i).wp().ring_colors(0), data.machines(i).wp().ring_colors(1), data.machines(i).wp().ring_colors(2), data.machines(i).wp().cap_color());
		}
		// Extract inside of CS
		else if(cs1_name.compare(data.machines(i).name()) == 0) {
			world_initInside[3] = cap_colors[data.machines(i).cs_buffered()];
			world_initOutside[3] = clips_smt_rewrite_product(data.machines(i).wp().base_color(), data.machines(i).wp().ring_colors(0), data.machines(i).wp().ring_colors(1), data.machines(i).wp().ring_colors(2), data.machines(i).wp().cap_color());
		}
		else if(cs2_name.compare(data.machines(i).name()) == 0) {
			world_initInside[4] = cap_colors[data.machines(i).cs_buffered()];
			world_initOutside[4] = clips_smt_rewrite_product(data.machines(i).wp().base_color(), data.machines(i).wp().ring_colors(0), data.machines(i).wp().ring_colors(1), data.machines(i).wp().ring_colors(2), data.machines(i).wp().cap_color());
		}
		else if(cs2_name.compare(data.machines(i).name()) == 0) {
			world_initInside[4] = cap_colors[data.machines(i).cs_buffered()];
			world_initOutside[4] = clips_smt_rewrite_product(data.machines(i).wp().base_color(), data.machines(i).wp().ring_colors(0), data.machines(i).wp().ring_colors(1), data.machines(i).wp().ring_colors(2), data.machines(i).wp().cap_color());
		}
		else if(bs_name.compare(data.machines(i).name()) == 0) {
			world_initInside[0] = 0;
			world_initOutside[0] = clips_smt_rewrite_product(data.machines(i).wp().base_color(), data.machines(i).wp().ring_colors(0), data.machines(i).wp().ring_colors(1), data.machines(i).wp().ring_colors(2), data.machines(i).wp().cap_color());
		}
	}

	// Extract holding and location information of robots
	for(int i=0; i<data.robots().size(); ++i){
		world_initPos[i+1] = node_names_inverted[data.robots(i).location()];
		world_initHold[i+1] = clips_smt_rewrite_product(data.robots(i).wp().base_color(), data.robots(i).wp().ring_colors(0), data.robots(i).wp().ring_colors(1), data.robots(i).wp().ring_colors(2), data.robots(i).wp().cap_color());
	}
}

void
ClipsSmtThread::graph_changed() throw()
{
	// wakeup();
}

void
ClipsSmtThread::clips_smt_init_navgraph()
{
	// Navgraph
	node_names[0] = "START-I"; // team+"-ins-in";
	node_names[1] = team+"-BS-O";
	node_names[2] = team+"-CS1-I";
	node_names[3] = team+"-CS1-O";
	node_names[4] = team+"-CS2-I";
	node_names[5] = team+"-CS2-O";
	node_names[6] = team+"-DS-I";
	node_names[7] = team+"-RS1-I";
	node_names[8] = team+"-RS1-O";
	node_names[9] = team+"-RS2-I";
	node_names[10] = team+"-RS2-O";

	node_names_inverted["START-I"] = 0; // team+"-ins-in"] = 0;
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

	logger->log_info(name(), "clips_smt_init_navgraph");

	MutexLocker lock(navgraph.objmutex_ptr());

	// Compute distances between unconnected C-ins-in and all other machines
	NavGraphNode ins_node(navgraph->node(team+"-ins-in"));
	NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

	for (unsigned int i = 1; i < node_names.size(); ++i) {
		std::pair<std::string, std::string> nodes_pair(ins_node.name(), node_names[i]);

		NavGraphNode to = navgraph->node(node_names[i]);
		NavGraphPath p = navgraph->search_path(from, to);

		distances[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
	}

	// Compute distances between machines
	for (unsigned int i = 1; i < node_names.size(); ++i) {
		for (unsigned int j = 1; j < node_names.size(); ++j) {
			if (i == j) continue;
			std::pair<std::string, std::string> nodes_pair(node_names[i], node_names[j]);

			NavGraphPath p = navgraph->search_path(node_names[i], node_names[j]);
			distances[nodes_pair] = p.cost();
		}
	}
}

void
ClipsSmtThread::clips_smt_init_post()
{
	logger->log_info(name(), "clips_smt_init_post");

	// Set PlanHorizon
	switch(desired_complexity) {
		case 0:
				plan_horizon_max = amount_min_req_actions[0];
				break;
		case 1:
				plan_horizon_max = amount_min_req_actions[1]
									+ amount_req_actions_add_bases*rings_req_add_bases[rings[0]]
									- amount_req_actions_add_bases;
				break;
		case 2:
				plan_horizon_max = amount_min_req_actions[2]
									+ amount_req_actions_add_bases*rings_req_add_bases[rings[0]]
									+ amount_req_actions_add_bases*rings_req_add_bases[rings[1]]
									- amount_req_actions_add_bases;
				break;
		case 3:
				plan_horizon_max = amount_min_req_actions[3]
									+ amount_req_actions_add_bases*rings_req_add_bases[rings[0]]
									+ amount_req_actions_add_bases*rings_req_add_bases[rings[1]]
									+ amount_req_actions_add_bases*rings_req_add_bases[rings[2]]
									- amount_req_actions_add_bases;
				break;
		default:
				std::cout << "Wrong desired_complexity " << desired_complexity << " for determining plan_horizon" << std::endl;
				break;
	}
}

/**
 * #############################################################################
 * ############################## ENCODER ######################################
 * #############################################################################
 *
 * encoder() returns a z3 formula which specifies the planning problem of the game for a complete plan
 * encoder_window() returns a z3 formula which specifies the planning problem of the game for a fixed plan_horizon
 */

/**
 * ENCODER
 */

z3::expr_vector
ClipsSmtThread::clips_smt_encoder()
{
	logger->log_info(name(), "clips_smt_encoder");

	/*
	 * PRECOMPUTATION
	 */

	// Map collecting all variables
	std::map<std::string, z3::expr> var;
	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);

	// Init variable true and false
	z3::expr var_false(_z3_context.bool_val(false));
	z3::expr var_true(_z3_context.bool_val(true));


	/*
	 * VARIABLES
	 */

	// Variables initDist_i_j
	for(int i = 0; i < amount_machines+1; ++i){
		for(int j = i+1; j < amount_machines+1; ++j) {
			var.insert(std::make_pair("initDist_" + std::to_string(i) + "_" + std::to_string(j), _z3_context.real_const(("initDist_" + std::to_string(i) + "_" + std::to_string(j)).c_str())));
		}
	}

	// Variables initPos and initHold
	for(int i = 1; i < amount_robots+1; ++i){
		var.insert(std::make_pair("initPos_" + std::to_string(i), _z3_context.int_const(("initPos_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initHold_" + std::to_string(i), _z3_context.int_const(("initHold_" + std::to_string(i)).c_str())));
	}

	// Variables initState1_i, initInside_i and initOutside_i
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		var.insert(std::make_pair("initInside_" + std::to_string(i), _z3_context.int_const(("initInside_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initOutside_" + std::to_string(i), _z3_context.int_const(("initOutside_" + std::to_string(i)).c_str())));
	}

	// Variables depending on plan_horizon
	for(int i=1; i<plan_horizon+1; ++i){
		var.insert(std::make_pair("t_" + std::to_string(i), _z3_context.real_const(("t_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("rd_" + std::to_string(i), _z3_context.real_const(("rd_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("pos_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(i)).c_str())));
		for(int r=1; r<amount_robots+1; ++r){
			var.insert(std::make_pair("pos_" + std::to_string(r) + "_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(r) + "_" + std::to_string(i)).c_str())));
		}
		var.insert(std::make_pair("md_" + std::to_string(i), _z3_context.real_const(("md_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("R_" + std::to_string(i), _z3_context.int_const(("R_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("A_" + std::to_string(i), _z3_context.int_const(("A_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("M_" + std::to_string(i), _z3_context.int_const(("M_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("holdA_" + std::to_string(i), _z3_context.int_const(("holdA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("insideA_" + std::to_string(i), _z3_context.int_const(("insideA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputA_" + std::to_string(i), _z3_context.int_const(("outputA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("holdB_" + std::to_string(i), _z3_context.int_const(("holdB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("insideB_" + std::to_string(i), _z3_context.int_const(("insideB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputB_" + std::to_string(i), _z3_context.int_const(("outputB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("rew_" + std::to_string(i), _z3_context.real_const(("rew_" + std::to_string(i)).c_str())));
	}


	/*
	 * CONSTRAINTS
	 */

	// Constraints depending on plan_horizon
	for(int i = 1; i < plan_horizon+1; ++i){

		// VarStartTime
		// General bound
		constraints.push_back(0 <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "t_"+std::to_string(i)) <= 900);
		// Robot specifc bound
		for(int j = 1; j < i; ++j){
			constraints.push_back(!(getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i))) || getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)));
		}

		constraints.push_back(0 <= getVar(var, "rd_"+std::to_string(i))); // VarRobotDuration
		constraints.push_back(1 <= getVar(var, "pos_"+std::to_string(i)) && getVar(var, "pos_"+std::to_string(i)) <= amount_machines); // VarRobotPosition
		for(int r=1; r<amount_robots+1; ++r){
			constraints.push_back(0 <= getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) && getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) <= amount_machines); // VarRobotPosition
		}
		constraints.push_back(0 <= getVar(var, "md_"+std::to_string(i))); // VarMachineDuration
		constraints.push_back(1 <= getVar(var, "R_"+std::to_string(i)) && getVar(var, "R_"+std::to_string(i)) <= amount_robots); // VarR
		constraints.push_back(1 <= getVar(var, "A_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(i)) <= index_upper_bound_actions[desired_complexity]); // VarA
		constraints.push_back(min_machine_groups <= getVar(var, "M_"+std::to_string(i)) && getVar(var, "M_"+std::to_string(i)) <= max_machine_groups); // VarM

		constraints.push_back(min_products <= getVar(var, "holdA_"+std::to_string(i)) && getVar(var, "holdA_"+std::to_string(i)) <= max_products); // VarHoldA
		constraints.push_back(0 <= getVar(var, "insideA_"+std::to_string(i)));
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"]) || getVar(var, "insideA_"+std::to_string(i)) <= 0);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
			|| getVar(var, "insideA_"+std::to_string(i)) <= max_inside_capstation);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"])
			|| getVar(var, "insideA_"+std::to_string(i)) <= max_add_bases_ringstation);
		constraints.push_back(min_products <= getVar(var, "outputA_"+std::to_string(i)) && getVar(var, "outputA_"+std::to_string(i)) <= max_products); // VarOutsideA

		constraints.push_back(min_products <= getVar(var, "holdB_"+std::to_string(i)) && getVar(var, "holdB_"+std::to_string(i)) <= max_products); // VarHoldB
		constraints.push_back(0 <= getVar(var, "insideB_"+std::to_string(i)));
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"]) || getVar(var, "insideB_"+std::to_string(i)) <= 0);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
			|| getVar(var, "insideB_"+std::to_string(i)) <= max_inside_capstation);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"])
			|| getVar(var, "insideB_"+std::to_string(i)) <= max_add_bases_ringstation);
		constraints.push_back(min_products <= getVar(var, "outputB_"+std::to_string(i)) && getVar(var, "outputB_"+std::to_string(i)) <= max_products); // VarOutsideB
	}

	// Constraint: robot states are initially consistent
	for(int i=1; i<amount_robots+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "R_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "R_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(var_false);
			for(int k=0; k<amount_machines+1; ++k){
				for(int l=1; l<amount_machines+1; ++l){
					if(k<l){
						constraint2 = constraint2 || (getVar(var, "initPos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)));
					}
					else if(l<k){
						constraint2 = constraint2 || (getVar(var, "initPos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "initDist_"+std::to_string(l)+"_"+std::to_string(k)));
					}
					else {
						constraint2 = constraint2 || (getVar(var, "initPos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=0);
					}
				}
			}

			constraints.push_back(constraint1 || (getVar(var, "holdA_"+std::to_string(ip))==getVar(var, "initHold_"+std::to_string(i)) && constraint2));
		}
	}

	// Constraint: robot states are inductively consistent
	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "R_"+std::to_string(ip)) == getVar(var, "R_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "R_"+std::to_string(ipp))==getVar(var, "R_"+std::to_string(i));
			}

			z3::expr constraint2(var_false);
			for(int k=1; k<amount_machines+1; ++k){
				for(int l=1; l<amount_machines+1; ++l){
					if(k<l){
						constraint2 = constraint2 || (getVar(var, "pos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "t_"+std::to_string(i))+getVar(var, "rd_"+std::to_string(ip))+getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)));
					}
					else if(l<k){
						constraint2 = constraint2 || (getVar(var, "pos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "t_"+std::to_string(i))+getVar(var, "rd_"+std::to_string(ip))+getVar(var, "initDist_"+std::to_string(l)+"_"+std::to_string(k)));
					}
					else {
						constraint2 = constraint2 || (getVar(var, "pos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "t_"+std::to_string(i))+getVar(var, "rd_"+std::to_string(ip)));
					}
				}
			}

			constraints.push_back(constraint1 || (getVar(var, "holdA_"+std::to_string(ip))==getVar(var, "holdB_"+std::to_string(i)) && constraint2));
		}
	}

	// Constraint: robots can not occupy the same position in the same action step
	for(int i=1; i<plan_horizon+1; ++i){
		for(int r=1; r<amount_robots+1; ++r){

			// If robot r is acting in step i then set his position to the position in step i
			z3::expr constraint_precondition( getVar(var, "R_"+std::to_string(i))==r );
			z3::expr constraint_effect( getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(i)) );

			for(int rp=1; rp<amount_robots+1; ++rp){
				if(r!=rp){
					if(i==1){
						constraint_effect = constraint_effect && getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == getVar(var, "initPos_"+std::to_string(rp));
					}
					else {
						constraint_effect = constraint_effect && getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i-1));
					}
				}
			}

			constraints.push_back( !(constraint_precondition) || constraint_effect);

			// All robot's positions must exclude each other
			for(int rp=r+1; rp<amount_robots+1; ++rp){
				constraints.push_back( !(getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)))
			 							|| ( getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == 0
											&& getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == 0) );
			}
		}
	}

	// Constraint: machine states are initially consistent
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "M_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "M_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(getVar(var, "insideA_"+std::to_string(ip))==getVar(var, "initInside_"+std::to_string(i))
								&& getVar(var, "outputA_"+std::to_string(ip))==getVar(var, "initOutside_"+std::to_string(i)));

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Constraint: machine states are inductively consistent
	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "M_"+std::to_string(ip)) == getVar(var, "M_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || (getVar(var, "M_"+std::to_string(ipp)) == getVar(var, "M_"+std::to_string(i)));
			}

			z3::expr constraint2(getVar(var, "t_"+std::to_string(ip))>=getVar(var, "t_"+std::to_string(i))+getVar(var, "md_"+std::to_string(i))
									&& getVar(var, "insideB_"+std::to_string(i))==getVar(var, "insideA_"+std::to_string(ip))
									&& getVar(var, "outputB_"+std::to_string(i))==getVar(var, "outputA_"+std::to_string(ip)));

			constraints.push_back(constraint1 || constraint2);

		}
	}

	/**
	 * ADDITIONAL CONSTRAINTS
	 */

	// Constraints to fix robot order
	// Start with R-1
	constraints.push_back(getVar(var, "R_1") == 1);

	// R-3 is chosen if R-2 has been chosen before
	for(int i=2; i<plan_horizon+1; ++i) {
		z3::expr constraint_r2_used(var_false);

		for(int j=2; j<i; ++j) {
			constraint_r2_used = constraint_r2_used || getVar(var, "R_"+std::to_string(j)) == 2;
		}

		constraints.push_back(!(getVar(var, "R_"+std::to_string(i)) == 3) || constraint_r2_used);
	}

	// Constraint: for some actions the robot is fixed
	for(int i=1; i<plan_horizon+1; ++i) {
		for(int j=1; j<i; ++j) {
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 1 && getVar(var, "A_"+std::to_string(i)) == 2 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 9 && getVar(var, "A_"+std::to_string(i)) == 10 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 11 && getVar(var, "A_"+std::to_string(i)) == 12 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 13 && getVar(var, "A_"+std::to_string(i)) == 4 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
		}
	}

	if(plan_horizon>=plan_horizon_max) {
		// Constraint: every action depends on actions to happen before
		for(int i=1; i<plan_horizon+1; ++i) {

			// Action x appears
			z3::expr constraint_dependency1(var_false);
			z3::expr constraint_dependency2(var_false);
			z3::expr constraint_dependency3(var_false);
			z3::expr constraint_dependency4(var_false);
			z3::expr constraint_dependency5(var_false);
			z3::expr constraint_dependency6(var_false);
			z3::expr constraint_dependency7(var_false);
			z3::expr constraint_dependency8(var_false);
			z3::expr constraint_dependency9(var_false);
			z3::expr constraint_dependency10(var_false);
			z3::expr constraint_dependency11(var_false);
			z3::expr constraint_dependency12(var_false);
			z3::expr constraint_dependency13(var_false);

			for(int j=1; j<i; ++j) {
				constraint_dependency1 = constraint_dependency1 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 1);
				constraint_dependency2 = constraint_dependency2 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 2);
				constraint_dependency3 = constraint_dependency3 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 3);
				constraint_dependency4 = constraint_dependency4 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 4);
				constraint_dependency5 = constraint_dependency5 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 5);
				constraint_dependency6 = constraint_dependency6 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 6);
				constraint_dependency7 = constraint_dependency7 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 7);
				constraint_dependency8 = constraint_dependency8 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 8);
				constraint_dependency9 = constraint_dependency9 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 9);
				constraint_dependency10 = constraint_dependency10 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 10);
				constraint_dependency11 = constraint_dependency11 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 11);
				constraint_dependency12 = constraint_dependency12 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 12);
				constraint_dependency13 = constraint_dependency13 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 13);
			}

			// Action y has dependency on actions x1,...,xn
			z3::expr constraint_inter2(constraint_dependency1);
			z3::expr constraint_inter3(constraint_dependency2);
			z3::expr constraint_inter7(constraint_dependency4);
			z3::expr constraint_inter8(constraint_dependency4);
			z3::expr constraint_inter9(constraint_dependency8);
			z3::expr constraint_inter10(constraint_dependency9);
			z3::expr constraint_inter11(constraint_dependency10);
			z3::expr constraint_inter12(constraint_dependency11);
			z3::expr constraint_inter13(constraint_dependency12);
			z3::expr constraint_inter5(constraint_dependency1 && constraint_dependency2 && constraint_dependency3);
			switch (desired_complexity) {
				case 0:
						constraint_inter5 = constraint_inter5 && constraint_dependency4;
						break;
				case 1:
						constraint_inter5 = constraint_inter5 && constraint_dependency9;
						break;
				case 2:
						constraint_inter5 = constraint_inter5 && constraint_dependency11;
						break;
				case 3:
						constraint_inter5 = constraint_inter5 && constraint_dependency13;
						break;
			}
			z3::expr constraint_inter6(constraint_dependency5);

			// Push constraint if action = y then dependencies of y must be fulfilled
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_inter2);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 3) || constraint_inter3);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || constraint_inter5);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || constraint_inter6);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 7) || constraint_inter7);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 8) || constraint_inter8);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 9) || constraint_inter9);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 10) || constraint_inter10);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 11) || constraint_inter11);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 12) || constraint_inter12);
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 13) || constraint_inter13);
		}
	}

	// Constraint: every action is encoded for every order
	// Save ids of order_colors as strings
	std::string base_str = std::to_string(base);
	std::string ring1_order_str = std::to_string(rings[0]);
	std::string ring2_order_str = std::to_string(rings[1]);
	std::string ring3_order_str = std::to_string(rings[2]);
	std::string cap_str = std::to_string(cap);

	// Init string identifiers for state maps
	std::string bi = "B" + base_str;
	std::string r1i = "R" + ring1_order_str;
	std::string r2i = "R" + ring2_order_str;
	std::string r3i = "R" + ring3_order_str;
	std::string ci = "C" + cap_str;

	// Construct combined string identifiers
	std::string br_ci = "BR" + ci;
	std::string bi_ci = bi + ci;
	std::string bi_r1i = bi + r1i;
	std::string bi_r1i_ci = bi_r1i + ci;
	std::string bi_r1i_r2i = bi_r1i + r2i;
	std::string bi_r1i_r2i_ci = bi_r1i_r2i + ci;
	std::string bi_r1i_r2i_r3i = bi_r1i_r2i + r3i;
	std::string bi_r1i_r2i_r3i_ci = bi_r1i_r2i_r3i + ci;

	std::string has_ci = "has_" + ci;

	std::string sub_product;
	std::string product;
	switch(desired_complexity) {
		case 0:
				sub_product = bi;
				product = bi_ci;
				break;
		case 1:
				sub_product = bi_r1i;
				product = bi_r1i_ci;
				break;
		case 2:
				sub_product = bi_r1i_r2i;
				product = bi_r1i_r2i_ci;
				break;
		case 3:
				sub_product = bi_r1i_r2i_r3i;
				product = bi_r1i_r2i_r3i_ci;
				break;
		default:
				std::cout << "Wrong desired_complexity " << desired_complexity << " to set sub_product and product" << std::endl;
				break;
	}

	// For every step up to the plan_horizon add all req actions depending on the order complexity
	for(int i=1; i<plan_horizon+1; ++i){

		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions all complexities require (which correspond to complexity 0)
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */

		// 1.Action: Retrieve cap_carrier_cap from CS-Shelf
		z3::expr constraintaction1((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[ci]])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
									&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[ci] + "-I"])
									&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products[br_ci])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_1) || constraintaction1);

		// 2.Action : Prepare and feed CS for RETRIEVE with cap_carrier
		z3::expr constraintaction2((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
									&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
									&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "outputB_"+std::to_string(i)) == products["BR"])
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"]) || getVar(var, "pos_"+std::to_string(i)) == 2)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"]) || getVar(var, "pos_"+std::to_string(i)) == 4)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products[br_ci])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_2) || constraintaction2);

		// 3.Action : Retrieve cap_carrier at CS
		z3::expr constraintaction3((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputA_"+std::to_string(i)) > 0) // Must be higher than 0 in order to retrieve someting as no prepare is intended here
									&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_disc)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"]) || getVar(var, "pos_"+std::to_string(i)) == 3)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"]) || getVar(var, "pos_"+std::to_string(i)) == 5)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "holdB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_3) || constraintaction3);

		// 4.Action : Prepare and retrieve base from BS
		z3::expr constraintaction4((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"]) // outputA is bound by the init or the predecessor
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
									&& (getVar(var, "pos_"+std::to_string(i)) == 1)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
									&& (!(getVar(var, "outputA_"+std::to_string(i)) == 0) || getVar(var, "holdB_"+std::to_string(i)) == products[bi])
									&& (!(getVar(var, "outputA_"+std::to_string(i)) > 0) || getVar(var, "holdB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_4) || constraintaction4);

		// 5.Action : Prepare and mount sub_product with cap at CS
		z3::expr constraintaction5((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
									&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
									&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
									&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "outputB_"+std::to_string(i)) == products[product])
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed) // TODO adapt the time
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"]) || getVar(var, "pos_"+std::to_string(i)) == 2)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"]) || getVar(var, "pos_"+std::to_string(i)) == 4)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products[sub_product])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_5) || constraintaction5);

		// 6.Action : Hold product and deliver at DS
		z3::expr constraintaction6((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"]) // TODO own group?
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
									&& (getVar(var, "pos_"+std::to_string(i)) == 6)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products[product])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_6) || constraintaction6);

		// 7.Action : Feed additional base into RS for payment and points
		z3::expr constraintaction7((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))+1)
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_feed)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"]) || getVar(var, "pos_"+std::to_string(i)) == 7)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"]) || getVar(var, "pos_"+std::to_string(i)) == 9)
									&& (getVar(var, "holdA_"+std::to_string(i)) > 2)
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_7) || constraintaction7);

		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions complexities 1,2 and 3 require
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */
		if(desired_complexity > 0) {


			// 8.Action : Prepare and mount base with ring1 at RS
			z3::expr constraintaction8((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r1i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))-rings_req_add_bases[rings[0]])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r1i] + "-I"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_8) || constraintaction8);

			// 9.Action : Retrieve base_ring1 from RS
			z3::expr constraintaction9((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r1i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r1i] + "-O"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_9) || constraintaction9);
		}
		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions complexities 2 and 3 require
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */
		if(desired_complexity > 1) {

			// 10.Action : Prepare and mount base_ring1 with ring2 at RS
			z3::expr constraintaction10((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r2i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))-rings_req_add_bases[rings[1]])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r2i] + "-I"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_10) || constraintaction10);

			// 11.Action : Retrieve base_ring1_ring2 from RS
			z3::expr constraintaction11((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r2i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r2i] + "-O"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_11) || constraintaction11);
		}
		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions complexity 3 requires
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */
		if(desired_complexity == 3) {

			// 12.Action : Prepare and mount base_ring1_ring2 with ring3 at RS
			z3::expr constraintaction12((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r3i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))-rings_req_add_bases[rings[2]])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r3i] + "-I"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_12) || constraintaction12);

			// 13.Action : Retrieve base_ring1_ring2_ring3 from RS
			z3::expr constraintaction13((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r3i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r3i] + "-O"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_13) || constraintaction13);
		}
	}

	// // Specify goal state for OMT
	// for(int i=1; i<plan_horizon+1; ++i){
	//     if(i==1){
	//         constraints.push_back((getVar(var, "A_"+std::to_string(i)) == index_delivery_action
	//                                     && getVar(var, "rew_"+std::to_string(i)) == (deadline-getVar(var, "t_"+std::to_string(i))-getVar(var, "md_"+std::to_string(i))))
	//                                 || (!(getVar(var, "A_"+std::to_string(i)) == index_delivery_action)
	//                                     && getVar(var, "rew_"+std::to_string(i))==0));
	//     }
	//     else {
	//         constraints.push_back((getVar(var, "A_"+std::to_string(i)) == index_delivery_action
	//                                     && getVar(var, "rew_"+std::to_string(i)) == (getVar(var, "rew_"+std::to_string(i-1))+deadline-getVar(var, "t_"+std::to_string(i))-getVar(var, "md_"+std::to_string(i))))
	//                                 || (!(getVar(var, "A_"+std::to_string(i)) == index_delivery_action)
	//                                     && getVar(var, "rew_"+std::to_string(i))==getVar(var, "rew_"+std::to_string(i-1))));
	//     }
	// }

	// Constraints encoding that final_actions for each order have to be at least executed once for SMT
	z3::expr constraint_goal(getVar(var, "A_"+std::to_string(plan_horizon)) == index_delivery_action); //  && getVar(var, "R_"+std::to_string(plan_horizon)) == 1);
	if(consider_temporal_constraint){
		constraints.push_back(!constraint_goal ||
								(getVar(var, "t_"+std::to_string(plan_horizon)) < (int) delivery_period_end
								&& getVar(var, "t_"+std::to_string(plan_horizon)) > (int) delivery_period_begin));
	}
	constraints.push_back(constraint_goal);

	// Specify initial situation for robots
	for(int i=1; i<amount_robots+1; ++i){
		constraints.push_back(getVar(var, "initHold_"+std::to_string(i)) == world_initHold[i]);
		constraints.push_back(getVar(var, "initPos_"+std::to_string(i)) == world_initPos[i]);
	}

	// Specify initial situation for machines
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		constraints.push_back(getVar(var, "initInside_"+std::to_string(i)) == world_initInside[i]);
		constraints.push_back(getVar(var, "initOutside_"+std::to_string(i)) == world_initOutside[i]);
	}


	// Specify positions which are down and therefore not useable
	z3::expr constraint_world_machine_down(var_true);
	for(int i=1; i<plan_horizon+1; ++i){

		for(int world_machine_down: world_machines_down) {

			// Distinguish between action 1 which operates on the input side but is still valid even if the corresponding cap station is down and other actions
			constraint_world_machine_down = constraint_world_machine_down && ( getVar(var, "A_"+std::to_string(i)) == 1 || getVar(var, "pos_"+std::to_string(i)) != world_machine_down);
		}
	}
	constraints.push_back(constraint_world_machine_down);

	// Specify distances between machines
	for(int k=0; k<amount_machines+1; ++k){
		for(int l=k+1; l<amount_machines+1; ++l){
			float distance = velocity_scaling_ * distances[std::make_pair(node_names[k], node_names[l])];
			z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
			constraints.push_back(getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)) == distance_z3);
		}
	}

	return constraints;
}

/**
 * ENCODER_WINDOW
 */

z3::expr_vector
ClipsSmtThread::clips_smt_encoder_window()
{
	logger->log_info(name(), "clips_smt_encoder_window");

	/*
	 * PRECOMPUTATION
	 */

	// Map collecting all variables
	std::map<std::string, z3::expr> var;
	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);

	// Init variable true and false
	z3::expr var_false(_z3_context.bool_val(false));
	z3::expr var_true(_z3_context.bool_val(true));


	/*
	 * VARIABLES
	 */

	// Variables initDist_i_j
	for(int i = 0; i < amount_machines+1; ++i){
		for(int j = i+1; j < amount_machines+1; ++j) {
			var.insert(std::make_pair("initDist_" + std::to_string(i) + "_" + std::to_string(j), _z3_context.real_const(("initDist_" + std::to_string(i) + "_" + std::to_string(j)).c_str())));
		}
	}

	// Variables initPos and initHold
	for(int i = 1; i < amount_robots+1; ++i){
		var.insert(std::make_pair("initPos_" + std::to_string(i), _z3_context.int_const(("initPos_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initHold_" + std::to_string(i), _z3_context.int_const(("initHold_" + std::to_string(i)).c_str())));
	}

	// Variables initState1_i, initInside_i and initOutside_i
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		var.insert(std::make_pair("initInside_" + std::to_string(i), _z3_context.int_const(("initInside_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initOutside_" + std::to_string(i), _z3_context.int_const(("initOutside_" + std::to_string(i)).c_str())));
	}

	// Variables depending on plan_horizon
	for(int i=1; i<plan_horizon+1; ++i){
		var.insert(std::make_pair("t_" + std::to_string(i), _z3_context.real_const(("t_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("rd_" + std::to_string(i), _z3_context.real_const(("rd_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("pos_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(i)).c_str())));
		for(int r=1; r<amount_robots+1; ++r){
			var.insert(std::make_pair("pos_" + std::to_string(r) + "_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(r) + "_" + std::to_string(i)).c_str())));
		}
		var.insert(std::make_pair("md_" + std::to_string(i), _z3_context.real_const(("md_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("R_" + std::to_string(i), _z3_context.int_const(("R_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("A_" + std::to_string(i), _z3_context.int_const(("A_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("M_" + std::to_string(i), _z3_context.int_const(("M_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("holdA_" + std::to_string(i), _z3_context.int_const(("holdA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("insideA_" + std::to_string(i), _z3_context.int_const(("insideA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputA_" + std::to_string(i), _z3_context.int_const(("outputA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("holdB_" + std::to_string(i), _z3_context.int_const(("holdB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("insideB_" + std::to_string(i), _z3_context.int_const(("insideB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputB_" + std::to_string(i), _z3_context.int_const(("outputB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("rew_" + std::to_string(i), _z3_context.real_const(("rew_" + std::to_string(i)).c_str())));
	}


	/*
	 * CONSTRAINTS
	 */

	// Constraints depending on plan_horizon
	for(int i = 1; i < plan_horizon+1; ++i){

		// VarStartTime
		// General bound
		constraints.push_back(0 <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "t_"+std::to_string(i)) <= 900);
		// Robot specifc bound
		for(int j = 1; j < i; ++j){
			constraints.push_back(!(getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i))) || getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)));
		}

		constraints.push_back(0 <= getVar(var, "rd_"+std::to_string(i))); // VarRobotDuration
		constraints.push_back(1 <= getVar(var, "pos_"+std::to_string(i)) && getVar(var, "pos_"+std::to_string(i)) <= amount_machines); // VarRobotPosition
		for(int r=1; r<amount_robots+1; ++r){
			constraints.push_back(0 <= getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) && getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) <= amount_machines); // VarRobotPosition
		}
		constraints.push_back(0 <= getVar(var, "md_"+std::to_string(i))); // VarMachineDuration
		constraints.push_back(1 <= getVar(var, "R_"+std::to_string(i)) && getVar(var, "R_"+std::to_string(i)) <= amount_robots); // VarR
		constraints.push_back(0 <= getVar(var, "A_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(i)) <= index_upper_bound_actions[desired_complexity]); // VarA
		constraints.push_back(min_machine_groups <= getVar(var, "M_"+std::to_string(i)) && getVar(var, "M_"+std::to_string(i)) <= max_machine_groups); // VarM

		constraints.push_back(min_products <= getVar(var, "holdA_"+std::to_string(i)) && getVar(var, "holdA_"+std::to_string(i)) <= max_products); // VarHoldA
		constraints.push_back(0 <= getVar(var, "insideA_"+std::to_string(i)));
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"]) || getVar(var, "insideA_"+std::to_string(i)) <= 0);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
			|| getVar(var, "insideA_"+std::to_string(i)) <= max_inside_capstation);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"])
			|| getVar(var, "insideA_"+std::to_string(i)) <= max_add_bases_ringstation);
		constraints.push_back(min_products <= getVar(var, "outputA_"+std::to_string(i)) && getVar(var, "outputA_"+std::to_string(i)) <= max_products); // VarOutsideA

		constraints.push_back(min_products <= getVar(var, "holdB_"+std::to_string(i)) && getVar(var, "holdB_"+std::to_string(i)) <= max_products); // VarHoldB
		constraints.push_back(0 <= getVar(var, "insideB_"+std::to_string(i)));
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"]) || getVar(var, "insideB_"+std::to_string(i)) <= 0);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
			|| getVar(var, "insideB_"+std::to_string(i)) <= max_inside_capstation);
		constraints.push_back(!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"])
			|| getVar(var, "insideB_"+std::to_string(i)) <= max_add_bases_ringstation);
		constraints.push_back(min_products <= getVar(var, "outputB_"+std::to_string(i)) && getVar(var, "outputB_"+std::to_string(i)) <= max_products); // VarOutsideB
	}

	// Constraint: robot states are initially consistent
	for(int i=1; i<amount_robots+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "R_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "R_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(var_false);
			for(int k=0; k<amount_machines+1; ++k){
				for(int l=1; l<amount_machines+1; ++l){
					if(k<l){
						constraint2 = constraint2 || (getVar(var, "initPos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)));
					}
					else if(l<k){
						constraint2 = constraint2 || (getVar(var, "initPos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "initDist_"+std::to_string(l)+"_"+std::to_string(k)));
					}
					else {
						constraint2 = constraint2 || (getVar(var, "initPos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=0);
					}
				}
			}

			constraints.push_back(constraint1 || (getVar(var, "holdA_"+std::to_string(ip))==getVar(var, "initHold_"+std::to_string(i)) && constraint2));
		}
	}

	// Constraint: robot states are inductively consistent
	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "R_"+std::to_string(ip)) == getVar(var, "R_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "R_"+std::to_string(ipp))==getVar(var, "R_"+std::to_string(i));
			}

			z3::expr constraint2(var_false);
			for(int k=1; k<amount_machines+1; ++k){
				for(int l=1; l<amount_machines+1; ++l){
					if(k<l){
						constraint2 = constraint2 || (getVar(var, "pos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "t_"+std::to_string(i))+getVar(var, "rd_"+std::to_string(ip))+getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)));
					}
					else if(l<k){
						constraint2 = constraint2 || (getVar(var, "pos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "t_"+std::to_string(i))+getVar(var, "rd_"+std::to_string(ip))+getVar(var, "initDist_"+std::to_string(l)+"_"+std::to_string(k)));
					}
					else {
						constraint2 = constraint2 || (getVar(var, "pos_"+std::to_string(i))==k
														&& getVar(var, "pos_"+std::to_string(ip))==l
														&& getVar(var, "t_"+std::to_string(ip))>=
															getVar(var, "t_"+std::to_string(i))+getVar(var, "rd_"+std::to_string(ip)));
					}
				}
			}

			constraints.push_back(constraint1 || (getVar(var, "holdA_"+std::to_string(ip))==getVar(var, "holdB_"+std::to_string(i)) && constraint2));
		}
	}

	// Constraint: robots can not occupy the same position in the same action step
	for(int i=1; i<plan_horizon+1; ++i){
		for(int r=1; r<amount_robots+1; ++r){

			// If robot r is acting in step i then set his position to the position in step i
			z3::expr constraint_precondition( getVar(var, "R_"+std::to_string(i))==r );
			z3::expr constraint_effect( getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(i)) );

			for(int rp=1; rp<amount_robots+1; ++rp){
				if(r!=rp){
					if(i==1){
						constraint_effect = constraint_effect && getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == getVar(var, "initPos_"+std::to_string(rp));
					}
					else {
						constraint_effect = constraint_effect && getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i-1));
					}
				}
			}

			constraints.push_back( !(constraint_precondition) || constraint_effect);

			// All robot's positions must exclude each other
			for(int rp=r+1; rp<amount_robots+1; ++rp){
				constraints.push_back( !(getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)))
			 							|| ( getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == 0
											&& getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == 0) );
			}
		}
	}

	// Constraint: machine states are initially consistent
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "M_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "M_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(getVar(var, "insideA_"+std::to_string(ip))==getVar(var, "initInside_"+std::to_string(i))
								&& getVar(var, "outputA_"+std::to_string(ip))==getVar(var, "initOutside_"+std::to_string(i)));

			constraints.push_back(constraint1 || constraint2);
		}
	}

	// Constraint: machine states are inductively consistent
	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "M_"+std::to_string(ip)) == getVar(var, "M_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || (getVar(var, "M_"+std::to_string(ipp)) == getVar(var, "M_"+std::to_string(i)));
			}

			z3::expr constraint2(getVar(var, "t_"+std::to_string(ip))>=getVar(var, "t_"+std::to_string(i))+getVar(var, "md_"+std::to_string(i))
									&& getVar(var, "insideB_"+std::to_string(i))==getVar(var, "insideA_"+std::to_string(ip))
									&& getVar(var, "outputB_"+std::to_string(i))==getVar(var, "outputA_"+std::to_string(ip)));

			constraints.push_back(constraint1 || constraint2);

		}
	}

	/**
	 * ADDITIONAL CONSTRAINTS
	 */

	// Constraints to fix robot order
	if(world_initPos[1] == 0 && world_initPos[2] == 0 && world_initPos[3] == 0) {

		// Start with R-1
		constraints.push_back(getVar(var, "R_1") == 1);

		// R-3 is chosen if R-2 has been chosen before
		for(int i=2; i<plan_horizon+1; ++i) {
			z3::expr constraint_r2_used(var_false);

			for(int j=2; j<i; ++j) {
				constraint_r2_used = constraint_r2_used || getVar(var, "R_"+std::to_string(j)) == 2;
			}

			constraints.push_back(!(getVar(var, "R_"+std::to_string(i)) == 3) || constraint_r2_used);
		}
	}

	// Constraint: for some actions the robot is fixed
	for(int i=1; i<plan_horizon+1; ++i) {
		for(int j=1; j<i; ++j) {
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_1 && getVar(var, "A_"+std::to_string(i)) == index_action_2 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );

			switch (desired_complexity) {
				case 0:
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_4 && getVar(var, "A_"+std::to_string(i)) == index_action_5 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
						break;
				case 1:
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_9 && getVar(var, "A_"+std::to_string(i)) == index_action_5 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
						break;
				case 2:
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_9 && getVar(var, "A_"+std::to_string(i)) == index_action_10 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_11 && getVar(var, "A_"+std::to_string(i)) == index_action_5 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
						break;
				case 3:
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_9 && getVar(var, "A_"+std::to_string(i)) == index_action_10 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_11 && getVar(var, "A_"+std::to_string(i)) == index_action_12 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == index_action_13 && getVar(var, "A_"+std::to_string(i)) == index_action_5 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
						break;
			}
		}
	}

	// // Constraint: every action depends on actions to happen before
	// for(int i=1; i<plan_horizon+1; ++i) {

	//     // Action x appears
	//     z3::expr constraint_dependency1(var_false);
	//     z3::expr constraint_dependency2(var_false);
	//     z3::expr constraint_dependency3(var_false);
	//     z3::expr constraint_dependency4(var_false);
	//     z3::expr constraint_dependency5(var_false);
	//     z3::expr constraint_dependency6(var_false);
	//     z3::expr constraint_dependency7(var_false);
	//     z3::expr constraint_dependency8(var_false);
	//     z3::expr constraint_dependency9(var_false);
	//     z3::expr constraint_dependency10(var_false);
	//     z3::expr constraint_dependency11(var_false);
	//     z3::expr constraint_dependency12(var_false);
	//     z3::expr constraint_dependency13(var_false);

	//     // for(unsigned j=0; j<world_all_actions.size(); ++j){
	//     //     switch(world_all_actions[j]) {
	//     //         case 1: constraint_dependency1 = constraint_dependency1 || var_true;
	//     //                 break;

	//     //         case 2: constraint_dependency2 = constraint_dependency2 || var_true;
	//     //                 break;

	//     //         case 3: constraint_dependency3 = constraint_dependency3 || var_true;
	//     //                 break;

	//     //         case 4: constraint_dependency4 = constraint_dependency4 || var_true;
	//     //                 break;

	//     //         case 5: constraint_dependency5 = constraint_dependency5 || var_true;
	//     //                 break;

	//     //         case 6: constraint_dependency6 = constraint_dependency6 || var_true;
	//     //                 break;

	//     //         case 7: constraint_dependency7 = constraint_dependency7 || var_true;
	//     //                 break;

	//     //         case 8: constraint_dependency8 = constraint_dependency8 || var_true;
	//     //                 break;

	//     //         case 9: constraint_dependency9 = constraint_dependency9 || var_true;
	//     //                 break;

	//     //         case 10: constraint_dependency10 = constraint_dependency10 || var_true;
	//     //                 break;

	//     //         case 11: constraint_dependency11 = constraint_dependency11 || var_true;
	//     //                 break;

	//     //         case 12: constraint_dependency12 = constraint_dependency12 || var_true;
	//     //                 break;

	//     //         case 13: constraint_dependency13 = constraint_dependency13 || var_true;
	//     //                 break;

	//     //         default: break;
	//     //     }
	//     // }

	//     for(int j=1; j<i; ++j) {
	//         constraint_dependency1 = constraint_dependency1 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 1);
	//         constraint_dependency2 = constraint_dependency2 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 2);
	//         constraint_dependency3 = constraint_dependency3 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 3);
	//         constraint_dependency4 = constraint_dependency4 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 4);
	//         constraint_dependency5 = constraint_dependency5 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 5);
	//         constraint_dependency6 = constraint_dependency6 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 6);
	//         constraint_dependency7 = constraint_dependency7 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 7);
	//         constraint_dependency8 = constraint_dependency8 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 8);
	//         constraint_dependency9 = constraint_dependency9 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 9);
	//         constraint_dependency10 = constraint_dependency10 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 10);
	//         constraint_dependency11 = constraint_dependency11 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 11);
	//         constraint_dependency12 = constraint_dependency12 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 12);
	//         constraint_dependency13 = constraint_dependency13 || ( getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(j)) == 13);
	//     }

	//     // Action y has dependency on actions x1,...,xn
	//     z3::expr constraint_inter2(constraint_dependency1);
	//     z3::expr constraint_inter3(constraint_dependency2);
	//     z3::expr constraint_inter7(constraint_dependency4);
	//     z3::expr constraint_inter8(constraint_dependency4);
	//     z3::expr constraint_inter9(constraint_dependency8);
	//     z3::expr constraint_inter10(constraint_dependency9);
	//     z3::expr constraint_inter11(constraint_dependency10);
	//     z3::expr constraint_inter12(constraint_dependency11);
	//     z3::expr constraint_inter13(constraint_dependency12);
	//     z3::expr constraint_inter5(constraint_dependency1 && constraint_dependency2 && constraint_dependency3);
	//     switch (desired_complexity) {
	//         case 0:
	//                 constraint_inter5 = constraint_inter5 && constraint_dependency4;
	//                 break;
	//         case 1:
	//                 constraint_inter5 = constraint_inter5 && constraint_dependency9;
	//                 break;
	//         case 2:
	//                 constraint_inter5 = constraint_inter5 && constraint_dependency11;
	//                 break;
	//         case 3:
	//                 constraint_inter5 = constraint_inter5 && constraint_dependency13;
	//                 break;
	//     }
	//     z3::expr constraint_inter6(constraint_dependency5);

	//     // Push constraint if action = y then dependencies of y must be fulfilled
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_inter2);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 3) || constraint_inter3);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || constraint_inter5);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || constraint_inter6);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 7) || constraint_inter7);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 8) || constraint_inter8);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 9) || constraint_inter9);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 10) || constraint_inter10);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 11) || constraint_inter11);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 12) || constraint_inter12);
	//     constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 13) || constraint_inter13);
	// }

	// Constraint: every action is encoded for every order
	// Save ids of order_colors as strings
	std::string base_str = std::to_string(base);
	std::string ring1_order_str = std::to_string(rings[0]);
	std::string ring2_order_str = std::to_string(rings[1]);
	std::string ring3_order_str = std::to_string(rings[2]);
	std::string cap_str = std::to_string(cap);

	// Init string identifiers for state maps
	std::string bi = "B" + base_str;
	std::string r1i = "R" + ring1_order_str;
	std::string r2i = "R" + ring2_order_str;
	std::string r3i = "R" + ring3_order_str;
	std::string ci = "C" + cap_str;

	// Construct combined string identifiers
	std::string br_ci = "BR" + ci;
	std::string bi_ci = bi + ci;
	std::string bi_r1i = bi + r1i;
	std::string bi_r1i_ci = bi_r1i + ci;
	std::string bi_r1i_r2i = bi_r1i + r2i;
	std::string bi_r1i_r2i_ci = bi_r1i_r2i + ci;
	std::string bi_r1i_r2i_r3i = bi_r1i_r2i + r3i;
	std::string bi_r1i_r2i_r3i_ci = bi_r1i_r2i_r3i + ci;

	std::string has_ci = "has_" + ci;

	std::string sub_product;
	std::string product;
	switch(desired_complexity) {
		case 0:
				sub_product = bi;
				product = bi_ci;
				break;
		case 1:
				sub_product = bi_r1i;
				product = bi_r1i_ci;
				break;
		case 2:
				sub_product = bi_r1i_r2i;
				product = bi_r1i_r2i_ci;
				break;
		case 3:
				sub_product = bi_r1i_r2i_r3i;
				product = bi_r1i_r2i_r3i_ci;
				break;
		default:
				std::cout << "Wrong desired_complexity " << desired_complexity << " to set sub_product and product" << std::endl;
				break;
	}

	// For every step up to the plan_horizon add all req actions depending on the order complexity
	for(int i=1; i<plan_horizon+1; ++i){


		// 0.Action: Do nothing
		z3::expr constraintaction0((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[ci]])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == 0)
									&& (getVar(var, "holdB_"+std::to_string(i)) == getVar(var, "holdA_"+std::to_string(i)))
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_0) || constraintaction0);

		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions all complexities require (which correspond to complexity 0)
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */

		// 1.Action: Retrieve cap_carrier_cap from CS-Shelf
		z3::expr constraintaction1((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[ci]])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
									&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[ci] + "-I"])
									&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products[br_ci])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_1) || constraintaction1);

		// 2.Action : Prepare and feed CS for RETRIEVE with cap_carrier
		z3::expr constraintaction2((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
									&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
									&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "outputB_"+std::to_string(i)) == products["BR"])
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"]) || getVar(var, "pos_"+std::to_string(i)) == 2)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"]) || getVar(var, "pos_"+std::to_string(i)) == 4)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products[br_ci])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_2) || constraintaction2);

		// 3.Action : Retrieve cap_carrier at CS
		z3::expr constraintaction3((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputA_"+std::to_string(i)) > 0) // products["BR"]) // Must be higher than 0 in order to retrieve someting as no prepare is intended here
									&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_disc)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"]) || getVar(var, "pos_"+std::to_string(i)) == 3)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"]) || getVar(var, "pos_"+std::to_string(i)) == 5)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "holdB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_3) || constraintaction3);

		// 4.Action : Prepare and retrieve base from BS
		z3::expr constraintaction4((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"]) // outputA is bound by the init or the predecessor
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
									&& (getVar(var, "pos_"+std::to_string(i)) == 1)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
									&& (!(getVar(var, "outputA_"+std::to_string(i)) == 0) || getVar(var, "holdB_"+std::to_string(i)) == products[bi])
									&& (!(getVar(var, "outputA_"+std::to_string(i)) > 0) || getVar(var, "holdB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i))) // outputA is 0 or the initial output
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_4) || constraintaction4);
		// 5.Action : Prepare and mount sub_product with cap at CS
		z3::expr constraintaction5((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"])
									&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
									&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
									&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "outputB_"+std::to_string(i)) == products[product])
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed) // TODO adapt the time
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS1"]) || getVar(var, "pos_"+std::to_string(i)) == 2)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["CS2"]) || getVar(var, "pos_"+std::to_string(i)) == 4)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products[sub_product])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_5) || constraintaction5);

		// 6.Action : Hold product and deliver at DS
		z3::expr constraintaction6((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"]) // TODO own group?
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
									&& (getVar(var, "pos_"+std::to_string(i)) == 6)
									&& (getVar(var, "holdA_"+std::to_string(i)) == products[product])
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_6) || constraintaction6);

		// 7.Action : Feed additional base into RS for payment and points
		z3::expr constraintaction7((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"] || getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"])
									&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))+1)
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == time_to_feed)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS1"]) || getVar(var, "pos_"+std::to_string(i)) == 7)
									&& (!(getVar(var, "M_"+std::to_string(i)) == machine_groups["RS2"]) || getVar(var, "pos_"+std::to_string(i)) == 9)
									&& (getVar(var, "holdA_"+std::to_string(i)) > 2)
									&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_7) || constraintaction7);

		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions complexities 1,2 and 3 require
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */
		if(desired_complexity > 0) {


			// 8.Action : Prepare and mount base with ring1 at RS
			z3::expr constraintaction8((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r1i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))-rings_req_add_bases[rings[0]])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r1i] + "-I"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_8) || constraintaction8);

			// 9.Action : Retrieve base_ring1 from RS
			z3::expr constraintaction9((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r1i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r1i] + "-O"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_9) || constraintaction9);
		}
		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions complexities 2 and 3 require
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */
		if(desired_complexity > 1) {

			// 10.Action : Prepare and mount base_ring1 with ring2 at RS
			z3::expr constraintaction10((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r2i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))-rings_req_add_bases[rings[1]])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r2i] + "-I"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_10) || constraintaction10);

			// 11.Action : Retrieve base_ring1_ring2 from RS
			z3::expr constraintaction11((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r2i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r2i] + "-O"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_11) || constraintaction11);
		}
		/*
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 * Actions complexity 3 requires
		 * ----------------------------------------------------------------------------------------------------------------------------------------------
		 */
		if(desired_complexity == 3) {

			// 12.Action : Prepare and mount base_ring1_ring2 with ring3 at RS
			z3::expr constraintaction12((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r3i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i))-rings_req_add_bases[rings[2]])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r3i] + "-I"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_12) || constraintaction12);

			// 13.Action : Retrieve base_ring1_ring2_ring3 from RS
			z3::expr constraintaction13((getVar(var, "M_"+std::to_string(i)) == machine_groups[station_colors[r3i]])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[team + "-" + station_colors[r3i] + "-O"])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_13) || constraintaction13);
		}
	}

	/*
	 * Determine points for each step
	 *
	 *  Action 0: Penalty
	 *  Actions 1,6,2,9,11,13: 0 -> 5 to be more favorable than action 3 and 7
	 *  Actio 3: 0
	 *  Action 7: +2
	 *	Action 8,10,12 with ring requires no additional bases: +5
	 *	Action 8,10,12 with ring requires 1 additional bases: +10
	 *	Action 8,10,12 with ring requires 2 additional bases: +20
	 *	Action 8 with last ring of c1: +10
	 *	Action 10 with last ring of c2: +30
	 *	Action 12 with last ring of c3: +80
	 *  Action 4: +10
	 *  Action 5: +20
	 *
	 *  Points are scaled such that they are not eliminated by the travel and machine duration
	 *  Points are scaled by order in action sequence - An earlier action gives more points than a later one
	 */
	for(int i=1; i<plan_horizon+1; ++i){

		// Constraints stating that a action appeared
		z3::expr constraint_action_1_appeared(var_false);
		z3::expr constraint_action_2_appeared(var_false);
		z3::expr constraint_action_3_appeared(var_false);
		z3::expr constraint_action_4_appeared(var_false);
		z3::expr constraint_action_8_appeared(var_false);
		z3::expr constraint_action_9_appeared(var_false);
		z3::expr constraint_action_10_appeared(var_false);
		z3::expr constraint_action_11_appeared(var_false);
		z3::expr constraint_action_12_appeared(var_false);
		z3::expr constraint_action_13_appeared(var_false);
		z3::expr constraint_action_5_appeared(var_false);
		z3::expr constraint_action_6_appeared(var_false);

		for(int j=1; j<i; ++j){
			constraint_action_1_appeared = constraint_action_1_appeared || getVar(var, "A_"+std::to_string(j)) == 1;
			constraint_action_2_appeared = constraint_action_2_appeared || getVar(var, "A_"+std::to_string(j)) == 2;
			constraint_action_3_appeared = constraint_action_3_appeared || getVar(var, "A_"+std::to_string(j)) == 3;
			constraint_action_4_appeared = constraint_action_4_appeared || getVar(var, "A_"+std::to_string(j)) == 4;
			constraint_action_8_appeared = constraint_action_8_appeared || getVar(var, "A_"+std::to_string(j)) == 8;
			constraint_action_9_appeared = constraint_action_9_appeared || getVar(var, "A_"+std::to_string(j)) == 9;
			constraint_action_10_appeared = constraint_action_10_appeared || getVar(var, "A_"+std::to_string(j)) == 10;
			constraint_action_11_appeared = constraint_action_11_appeared || getVar(var, "A_"+std::to_string(j)) == 11;
			constraint_action_12_appeared = constraint_action_12_appeared || getVar(var, "A_"+std::to_string(j)) == 12;
			constraint_action_13_appeared = constraint_action_13_appeared || getVar(var, "A_"+std::to_string(j)) == 13;
			constraint_action_5_appeared = constraint_action_5_appeared || getVar(var, "A_"+std::to_string(j)) == 5;
			constraint_action_6_appeared = constraint_action_6_appeared || getVar(var, "A_"+std::to_string(j)) == 6;
		}

		// for(unsigned j=0; j<world_all_actions.size(); ++j){
		//     if(world_all_actions[j] == 1) {
		//         constraint_action_1_appeared = constraint_action_1_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 2) {
		//         constraint_action_2_appeared = constraint_action_2_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 3) {
		//         constraint_action_3_appeared = constraint_action_3_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 4) {
		//         constraint_action_4_appeared = constraint_action_4_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 8) {
		//         constraint_action_8_appeared = constraint_action_8_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 9) {
		//         constraint_action_9_appeared = constraint_action_9_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 10) {
		//         constraint_action_10_appeared = constraint_action_10_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 11) {
		//         constraint_action_11_appeared = constraint_action_11_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 12) {
		//         constraint_action_12_appeared = constraint_action_12_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 13) {
		//         constraint_action_13_appeared = constraint_action_13_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 5) {
		//         constraint_action_5_appeared = constraint_action_5_appeared || var_true;
		//     }
		//     else if(world_all_actions[j] == 6) {
		//         constraint_action_6_appeared = constraint_action_6_appeared || var_true;
		//     }
		// }

		// Onyl consider action 4,7 multiple times if we have an complexity higher than 0
		constraints.push_back(!(constraint_action_1_appeared) || getVar(var, "A_"+std::to_string(i)) != 1);
		constraints.push_back(!(constraint_action_2_appeared) || getVar(var, "A_"+std::to_string(i)) != 2);
		constraints.push_back(!(constraint_action_3_appeared) || getVar(var, "A_"+std::to_string(i)) != 3);
		if(!desired_complexity) {
			constraints.push_back(!(constraint_action_4_appeared) || getVar(var, "A_"+std::to_string(i)) != 4);
		}
		constraints.push_back(!(constraint_action_8_appeared) || getVar(var, "A_"+std::to_string(i)) != 8);
		constraints.push_back(!(constraint_action_9_appeared) || getVar(var, "A_"+std::to_string(i)) != 9);
		constraints.push_back(!(constraint_action_10_appeared) || getVar(var, "A_"+std::to_string(i)) != 10);
		constraints.push_back(!(constraint_action_11_appeared) || getVar(var, "A_"+std::to_string(i)) != 11);
		constraints.push_back(!(constraint_action_12_appeared) || getVar(var, "A_"+std::to_string(i)) != 12);
		constraints.push_back(!(constraint_action_13_appeared) || getVar(var, "A_"+std::to_string(i)) != 13);
		constraints.push_back(!(constraint_action_5_appeared) || getVar(var, "A_"+std::to_string(i)) != 5);
		constraints.push_back(!(constraint_action_6_appeared) || getVar(var, "A_"+std::to_string(i)) != 6);

		// First step refering to world_points
		if(i==1){
			constraints.push_back(!(getVar(var, "A_1") == 0) || getVar(var, "rew_1") == world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)) - points_penalty);

			constraints.push_back(!( 	getVar(var, "A_1") == index_action_1 ||
										getVar(var, "A_1") == index_action_2 ||
										getVar(var, "A_1") == index_action_3 ||
										getVar(var, "A_1") == index_action_9 ||
										getVar(var, "A_1") == index_action_11 ||
										getVar(var, "A_1") == index_action_13 ) || getVar(var, "rew_1") == initial_points + points_none + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(getVar(var, "A_1") == index_action_4) || getVar(var, "rew_1") == initial_points + points_get_base + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_1") == index_action_7) || getVar(var, "rew_1") == initial_points + points_additional_base + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!( 	(getVar(var, "A_1") == index_action_8 && rings_req_add_bases[0]==0) ||
										(getVar(var, "A_1") == index_action_10 && rings_req_add_bases[1]==0)
									) || getVar(var, "rew_1") == initial_points + points_mount_ring_0 + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!( 	(getVar(var, "A_1") == index_action_8 && rings_req_add_bases[0]==1) ||
										(getVar(var, "A_1") == index_action_10 && rings_req_add_bases[1]==1)
									) || getVar(var, "rew_1") == initial_points + points_mount_ring_1 + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!( 	(getVar(var, "A_1") == index_action_8 && rings_req_add_bases[0]==2) ||
										(getVar(var, "A_1") == index_action_10 && rings_req_add_bases[1]==2)
									) || getVar(var, "rew_1") == initial_points + points_mount_ring_2 + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(getVar(var, "A_1") == index_action_12 && rings_req_add_bases[2]==0) || getVar(var, "rew_1") == initial_points + points_mount_ring1_last + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_1") == index_action_12 && rings_req_add_bases[2]==1) || getVar(var, "rew_1") == initial_points + points_mount_ring2_last + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_1") == index_action_12 && rings_req_add_bases[2]==2) || getVar(var, "rew_1") == initial_points + points_mount_ring3_last + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(getVar(var, "A_1") == index_action_5) || getVar(var, "rew_1") == initial_points + points_mount_cap + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(	getVar(var, "A_1") == index_delivery_action
									) || getVar(var, "rew_1") == initial_points + points_deliver + world_points - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

		}
		// Other steps refering to points_i-1
		else {
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 0) || getVar(var, "rew_"+std::to_string(i)) == getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)) - points_penalty);

			constraints.push_back(!( 	getVar(var, "A_"+std::to_string(i)) == index_action_1 ||
										getVar(var, "A_"+std::to_string(i)) == index_action_2 ||
										getVar(var, "A_"+std::to_string(i)) == index_action_3 ||
										getVar(var, "A_"+std::to_string(i)) == index_action_9 ||
										getVar(var, "A_"+std::to_string(i)) == index_action_11 ||
										getVar(var, "A_"+std::to_string(i)) == index_action_13
									) || getVar(var, "rew_"+std::to_string(i)) == (points_none/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_4) || getVar(var, "rew_"+std::to_string(i)) == points_get_base + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_7) || getVar(var, "rew_"+std::to_string(i)) == points_additional_base + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!( 	(getVar(var, "A_"+std::to_string(i)) == index_action_8 && rings_req_add_bases[0]==0) ||
										(getVar(var, "A_"+std::to_string(i)) == index_action_10 && rings_req_add_bases[1]==0)
									) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_ring_0/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!( 	(getVar(var, "A_"+std::to_string(i)) == index_action_8 && rings_req_add_bases[0]==1) ||
										(getVar(var, "A_"+std::to_string(i)) == index_action_10 && rings_req_add_bases[1]==1)
									) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_ring_1/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!( 	(getVar(var, "A_"+std::to_string(i)) == index_action_8 && rings_req_add_bases[0]==2) ||
										(getVar(var, "A_"+std::to_string(i)) == index_action_10 && rings_req_add_bases[1]==2)
									) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_ring_2/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_12 && rings_req_add_bases[2]==0) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_ring1_last/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_12 && rings_req_add_bases[2]==1) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_ring2_last/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_12 && rings_req_add_bases[2]==2) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_ring3_last/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == index_action_5) || getVar(var, "rew_"+std::to_string(i)) == (points_mount_cap/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));

			constraints.push_back(!(	getVar(var, "A_"+std::to_string(i)) == index_delivery_action
									) || getVar(var, "rew_"+std::to_string(i)) == (points_deliver/i) + getVar(var, "rew_"+std::to_string(i-1)) - getVar(var, "t_"+std::to_string(i)) - getVar(var, "md_"+std::to_string(i)));
		}
	}

	// After delivery do nothing
	for(int i=1; i<plan_horizon+1; ++i) {
		z3::expr constraint_delivery(!(getVar(var, "A_"+std::to_string(i)) == index_delivery_action));
		z3::expr constraint_do_nothing(var_true);
		for(int j=i+1; j<plan_horizon+1; ++j) {
			constraint_do_nothing = constraint_do_nothing && getVar(var, "A_"+std::to_string(j)) == 0;
		}
		constraints.push_back(constraint_delivery || constraint_do_nothing);
	}

	// Specify initial situation for robots
	for(int i=1; i<amount_robots+1; ++i){
		constraints.push_back(getVar(var, "initHold_"+std::to_string(i)) == world_initHold[i]);
		constraints.push_back(getVar(var, "initPos_"+std::to_string(i)) == world_initPos[i]);
	}

	// Specify initial situation for machines
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		constraints.push_back(getVar(var, "initInside_"+std::to_string(i)) == world_initInside[i]);
		constraints.push_back(getVar(var, "initOutside_"+std::to_string(i)) == world_initOutside[i]);
	}


	// Specify positions which are down and therefore not useable
	z3::expr constraint_world_machine_down(var_true);
	for(int i=1; i<plan_horizon+1; ++i){

		for(int world_machine_down: world_machines_down) {

			// Distinguish between action 1 which operates on the input side but is still valid even if the corresponding cap station is down and other actions
			constraint_world_machine_down = constraint_world_machine_down && ( getVar(var, "A_"+std::to_string(i)) == 1 || getVar(var, "pos_"+std::to_string(i)) != world_machine_down);
		}
	}
	constraints.push_back(constraint_world_machine_down);

	// Specify distances between machines
	for(int k=0; k<amount_machines+1; ++k){
		for(int l=k+1; l<amount_machines+1; ++l){
			float distance = velocity_scaling_ * distances[std::make_pair(node_names[k], node_names[l])];
			z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
			constraints.push_back(getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)) == distance_z3);
		}
	}

	return constraints;
}
/**
 * #############################################################################
 * ############################## SOLVE or OPTIMIZE ############################
 * #############################################################################
 *
 * solve_formula() only checks SAT
 * optimize_formula() returns best solution (if SAT) with respect to objective function
 */

bool
 ClipsSmtThread::clips_smt_solve_formula(z3::expr_vector formula)
{
	logger->log_info(name(), "clips_smt_solve_formula");
	bool result = true;

	z3::solver z3Solver(_z3_context); // Use for solving
	for (unsigned i = 0; i < formula.size(); i++) {
		z3Solver.add(formula[i]);
	}

	// Begin measuring solving time
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	z3::set_param("pp.decimal", true);

	if (z3Solver.check() == z3::sat){
		end = std::chrono::high_resolution_clock::now();
		logger->log_info(name(), "Formula is SAT");
		clips_smt_extract_plan_from_model(z3Solver.get_model());


	} else {
		end = std::chrono::high_resolution_clock::now();
		logger->log_info(name(), "Formula is UNSAT");
		result = false;
	}

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	logger->log_info(name(), "Time used for solving is %f ms, %f m", diff_ms, diff_m); // Measure time in nanoseconds but display in milliseconds for convenience
	return result;
}

void
 ClipsSmtThread::clips_smt_optimize_formula(z3::expr_vector formula, std::string var)
{
	logger->log_info(name(), "clips_smt_optimize_formula");

	z3::optimize z3Optimizer(_z3_context); // Use for optimizing
	for (unsigned i = 0; i < formula.size(); i++) {
		z3Optimizer.add(formula[i]);
	}

	// Prepare optimizing
	z3::expr rew_planhorizon = _z3_context.real_const((var.c_str()+std::to_string(plan_horizon)).c_str());
	z3Optimizer.maximize(rew_planhorizon);

	// Begin measuring solving time
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	z3::set_param("pp.decimal", true);

	if (z3Optimizer.check() == z3::sat){
		end = std::chrono::high_resolution_clock::now();
		logger->log_info(name(), "Finished optimizing");
		clips_smt_extract_plan_from_model(z3Optimizer.get_model());

	} else {
		// End measuring solving time
		end = std::chrono::high_resolution_clock::now();
		logger->log_info(name(), "Formula is UNSAT");
	}

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	logger->log_info(name(), "Time used of optimizing is %f ms, %f m", diff_ms, diff_m); // Measure time in nanoseconds but display in milliseconds for convenience
}

void
ClipsSmtThread::clips_smt_extract_plan_from_model(z3::model model)
{
	// Extract plan from model
	for(unsigned i=0; i<model.size(); ++i) {
		z3::func_decl function = model[i];
		// std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;

		std::string function_name = function.name().str();
		z3::expr expr = model.get_const_interp(function);
		float interp = std::stof(Z3_get_numeral_decimal_string(_z3_context, expr, 6));

		for(int j=1; j<plan_horizon+1; ++j){

			if(interp>0) {
				// Compare function_name with descriptions for variabels and extract them into the correpsonding model vector
				if(function_name.compare("M_"+std::to_string(j))==0) {
					model_machines[j] = interp;
				}
				else if(function_name.compare("t_"+std::to_string(j))==0) {
					model_times[j] = interp;
				}
				else if(function_name.compare("pos_"+std::to_string(j))==0) {
					model_positions[j] = (int) interp;
				}
				else if(function_name.compare("R_"+std::to_string(j))==0) {
					model_robots[j] = (int) interp;
				}
				else if(function_name.compare("A_"+std::to_string(j))==0) {
					model_actions[j] = (int) interp;
				}
				else if(function_name.compare("holdA_"+std::to_string(j))==0) {
					model_holdA[j] = (int) interp;
				}
				else if(function_name.compare("insideA_"+std::to_string(j))==0) {
					model_insideA[j] = (int) interp;
				}
				else if(function_name.compare("outputA_"+std::to_string(j))==0) {
					model_outputA[j] = (int) interp;
				}
				else if(function_name.compare("holdB_"+std::to_string(j))==0) {
					model_holdB[j] = (int) interp;
				}
				else if(function_name.compare("insideB_"+std::to_string(j))==0) {
					model_insideB[j] = (int) interp;
				}
				else if(function_name.compare("outputB_"+std::to_string(j))==0) {
					model_outputB[j] = (int) interp;
				}
				else if(function_name.compare("rew_"+std::to_string(j))==0) {
					model_rew[j] = (int) interp;
				}
			}
		}
	}

	// Add plan specified by the model to stats
	logger->log_info(name(), "Generate plan for order with complexity %i with components: B%iC%i", desired_complexity, base, cap);

	for(int j=1; j<plan_horizon+1; ++j){
		logger->log_info(name(), "%i. R-%i A%i hold[%i-%i] pos[%i] M%i input[%i-%i] output[%i-%i] time[%f] rew[%i]", j, model_robots[j], model_actions[j], model_holdA[j], model_holdB[j], model_positions[j], model_machines[j], model_insideA[j], model_insideB[j], model_outputA[j], model_outputB[j], model_times[j], model_rew[j]);
		// world_all_actions.push_back(model_actions[j]);
	}
}

/**
 * #############################################################################
 * ############################## HELP-METHODS #################################
 * #############################################################################
 *
 * getVar() extracts the z3::expr var from the map vars
 * initShelf() initializes the shelf_posiiton structure
 * getNextShelf() returns the next shelf spot with a capcarrier
 */

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

void ClipsSmtThread::initShelf()
{
	shelf_position[0] = true;
	shelf_position[1] = true;
}

std::string ClipsSmtThread::getNextShelf()
{
	if(cap_carrier_index<3) {
		cap_carrier_index++;
	}
	else {
		cap_carrier_index = 1;
	}

	if(shelf_position[0]){
		shelf_position[0] = false;
		return "LEFT";
	}
	else if(shelf_position[1]){
		shelf_position[1] = false;
		return "MIDDLE";
	}

	initShelf();
	return "RIGHT";
}

void ClipsSmtThread::clips_smt_clear_maps()
{
	model_machines.clear();
	model_times.clear();
	model_positions.clear();
	model_robots.clear();
	model_actions.clear();
	model_holdA.clear();
	model_insideA.clear();
	model_outputA.clear();
	model_holdB.clear();
	model_insideB.clear();
	model_outputB.clear();
	model_rew.clear();
	world_all_actions.clear();
}

int ClipsSmtThread::clips_smt_rewrite_product(int base, int ring1, int ring2, int ring3, int cap)
{
	std::string product_description = "nothing";

	// cap_carrier before and after retrieve
	// BRC1
	if (base == 5 && ring1 == 5 && ring2 == 5 && ring3 == 5 && cap == 4) {
		product_description = "BRC1";
	}
	// BRC2
	else if (base == 5 && ring1 == 5 && ring2 == 5 && ring3 == 5 && cap == 5) {
		product_description = "BRC2";
	}
	// BR
	else if (base == 5 && ring1 == 5 && ring2 == 5 && ring3 == 5 && cap == 3) {
		product_description = "BR";
	}
	// wp
	// Only B
	else if (base < 4 && ring1 == 5 && ring2 == 5 && ring3 == 5 && cap == 3) {
		product_description = "B"+std::to_string(base);
	}
	// C0
	else if (base < 4 && ring1 == 5 && ring2 == 5 && ring3 == 5 && cap < 3) {
		product_description = "B"+std::to_string(base)+"C"+std::to_string(cap);
	}
	// Only BxRx
	else if (base < 4 && ring1 < 5 && ring2 == 5 && ring3 == 5 && cap == 3) {
		product_description = "B"+std::to_string(base)+"R"+std::to_string(ring1);
	}
	// C1
	else if (base < 4 && ring1 < 5 && ring2 == 5 && ring3 == 5 && cap < 3) {
		product_description = "B"+std::to_string(base)+"R"+std::to_string(ring1)+"C"+std::to_string(cap);
	}
	// Only BxRxRx
	else if (base < 4 && ring1 < 5 && ring2 < 5 && ring3 == 5 && cap == 3) {
		product_description = "B"+std::to_string(base)+"R"+std::to_string(ring1)+"R"+std::to_string(ring2);
	}
	// C2
	else if (base < 4 && ring1 < 5 && ring2 < 5 && ring3 == 5 && cap < 3) {
		product_description = "B"+std::to_string(base)+"R"+std::to_string(ring1)+"R"+std::to_string(ring2)+"C"+std::to_string(cap);
	}
	// Only BxRxRxRx
	else if (base < 4 && ring1 < 5 && ring2 < 5 && ring3 < 5 && cap == 3) {
		product_description = "B"+std::to_string(base)+"R"+std::to_string(ring1)+"R"+std::to_string(ring2)+"R"+std::to_string(ring3);
	}
	// C3
	else if (base < 4 && ring1 < 5 && ring2 < 5 && ring3 < 5 && cap < 3) {
		product_description = "B"+std::to_string(base)+"R"+std::to_string(ring1)+"R"+std::to_string(ring2)+"R"+std::to_string(ring3)+"C"+std::to_string(cap);
	}
	// empty
	else if (base == 4 && ring1 == 5 && ring2 == 5 && ring3 == 5 && cap == 3) {
	}
	else {
		logger->log_info(name(), "Product was not recognized");
	}

	return products[product_description];
}
