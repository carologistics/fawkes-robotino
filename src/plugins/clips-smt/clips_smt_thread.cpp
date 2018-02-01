
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
	/**
	 * Initialize maps for the formula encoder.
	 */

	/*
	 * Inside encodes if the CS has the cap retrieved in order to mount it with some subproduct.
	 */
	inside_capstation["nothing"]=0;
	inside_capstation["has_C1"]=1;
	inside_capstation["has_C2"]=2;

	/*
	 * Products encodes the output of an station which can be any product (at the CS) and subproduct (at the BS and RS) OR the product a robot is holding
	 * TODO Does a smarter way exist to invert a map?
	 * TODO Do we need all products in both maps, because the base station is not putting something on the belt.
	 */
	products["nothing"]=0;
	products_inverted[0]="nothing";
	products["full"]=1;
	products_inverted[1]="full";
	products["BR"]=2;
	products_inverted[2]="BR";

	unsigned ctr = 2;

	// B1 ... B3 
	for(unsigned b=1; b<4; ++b){
		std::string name = "B"+std::to_string(b);
		ctr++;
		products[name] = ctr;
		products_inverted[ctr] = name;
	}

	// B1C1 ... B3C2
	for(unsigned b=1; b<4; ++b){
		for(unsigned c=1; c<3; ++c) {
			std::string name = "B"+std::to_string(b)+"C"+std::to_string(c);
			ctr++;
			products[name] = ctr;
			products_inverted[ctr] = name;
		}
	}

	// B1R1 ... B3R4
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1);
			ctr++;
			products[name] = ctr;
			products_inverted[ctr] = name;
		}
	}

	// B1R1C1 ... B3R4C2
	for(unsigned b=1; b<4; ++b){
		for(unsigned r1=1; r1<5; ++r1) {
			for(unsigned c=1; c<3; ++c) {
				std::string name = "B"+std::to_string(b)+"R"+std::to_string(r1)+"C"+std::to_string(c);
				ctr++;
				products[name] = ctr;
				products_inverted[ctr] = name;
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
				products_inverted[ctr] = name;
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
					products_inverted[ctr] = name;
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
					products_inverted[ctr] = name;
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
						products_inverted[ctr] = name;
					}
				}
			}
		}
	}

	/*
	 * Machine_groups assigns each station type to a number
	 */
	machine_groups["CS"]=0;
	machine_groups["BS"]=1;
	machine_groups["RS"]=2;
	machine_groups["DS"]=3;

	/**
	 * Initialize maps and vectors for managing the input and ouput.
	 */

	// Description of MACRO-actions
	description_actions.push_back("dummy"); // We are not interested in the action with value 0
	description_actions.push_back("[Feed C with cap carrier from shelf into CS]"); // 1
	description_actions.push_back("[Discard cap carrier at CS-O]"); // 2
	description_actions.push_back("[Retrieve B at BS]"); // 3
	description_actions.push_back("[Feed intermediate workpiece into CS-I to mount with C]"); // 4
	description_actions.push_back("[Retrieve final product at CS-O]"); // 5
	description_actions.push_back("[Deliver final product at DS]"); // 6
	description_actions.push_back("[Feed add base into RS-I]"); // 7
	description_actions.push_back("[Feed B into RS-I to mount with R1]"); // 8
	description_actions.push_back("[Retrieve BR1 at RS-O]"); // 9
	description_actions.push_back("[Feed BR1 into RS-I to mount with R2]"); // 10
	description_actions.push_back("[Retrieve BR1R2 at RS-O]"); // 11
	description_actions.push_back("[Feed BR1R2 into RS-I to mount with R3]"); // 12
	description_actions.push_back("Retrieve BR1R2R3 at RS-O"); // 13

	// shelf positions // TODO Use all shelf-positions in protobuf
	shelf_position.push_back(true);
	shelf_position.push_back(true);

	// number_required_bases
	number_required_bases.push_back(0); // dummy
	number_required_bases.push_back(0); // 1 or RING_BLUE
	number_required_bases.push_back(0); // 2 or RING_GREEN
	number_required_bases.push_back(0); // 3 or RING_ORANGE
	number_required_bases.push_back(0); // 4 or RING_YELLOW

	max_number_required_bases_rs1 = 0;
	max_number_required_bases_rs2 = 0;

	// Initialize rings_order
	base_order = 1;
	rings_order.push_back(1);
	rings_order.push_back(2);
	rings_order.push_back(3);
	cap_order = 1;

	// Initialize number_orders_c3
	number_orders_c0 = 0;
	number_orders_c1 = 0;
	number_orders_c2 = 0;
	number_orders_c3 = 0;

	// Initialize world state fix 
	world_initHold.push_back(0); // dummy
	world_initHold.push_back(products["nothing"]);
	world_initHold.push_back(products["nothing"]);
	world_initHold.push_back(products["nothing"]);

	world_initPos.push_back(0); // dummy
	world_initPos.push_back(0);
	world_initPos.push_back(0);
	world_initPos.push_back(0);

	world_initInside.push_back(inside_capstation["nothing"]); // CS
	world_initInside.push_back(inside_capstation["nothing"]); // BS
	world_initInside.push_back(inside_capstation["nothing"]); // RS
	world_initInside.push_back(inside_capstation["nothing"]); // DS

	world_initOutside.push_back(products["nothing"]); // CS
	world_initOutside.push_back(products["nothing"]); // BS
	world_initOutside.push_back(products["nothing"]); // RS
	world_initOutside.push_back(products["nothing"]); // DS

	world_initAddRS1 = 0;
	world_initAddRS2 = 0;

	world_points = 0;

	/*
	 * Initialize colors for bases, rings and caps
	 */
	base_colors[1] = "BASE_RED";
	base_colors[2] = "BASE_BLACK";
	base_colors[3] = "BASE_SILVER";

	ring_colors[1] = "RING_BLUE";
	ring_colors[2] = "RING_GREEN";
	ring_colors[3] = "RING_ORANGE";
	ring_colors[4] = "RING_YELLOW";

	cap_colors[1] = "CAP_BLACK";
	cap_colors[2] = "CAP_GREY";
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
	actor_plan->set_actor_name("Master");
	llsf_msgs::SequentialPlan *plan = actor_plan->mutable_sequential_plan();
	llsf_msgs::PlanAction *action;
	llsf_msgs::PlanActionParameter *param;
	// llsf_msgs::uint32 *uint32;

	// action = plan->add_actions();
	// action->set_name("enter-field");

	// Francesco Leofante's approach
	std::vector<uint32_t> action_id_last;
	for(int i=0; i<=number_total_actions; ++i){
		action_id_last.push_back(0);
	}
	std::vector<uint32_t> action_id_last_bs_robot;
	for(int i=0; i<=4; ++i){
		action_id_last_bs_robot.push_back(0);
	}
	std::vector<uint32_t> action_id_last_rs1_pay;
	std::vector<uint32_t> action_id_last_rs2_pay;
	uint32_t action_id_last_rs1_feed=0;
	uint32_t action_id_last_rs1_retr=0;
	uint32_t action_id_last_rs2_feed=0;
	uint32_t action_id_last_rs2_retr=0;

	uint32_t action_id=0;

	for(int i=1; i<plan_horizon+1; ++i){
		switch(model_actions[i]) { // }((model_actions[i]-1)%index_upper_bound_actions)+1){
			case 0: // Dummy action to free location

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

			case 1:	// Actions 1,2,3

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve_shelf");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("shelf");
					param->set_value("TRUE"); // TODO Replace TRUE by getNextShelf() once implemented

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("operation");
					param->set_value("RETRIEVE_CAP");

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 2:	// Action 8

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[1]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("discard");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());

					break;
			case 3:	// Action 7,6

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					// Only go to the base station if last robot which got a base (mentioned in our sequential plan) finishes its action
					// if(action_id_last[3]) action->add_parent_id(action_id_last[3]);

					// Only go to the base station if the base intended to be mounted comes after all the base retrievals which are intended as payment
					// Look ahead to dermine if this instance of action 3 is the (!) one which requires add parent_ids
					for(unsigned j=i+1; j<model_actions.size(); ++j) {

						// Check if action is an instance of action 7 and the current base retrieval is performed by the same robot
						// In this case we do not need to add further parent_ids
						if(model_actions[j] == 7 && model_robots[i] == model_robots[j]) {
							break;
						}

						// Check if action is an instance of action 8 and the current base retrieval is performed by the same robot
						else if(model_actions[j] == 8 && model_robots[i] == model_robots[j]) {

							// RS1
							if(model_positions[j]==7) {
								for(unsigned k=0; k<action_id_last_rs1_pay.size(); ++k) {
									if((int) k<number_required_bases[rings_order[0]]) {
										action->add_parent_id(action_id_last_rs1_pay[k]);
									}
									else {
										std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 1." << std::endl;
									}
								}
							}
							// RS2
							else if(model_positions[j]==9) {
								for(unsigned k=0; k<action_id_last_rs2_pay.size(); ++k) {
									if((int) k<number_required_bases[rings_order[0]]) {
										action->add_parent_id(action_id_last_rs2_pay[k]);
									}
									else {
										std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 2." << std::endl;
									}
								}
							}

						}

					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[1]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[1]);
					param = action->add_params();
					param->set_key("color");
					param->set_value(base_colors[data.orders(order_id).base_color()]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[1]);

					action_id_last_bs_robot[model_robots[i]] = action_id;

					break;
			case 4:	// Action 4,5

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
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
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[1]);
					action->add_parent_id(action_id_last[2]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("operation");
					param->set_value("MOUNT_CAP");

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;
			case 5:	// Action 9

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[2]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[4]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

					break;

			case 6:	// Action 1order_id,11

					++action_id;
					action = plan->add_actions();
					action->set_name("move");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id_last[5]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[6]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[6]);
					param = action->add_params();
					param->set_key("gate");
					param->set_value(std::to_string(data.orders(order_id).delivery_gate()));

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[6]);

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
					// ...if the corresponding ring station is already filled with 3 bases, wait for the feed action to occur in order to nothing.
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
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("slide");
					param->set_value("true");

					// Save action_id_last for feed at the correpsonding ring station
					if(model_positions[i] == 7) {
						action_id_last_rs1_pay.push_back(action_id);
					}
					else if(model_positions[i] == 9) {
						action_id_last_rs2_pay.push_back(action_id);
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
						for(unsigned j=0; j<action_id_last_rs1_pay.size(); ++j) {
							if((int) j<number_required_bases[rings_order[0]]) {
								action->add_parent_id(action_id_last_rs1_pay[j]);
							}
							else {
								std::cout << "We omit putting a payment action as a parent which is not necessary for some feed action at the ring station 1." << std::endl;
							}
						}
					}
					else if(model_positions[i]==9) {
						for(unsigned j=0; j<action_id_last_rs2_pay.size(); ++j) {
							if((int) j<number_required_bases[rings_order[0]]) {
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
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring_color");
					param->set_value(ring_colors[data.orders(order_id).ring_colors(0)]);

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

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
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[8]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

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
						for(unsigned j=0; j<action_id_last_rs1_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs1_pay[j]);
						}
						if(action_id_last_rs1_feed) {
							action->add_parent_id(action_id_last_rs1_feed);
						}
					}
					else if(model_positions[i]==9){
						for(unsigned j=0; j<action_id_last_rs2_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs2_pay[j]);
						}
						if(action_id_last_rs2_feed) {
							action->add_parent_id(action_id_last_rs2_feed);
						}
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring_color");
					param->set_value(ring_colors[data.orders(order_id).ring_colors(1)]);

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

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
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[10]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

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
						for(unsigned j=0; j<action_id_last_rs1_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs1_pay[j]);
						}
						if(action_id_last_rs1_feed) {
							action->add_parent_id(action_id_last_rs1_feed);
						}
					}
					else if(model_positions[i]==9){
						for(unsigned j=0; j<action_id_last_rs2_pay.size(); ++j) {
							action->add_parent_id(action_id_last_rs2_pay[j]);
						}
						if(action_id_last_rs2_feed) {
							action->add_parent_id(action_id_last_rs2_feed);
						}
					}
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("to");
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("prepare");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);
					param = action->add_params();
					param->set_key("ring_color");
					param->set_value(ring_colors[data.orders(order_id).ring_colors(2)]);

					++action_id;
					action = plan->add_actions();
					action->set_name("feed");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

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
					param->set_value(node_names_[model_positions[i]]);

					++action_id;
					action = plan->add_actions();
					action->set_name("retrieve");
					action->set_actor("R-"+std::to_string(model_robots[i]));
					action->set_id(action_id);
					action->add_parent_id(action_id-1);
					action->add_parent_id(action_id_last[12]);
					action->set_goal_id(data.orders(order_id).id());
					param = action->add_params();
					param->set_key("mps");
					param->set_value(node_names_[model_positions[i]]);

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

	// Fill the nothing protobuf message with the computed task
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

	// Extract required information from protobuf
	clips_smt_fill_general_info();
	// Start with desired complexity 3 and decrease until a valid order was found (by comparing number_order)
	clips_smt_fill_order_details(0);
	clips_smt_fill_ringstation_details();
	clips_smt_fill_capstation_details();
	clips_smt_initialize_numbers();

	// Extract required information from navgraph
	clips_smt_fill_node_names();
	clips_smt_fill_robot_names();
	clips_smt_compute_distances_robots();
	clips_smt_compute_distances_machines();

	clips_smt_fill_ringstation_details_extended();

	/*
	 * Francesco Leofante's approach
	 */

	// Declare variable for encoding
	std::map<std::string, z3::expr> var;

	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	bool iterate = true;
	while(iterate) {
		z3::expr_vector formula = clips_smt_encoder(var);

		clips_smt_optimize_formula(formula, "score_");
		// clips_smt_solve_formula(formula);
		
		// Break and display plan if delivery_action is performed
		for(int action: world_all_actions) {
			if(action == index_delivery_action) {
				iterate = false;
			}
		}
		// What happens if all actions are 0, deadlock because a machine is down for example
		// Then stop solving iteratively and reduce the complexity 
		// If no order can be pursuaded (no time and/or exaclty the necessary cap station is down) go for filling bases into the ringstation
	}

	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	

	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	logger->log_info(name(), "Time difference is %f ms", diff_ms); // Measure time in nanoseconds but display in milliseconds for convenience

	logger->log_info(name(), "Thread reached end of loop");

	envs_[data_env].lock();
	envs_[data_env]->assert_fact_f("(smt-plan-complete \"%s\")", data_handle.c_str());
	envs_[data_env].unlock();
}

/**
 * From protobuf Methods
 *	- Order details
 *	- Ringstation details
 *	- Capstation details (actually from the config)
 *	- Navgraph details (actually from Navgraph)
 */

void
ClipsSmtThread::clips_smt_fill_general_info()
{
	logger->log_info(name(), "Extract general information");

	// Extract constants
	number_robots = data.robots().size()-1;
	number_machines = 10; // data.machines().size();
	number_orders_protobuf = data.orders().size();

	// Extract team from protobuf
	team = "M";
	if(data.robots(0).team_color() == 0){
		team = "C";
	}

	// Extract add bases requried for corresponding colors
	for( int i=0; i<data.rings().size(); ++i ) {
		number_required_bases[data.rings(i).ring_color()] = data.rings(i).raw_material();
	}
}
void
ClipsSmtThread::clips_smt_fill_order_details(int desired_complexity)
{
	logger->log_info(name(), "Extract details about order to pursue");

	// Goal strategy pick last one of desired complexity
	for(int i=0; i<number_orders_protobuf; ++i) {
		std::cout << "Compare data_complexity " << data.orders(i).complexity() << " with desired_complexity " << desired_complexity << std::endl;
		if(data.orders(i).complexity() == desired_complexity) {

			base_order = data.orders(i).base_color();
			cap_order = data.orders(i).cap_color();

			// Extract rings order information with respect ot the desired complexity
			switch(desired_complexity) {
				case 1: 
					rings_order[0] = data.orders(i).ring_colors(0);
					number_orders_c1 = 1;
					break;

				case 2:
					rings_order[0] = data.orders(i).ring_colors(0);
					rings_order[1] = data.orders(i).ring_colors(1);
					number_orders_c2 = 1;
					break;

				case 3:
					rings_order[0] = data.orders(i).ring_colors(0);
					rings_order[1] = data.orders(i).ring_colors(1);
					rings_order[2] = data.orders(i).ring_colors(2);
					number_orders_c3 = 1;
					break;
				
				default:
					number_orders_c0 = 1;
					break;
			}
			
			order_id = i;
		}
	}

	std::cout << "Detected number_orders (" << number_orders_c0 <<", "  << number_orders_c1 <<", " << number_required_actions_c2 << ", " << number_orders_c3 << ")" << std::endl;
	
}

void
ClipsSmtThread::clips_smt_fill_ringstation_details()
{
	logger->log_info(name(), "Extract details about ringstations");

	// Extract information from protobuf which
	std::string var_ringStation = team+"-RS1";
	for(int i=0; i<data.machines().size(); ++i) {
		if(var_ringStation.compare(data.machines(i).name()) == 0){
			if(data.machines(i).ring_colors(0)==1 || data.machines(i).ring_colors(1)==1) {
				// team-RS1 contains blue r1 ring
				colors_input["R1"] = team+"-RS1-I";
				colors_output["R1"] = team+"-RS1-O";
			}
			else {
				// team-RS2 contains blue r1 rings
				colors_input["R1"] = team+"-RS2-I";
				colors_output["R1"] = team+"-RS2-O";
			}
			if(data.machines(i).ring_colors(0)==2 || data.machines(i).ring_colors(1)==2) {
				// team-RS1 contains green r2 rings
				colors_input["R2"] = team+"-RS1-I";
				colors_output["R2"] = team+"-RS1-O";
			}
			else {
				// team-RS2 contains green r2 rings
				colors_input["R2"] = team+"-RS2-I";
				colors_output["R2"] = team+"-RS2-O";
			}
			if(data.machines(i).ring_colors(0)==3 || data.machines(i).ring_colors(1)==3) {
				// team-RS1 contains orange r3 rings
				colors_input["R3"] = team+"-RS1-I";
				colors_output["R3"] = team+"-RS1-O";
			}
			else {
				// team-RS2 contains orange r3 rings
				colors_input["R3"] = team+"-RS2-I";
				colors_output["R3"] = team+"-RS2-O";
			}
			if(data.machines(i).ring_colors(0)==4 || data.machines(i).ring_colors(1)==4) {
				// team-RS1 contains yellow r4 rings
				colors_input["R4"] = team+"-RS1-I";
				colors_output["R4"] = team+"-RS1-O";
			}
			else {
				// team-RS2 contains yellow r4 rings
				colors_input["R4"] = team+"-RS2-I";
				colors_output["R4"] = team+"-RS2-O";
			}
			continue;
		}
	}
}

void
ClipsSmtThread::clips_smt_fill_ringstation_details_extended()
{

	for(int i=0; i<data.machines().size(); ++i){
		std::string var_down = "DOWN";
		std::string var_break = "BREAK";

		if(var_down.compare(data.machines(i).state().c_str()) == 0 || var_break.compare(data.machines(i).state().c_str()) == 0) {
			std::string machine_name = data.machines(i).name().c_str();
			machine_name += "-I";

			world_machines_down.push_back(node_names_inverted[machine_name]);
		}
	}

	std::cout << "Machines down are ";
	for(int world_machine_down: world_machines_down) {
		std::cout << world_machine_down << " ";
	}
	std::cout << std::endl;

	for(unsigned i=0; i<rings_order.size(); ++i) {
		// std::cout << "Check color of R" << rings_order[i] << " corresponding to station " << node_names_inverted[colors_input["R"+std::to_string(rings_order[i])]] << std::endl;
		switch(node_names_inverted[colors_input["R"+std::to_string(rings_order[i])]]) {
			case 7:
					if(number_required_bases[rings_order[i]] > max_number_required_bases_rs1) {
						// std::cout << "Update RS1" << std::endl;
						max_number_required_bases_rs1 = number_required_bases[rings_order[i]];
					}
					break;
			case 9:
					if(number_required_bases[rings_order[i]] > max_number_required_bases_rs2) {
						// std::cout << "Update RS2" << std::endl;
						max_number_required_bases_rs2 = number_required_bases[rings_order[i]];
					}
					break;
			default:
					break;
		}
	}
}

void
ClipsSmtThread::clips_smt_fill_capstation_details()
{
	logger->log_info(name(), "Extract details about capstations");

	// Extract information from config which cap-station-shelf has which color
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
}

/**
 * Manage order and actions constants
 */

void
ClipsSmtThread::clips_smt_initialize_numbers()
{
	logger->log_info(name(), "Initialize numbers");

	// Determine number_required_actions for the corresponding complexity
	number_required_actions_c1 += 2*number_required_bases[rings_order[0]];
	number_required_actions_c2 += 2*number_required_bases[rings_order[0]] + 2*number_required_bases[rings_order[1]];
	number_required_actions_c3 += 2*number_required_bases[rings_order[0]] + 2*number_required_bases[rings_order[1]] + 2*number_required_bases[rings_order[2]];

	// Determine index_upper_bound_actions and number_required_actions
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

	// For now we assume that only one order is solved at the same time TODO remove or keep current structure?
	number_orders = 1; //number_orders_c0 + number_orders_c1 + number_orders_c2 + number_orders_c3; 
	number_total_actions = number_orders*index_upper_bound_actions;

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
	logger->log_info(name(), "Extract name of machines");
	node_names_.clear();

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
}

void
ClipsSmtThread::clips_smt_fill_robot_names()
{
	logger->log_info(name(), "Extract name of robots");
	robot_names_.clear();

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

			for (unsigned i = 1; i < node_names_.size(); ++i) {
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

	// Compute distances between unconnected C-ins-in and all other machines
	NavGraphNode ins_node(navgraph->node(node_names_[0]));
	NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

	for (unsigned i = 1; i < node_names_.size(); ++i) {
		std::pair<std::string, std::string> nodes_pair(ins_node.name(), node_names_[i]);

		NavGraphNode to = navgraph->node(node_names_[i]);
		NavGraphPath p = navgraph->search_path(from, to);

		distances_[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
	}

	// Compute distances between machines
	for (unsigned i = 1; i < node_names_.size(); ++i) {
		for (unsigned j = 1; j < node_names_.size(); ++j) {
			if (i == j) continue;
			std::pair<std::string, std::string> nodes_pair(node_names_[i], node_names_[j]);

			NavGraphPath p = navgraph->search_path(node_names_[i], node_names_[j]);

			distances_[nodes_pair] = p.cost();
		}
	}
}

/*
 * Methods for encoding the given protobuf data in formulas
 */

z3::expr_vector
ClipsSmtThread::clips_smt_encoder(std::map<std::string, z3::expr>& var)
{
	logger->log_info(name(), "Create z3 encoder");

	/*
	 * PRECOMPUTATION
	 */

	// Vector collecting all constraints
	z3::expr_vector constraints(_z3_context);

	// Init variable true and false
	z3::expr var_false(_z3_context.bool_val(false));
	z3::expr var_true(_z3_context.bool_val(true));


	/*
	 * VARIABLES
	 */
	logger->log_info(name(), "Add variables");

	// Variables initDist_i_j
	for(int i = 0; i < number_machines+1; ++i){
		for(int j = i+1; j < number_machines+1; ++j) {
			var.insert(std::make_pair("initDist_" + std::to_string(i) + "_" + std::to_string(j), _z3_context.real_const(("initDist_" + std::to_string(i) + "_" + std::to_string(j)).c_str())));
		}
	}

	// Variables initPos and initHold
	for(int i = 1; i < number_robots+1; ++i){
		var.insert(std::make_pair("initPos_" + std::to_string(i), _z3_context.int_const(("initPos_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initHold_" + std::to_string(i), _z3_context.int_const(("initHold_" + std::to_string(i)).c_str())));
	}

	// Variables initState1_i, initInside_i and initOutside_i
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		// var.insert(std::make_pair("initState1_" + std::to_string(i), _z3_context.int_const(("initState1_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initInside_" + std::to_string(i), _z3_context.int_const(("initInside_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("initOutside_" + std::to_string(i), _z3_context.int_const(("initOutside_" + std::to_string(i)).c_str())));
	}
	var.insert(std::make_pair("initAddRS1", _z3_context.int_const(("initAddRS1"))));
	var.insert(std::make_pair("initAddRS2", _z3_context.int_const(("initAddRS2"))));

	// Variables depending on plan_horizon
	for(int i=1; i<plan_horizon+1; ++i){
		var.insert(std::make_pair("t_" + std::to_string(i), _z3_context.real_const(("t_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("rd_" + std::to_string(i), _z3_context.real_const(("rd_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("pos_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(i)).c_str())));
		for(int r=1; r<number_robots+1; ++r){
			var.insert(std::make_pair("pos_" + std::to_string(r) + "_" + std::to_string(i), _z3_context.int_const(("pos_" + std::to_string(r) + "_" + std::to_string(i)).c_str())));
		}
		var.insert(std::make_pair("md_" + std::to_string(i), _z3_context.real_const(("md_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("R_" + std::to_string(i), _z3_context.int_const(("R_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("A_" + std::to_string(i), _z3_context.int_const(("A_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("M_" + std::to_string(i), _z3_context.int_const(("M_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("holdA_" + std::to_string(i), _z3_context.int_const(("holdA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("insideA_" + std::to_string(i), _z3_context.int_const(("insideA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputA_" + std::to_string(i), _z3_context.int_const(("outputA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputA_" + std::to_string(i), _z3_context.int_const(("outputA_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("addRS1A_" + std::to_string(i), _z3_context.int_const(("addRS1A_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("addRS2A_" + std::to_string(i), _z3_context.int_const(("addRS2A_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("holdB_" + std::to_string(i), _z3_context.int_const(("holdB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("insideB_" + std::to_string(i), _z3_context.int_const(("insideB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("outputB_" + std::to_string(i), _z3_context.int_const(("outputB_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("addRS1B_" + std::to_string(i), _z3_context.int_const(("addRS1B_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("addRS2B_" + std::to_string(i), _z3_context.int_const(("addRS2B_" + std::to_string(i)).c_str())));
		// var.insert(std::make_pair("rew_" + std::to_string(i), _z3_context.real_const(("rew_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("points_" + std::to_string(i), _z3_context.real_const(("points_" + std::to_string(i)).c_str())));
		var.insert(std::make_pair("score_" + std::to_string(i), _z3_context.real_const(("score_" + std::to_string(i)).c_str())));
	}


	/*
	 * CONSTRAINTS
	 */
	logger->log_info(name(), "Add constraints");

	// Constraints depending on plan_horizon
	// logger->log_info(name(), "Add constraints depending on plan_horizon");

	for(int i = 1; i < plan_horizon+1; ++i){

		// VarStartTime
		// General bound
		constraints.push_back(0 <= getVar(var, "t_"+std::to_string(i)) && getVar(var, "t_"+std::to_string(i)) <= 900);
		// Robot specifc bound
		for(int j = 1; j < i; ++j){
			constraints.push_back(!(getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i))) || getVar(var, "t_"+std::to_string(j)) <= getVar(var, "t_"+std::to_string(i)));
		}

		constraints.push_back(0 <= getVar(var, "rd_"+std::to_string(i))); // VarRobotDuration
		constraints.push_back(1 <= getVar(var, "pos_"+std::to_string(i)) && getVar(var, "pos_"+std::to_string(i)) <= number_machines); // VarRobotPosition
		for(int r=1; r<number_robots+1; ++r){
			constraints.push_back(0 <= getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) && getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) <= number_machines); // VarRobotPosition
		}
		constraints.push_back(0 <= getVar(var, "md_"+std::to_string(i))); // VarMachineDuration
		constraints.push_back(1 <= getVar(var, "R_"+std::to_string(i)) && getVar(var, "R_"+std::to_string(i)) <= number_robots); // VarR
		constraints.push_back(0 <= getVar(var, "A_"+std::to_string(i)) && getVar(var, "A_"+std::to_string(i)) <= number_total_actions); // VarA
		constraints.push_back(min_machine_groups <= getVar(var, "M_"+std::to_string(i)) && getVar(var, "M_"+std::to_string(i)) <= max_machine_groups); // VarM
		constraints.push_back(min_products <= getVar(var, "holdA_"+std::to_string(i)) && getVar(var, "holdA_"+std::to_string(i)) <= max_products); // VarHoldA
		constraints.push_back(min_inside_capstation <= getVar(var, "insideA_"+std::to_string(i)) && getVar(var, "insideA_"+std::to_string(i)) <= max_inside_capstation); // VarInsideA
		constraints.push_back(min_products <= getVar(var, "outputA_"+std::to_string(i)) && getVar(var, "outputA_"+std::to_string(i)) <= max_products); // VarOutsideA
		constraints.push_back(min_add_bases_ringstation <= getVar(var, "addRS1A_"+std::to_string(i)) && getVar(var, "addRS1A_"+std::to_string(i)) <= max_number_required_bases_rs1); // VarAddRS1A
		constraints.push_back(min_add_bases_ringstation <= getVar(var, "addRS2A_"+std::to_string(i)) && getVar(var, "addRS2A_"+std::to_string(i)) <= max_number_required_bases_rs2); // VarAddRS2A
		constraints.push_back(min_products <= getVar(var, "holdB_"+std::to_string(i)) && getVar(var, "holdB_"+std::to_string(i)) <= max_products); // VarHoldB
		constraints.push_back(min_inside_capstation <= getVar(var, "insideB_"+std::to_string(i)) && getVar(var, "insideB_"+std::to_string(i)) <= max_inside_capstation); // VarInsideB
		constraints.push_back(min_products <= getVar(var, "outputB_"+std::to_string(i)) && getVar(var, "outputB_"+std::to_string(i)) <= max_products); // VarOutsideB
		constraints.push_back(min_add_bases_ringstation <= getVar(var, "addRS1B_"+std::to_string(i)) && getVar(var, "addRS1B_"+std::to_string(i)) <= max_number_required_bases_rs1); // VarAddRS1A
		constraints.push_back(min_add_bases_ringstation <= getVar(var, "addRS2B_"+std::to_string(i)) && getVar(var, "addRS2B_"+std::to_string(i)) <= max_number_required_bases_rs2); // VarAddRS2A

	}

	// Constraint: robot states are initially consistent
	// logger->log_info(name(), "Add constraints stating robot states are initially consistent");

	for(int i=1; i<number_robots+1; ++i){
		for(int ip=1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "R_"+std::to_string(ip)) == i));
			for(int ipp=1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "R_"+std::to_string(ipp))==i;
			}

			z3::expr constraint2(var_false);
			for(int k=0; k<number_machines+1; ++k){
				for(int l=1; l<number_machines+1; ++l){
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
	// logger->log_info(name(), "Add constraints stating robot states are inductively consistent");

	for(int i=1; i<plan_horizon+1; ++i){
		for(int ip=i+1; ip<plan_horizon+1; ++ip){

			z3::expr constraint1( !(getVar(var, "R_"+std::to_string(ip)) == getVar(var, "R_"+std::to_string(i))));
			for(int ipp=i+1; ipp<ip; ++ipp){
				constraint1 = constraint1 || getVar(var, "R_"+std::to_string(ipp))==getVar(var, "R_"+std::to_string(i));
			}

			z3::expr constraint2(var_false);
			for(int k=1; k<number_machines+1; ++k){
				for(int l=1; l<number_machines+1; ++l){
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
		for(int r=1; r<number_robots+1; ++r){

			// If robot r is acting in step i then set his position to the position in step i
			z3::expr constraint_precondition( getVar(var, "R_"+std::to_string(i))==r );
			z3::expr constraint_effect( getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(i)) );

			for(int rp=1; rp<number_robots+1; ++rp){
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
			for(int rp=r+1; rp<number_robots+1; ++rp){
				constraints.push_back( !(getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)))
			 							|| ( getVar(var, "pos_"+std::to_string(r)+"_"+std::to_string(i)) == 0
											&& getVar(var, "pos_"+std::to_string(rp)+"_"+std::to_string(i)) == 0) );
			}
		}
	}

	// Constraint: machine states are initially consistent
	// logger->log_info(name(), "Add constraints stating machine states are initially consistent");

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
	constraints.push_back(getVar(var, "addRS1A_"+std::to_string(1))==getVar(var, "initAddRS1")
						&& getVar(var, "addRS2A_"+std::to_string(1))==getVar(var, "initAddRS2"));

	// Constraint: machine states are inductively consistent
	// logger->log_info(name(), "Add constraints stating robot states are inductively consistent");

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

		if(i<plan_horizon){
			constraints.push_back(getVar(var, "addRS1B_"+std::to_string(i))==getVar(var, "addRS1A_"+std::to_string(i+1))
									&& getVar(var, "addRS2B_"+std::to_string(i))==getVar(var, "addRS2A_"+std::to_string(i+1)));
		}
	}

	// Constraints to fix robot order
	// logger->log_info(name(), "Consider order of robots in the initial situation");

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
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 9 && getVar(var, "A_"+std::to_string(i)) == 10 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 11 && getVar(var, "A_"+std::to_string(i)) == 12 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 13 && getVar(var, "A_"+std::to_string(i)) == 4 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
			constraints.push_back( !( getVar(var, "A_"+std::to_string(j)) == 5 && getVar(var, "A_"+std::to_string(i)) == 6 ) || getVar(var, "R_"+std::to_string(j)) == getVar(var, "R_"+std::to_string(i)) );
		}
	}

	// Constraint: every action depends on actions to happen before
	for(int i=1; i<plan_horizon+1; ++i) {

		// Inter MACRO action dependency
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

		for(unsigned j=0; j<world_all_actions.size(); ++j){
			switch(world_all_actions[j]) {
				case 1: constraint_dependency1 = constraint_dependency1 || var_true;
						break;

				case 2: constraint_dependency2 = constraint_dependency2 || var_true;
						break;

				case 3: constraint_dependency3 = constraint_dependency3 || var_true;
						break;

				case 4: constraint_dependency4 = constraint_dependency4 || var_true;
						break;

				case 5: constraint_dependency5 = constraint_dependency5 || var_true;
						break;

				case 6: constraint_dependency6 = constraint_dependency6 || var_true;
						break;

				case 7: constraint_dependency7 = constraint_dependency7 || var_true;
						break;

				case 8: constraint_dependency8 = constraint_dependency8 || var_true;
						break;

				case 9: constraint_dependency9 = constraint_dependency9 || var_true;
						break;

				case 10: constraint_dependency10 = constraint_dependency10 || var_true;
						break;

				case 11: constraint_dependency11 = constraint_dependency11 || var_true;
						break;

				case 12: constraint_dependency12 = constraint_dependency12 || var_true;
						break;

				case 13: constraint_dependency13 = constraint_dependency13 || var_true;
						break;

				default: break;
			}
			
		}
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

		z3::expr constraint_inter2(constraint_dependency1);
		z3::expr constraint_inter7(constraint_dependency3);
		z3::expr constraint_inter8(constraint_dependency3);
		z3::expr constraint_inter9(constraint_dependency8);
		z3::expr constraint_inter10(constraint_dependency9);
		z3::expr constraint_inter11(constraint_dependency10);
		z3::expr constraint_inter12(constraint_dependency11);
		z3::expr constraint_inter13(constraint_dependency12);
		z3::expr constraint_inter4(constraint_dependency1 && constraint_dependency2); 
		if(number_orders_c0 == 1) {
			constraint_inter4 = constraint_inter4 && constraint_dependency3;
		}
		else if(number_orders_c1 == 1) {
			constraint_inter4 = constraint_inter4 && constraint_dependency9;
		}
		else if(number_orders_c2 == 1) {
			constraint_inter4 = constraint_inter4 && constraint_dependency11;
		}
		else if(number_orders_c3 == 1) {
			constraint_inter4 = constraint_inter4 && constraint_dependency13;
		}
		z3::expr constraint_inter5(constraint_dependency4);
		z3::expr constraint_inter6(constraint_dependency5);

		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_inter2);
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || constraint_inter4);
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

	// Constraint: every action is encoded for every order
	// logger->log_info(name(), "Add constraints for actions for %i orders (%i inside protobuf)", number_orders, number_orders_protobuf);

	// Save ids of order_colors as strings
	std::string base_order_str = std::to_string(base_order);
	std::string ring1_order_str = std::to_string(rings_order[0]);
	std::string ring2_order_str = std::to_string(rings_order[1]);
	std::string ring3_order_str = std::to_string(rings_order[2]);
	std::string cap_order_str = std::to_string(cap_order);

	// Init string identifiers for state maps
	std::string bi = "B" + base_order_str;
	std::string r1i = "R" + ring1_order_str;
	std::string r2i = "R" + ring2_order_str;
	std::string r3i = "R" + ring3_order_str;
	std::string ci = "C" + cap_order_str;

	// Construct combined string identifiers
	std::string bi_ci = bi + ci;
	std::string bi_r1i = bi + r1i;
	std::string bi_r1i_ci = bi_r1i + ci;
	std::string bi_r1i_r2i = bi_r1i + r2i;
	std::string bi_r1i_r2i_ci = bi_r1i_r2i + ci;
	std::string bi_r1i_r2i_r3i = bi_r1i_r2i + r3i;
	std::string bi_r1i_r2i_r3i_ci = bi_r1i_r2i_r3i + ci;

	std::string has_ci = "has_" + ci;
	std::string has_r1i = "has_" + r1i;
	std::string has_r2i = "has_" + r2i;
	std::string has_r3i = "has_" + r3i;

	// For every step up to the plan_horizon add all required actions depending on the order complexity
	for(int i=1; i<plan_horizon+1; ++i){

		// 0.Dummyaction: Move somewhere else and do nothing
		z3::expr constraint_dummyaction(
									(getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
									&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
									&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
									&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
									&& (getVar(var, "md_"+std::to_string(i)) == 0)
									&& (getVar(var, "holdB_"+std::to_string(i)) == getVar(var, "holdA_"+std::to_string(i))
										|| getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
									&& (getVar(var, "rd_"+std::to_string(i)) == 0));
		constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 0) || constraint_dummyaction);

		// Help constraint
		z3::expr constraint_rs1( getVar(var, "pos_"+std::to_string(i))==7 );
		z3::expr constraint_rs2( getVar(var, "pos_"+std::to_string(i))==9 );

		if(number_orders_c0) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["BR"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == 1)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(var, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(var, "pos_"+std::to_string(i)) == 6)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_ci])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);
		}
		else if(number_orders_c1) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == 1)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_ci])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_ci])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_ci])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(var, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(var, "pos_"+std::to_string(i)) == 6)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_ci])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);

			// 7.Macroaction : Feed base into ringstation
			z3::expr constraint_macroaction7((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))+1 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))+1
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == 7 || getVar(var, "pos_"+std::to_string(i)) == 9) 
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 7) || constraint_macroaction7);

			// 8.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction8((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i])
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))-number_required_bases[rings_order[0]] 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))-number_required_bases[rings_order[0]]
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 8) || constraint_macroaction8);

			// 9.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction9((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										// && (getVar(var, "state1B_"+std::to_string(i)) == getVar(var, "state1A_"+std::to_string(i)))
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r1i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 9) || constraint_macroaction9);
		}
		else if(number_orders_c2) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == 1)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i_ci])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i_ci])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_ci])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(var, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(var, "pos_"+std::to_string(i)) == 6)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i_ci])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);

			// 7.Macroaction : Feed base into ringstation
			z3::expr constraint_macroaction7((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))+1 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))+1
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == 7 || getVar(var, "pos_"+std::to_string(i)) == 9) 
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 7) || constraint_macroaction7);

			// 8.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction8((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i])
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))-number_required_bases[rings_order[0]] 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))-number_required_bases[rings_order[0]]
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 8) || constraint_macroaction8);

			// 9.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction9((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r1i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 9) || constraint_macroaction9);

			// 10.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction10((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))-number_required_bases[rings_order[1]] 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))-number_required_bases[rings_order[1]]
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r2i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 10) || constraint_macroaction10);

			// 11.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction11((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r2i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 11) || constraint_macroaction11);
		}
		else if(number_orders_c3) {
			// 1.Macroaction: Prepare CapStation for Retrieve [1,2,3]
			z3::expr constraint_macroaction1((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch+time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 1) || constraint_macroaction1);

			// 2.Macroaction : discard capless base from CS [8]
			z3::expr constraint_macroaction2((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["full"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_disc)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["BR"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 2) || constraint_macroaction2);

			// 3.Macroaction : Get Base from BaseStation [7,6]
			z3::expr constraint_macroaction3((getVar(var, "M_"+std::to_string(i)) == machine_groups["BS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == 1)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 3) || constraint_macroaction3);

			// 4.Macroaction : Prepare CapStation for Mount [4,5]
			z3::expr constraint_macroaction4((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation[has_ci])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i_ci])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || constraint_macroaction4);

			// 5.Action : retrieve base with cap from CS [9]
			z3::expr constraint_macroaction5((getVar(var, "M_"+std::to_string(i)) == machine_groups["CS"])
										&& (getVar(var, "insideA_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == inside_capstation["nothing"])
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i_ci])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[ci]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i_ci])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || constraint_macroaction5);

			// 6.Macroaction : Deliver product [10,11]
			z3::expr constraint_macroaction6((getVar(var, "M_"+std::to_string(i)) == machine_groups["DS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_prep)
										&& (getVar(var, "pos_"+std::to_string(i)) == 6)
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i_ci])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || constraint_macroaction6);

			// 7.Macroaction : Feed base into ringstation
			z3::expr constraint_macroaction7((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputB_"+std::to_string(i)) == getVar(var, "outputA_"+std::to_string(i)))
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))+1 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))+1
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == 7 || getVar(var, "pos_"+std::to_string(i)) == 9) 
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi]
											|| getVar(var, "holdA_"+std::to_string(i)) == products["BR"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 7) || constraint_macroaction7);

			// 8.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction8((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i])
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))-number_required_bases[rings_order[0]] 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))-number_required_bases[rings_order[0]]
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r1i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 8) || constraint_macroaction8);

			// 9.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction9((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r1i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 9) || constraint_macroaction9);

			// 10.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction10((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))-number_required_bases[rings_order[1]] 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))-number_required_bases[rings_order[1]]
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r2i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 10) || constraint_macroaction10);

			// 11.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction11((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r2i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 11) || constraint_macroaction11);

			// 12.Macroaction : Prepare and mount base with ring at RS
			z3::expr constraint_macroaction12((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& ( !constraint_rs1 || ( getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))-number_required_bases[rings_order[2]] 
										   && getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))) )
										&& ( !constraint_rs2 || ( getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i))-number_required_bases[rings_order[2]]
											&& getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i))) )
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_prep+time_to_feed)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_input[r3i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products[bi_r1i_r2i])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 12) || constraint_macroaction12);

			// 13.Macroaction : Retrieve base_ring from RS
			z3::expr constraint_macroaction13((getVar(var, "M_"+std::to_string(i)) == machine_groups["RS"])
										&& (getVar(var, "insideB_"+std::to_string(i)) == getVar(var, "insideA_"+std::to_string(i)))
										&& (getVar(var, "outputA_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "outputB_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "addRS1B_"+std::to_string(i)) == getVar(var, "addRS1A_"+std::to_string(i)))
										&& (getVar(var, "addRS2B_"+std::to_string(i)) == getVar(var, "addRS2A_"+std::to_string(i)))
										&& (getVar(var, "md_"+std::to_string(i)) == time_to_fetch)
										&& (getVar(var, "pos_"+std::to_string(i)) == node_names_inverted[colors_output[r3i]])
										&& (getVar(var, "holdA_"+std::to_string(i)) == products["nothing"])
										&& (getVar(var, "holdB_"+std::to_string(i)) == products[bi_r1i_r2i_r3i])
										&& (getVar(var, "rd_"+std::to_string(i)) == 0));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 13) || constraint_macroaction13);
		}
	}


	// TODO add for multiple orders
	// logger->log_info(name(), "Add constraints for scores");
	for(int i=1; i<plan_horizon+1; ++i){
		//phase
		z3::expr constraint_action_1_appeared(var_false);
		z3::expr constraint_action_2_appeared(var_false);
		z3::expr constraint_action_3_appeared(var_false);
		z3::expr constraint_action_8_appeared(var_false);
		z3::expr constraint_action_9_appeared(var_false);
		z3::expr constraint_action_10_appeared(var_false);
		z3::expr constraint_action_11_appeared(var_false);
		z3::expr constraint_action_12_appeared(var_false);
		z3::expr constraint_action_13_appeared(var_false);

		for(int j=1; j<i; ++j){
			constraint_action_1_appeared = constraint_action_1_appeared || getVar(var, "A_"+std::to_string(j)) == 1;
			constraint_action_2_appeared = constraint_action_2_appeared || getVar(var, "A_"+std::to_string(j)) == 2;
			constraint_action_3_appeared = constraint_action_3_appeared || getVar(var, "A_"+std::to_string(j)) == 3;
			constraint_action_8_appeared = constraint_action_8_appeared || getVar(var, "A_"+std::to_string(j)) == 8;
			constraint_action_9_appeared = constraint_action_9_appeared || getVar(var, "A_"+std::to_string(j)) == 9;
			constraint_action_10_appeared = constraint_action_10_appeared || getVar(var, "A_"+std::to_string(j)) == 10;
			constraint_action_11_appeared = constraint_action_11_appeared || getVar(var, "A_"+std::to_string(j)) == 11;
			constraint_action_12_appeared = constraint_action_12_appeared || getVar(var, "A_"+std::to_string(j)) == 12;
			constraint_action_13_appeared = constraint_action_13_appeared || getVar(var, "A_"+std::to_string(j)) == 13;
		}

		for(unsigned j=0; j<world_all_actions.size(); ++j){
			if(world_all_actions[j] == 1) {
				constraint_action_1_appeared = constraint_action_1_appeared || var_true;
			}
			else if(world_all_actions[j] == 2) {
				constraint_action_2_appeared = constraint_action_2_appeared || var_true;
			}
			else if(world_all_actions[j] == 3) {
				constraint_action_3_appeared = constraint_action_3_appeared || var_true;
			}
			else if(world_all_actions[j] == 8) {
				constraint_action_8_appeared = constraint_action_8_appeared || var_true;
			}
			else if(world_all_actions[j] == 9) {
				constraint_action_9_appeared = constraint_action_9_appeared || var_true;
			}
			else if(world_all_actions[j] == 10) {
				constraint_action_10_appeared = constraint_action_10_appeared || var_true;
			}
			else if(world_all_actions[j] == 11) {
				constraint_action_11_appeared = constraint_action_11_appeared || var_true;
			}
			else if(world_all_actions[j] == 12) {
				constraint_action_12_appeared = constraint_action_12_appeared || var_true;
			}
			else if(world_all_actions[j] == 13) {
				constraint_action_13_appeared = constraint_action_13_appeared || var_true;
			}
		}
		
		// Do not consider action 1 and 2 a second time in the production cycle of the product
		constraints.push_back(!(constraint_action_1_appeared) || getVar(var, "A_"+std::to_string(i)) != 1);
		constraints.push_back(!(constraint_action_2_appeared) || getVar(var, "A_"+std::to_string(i)) != 2);
		if(number_orders_c0) {
			constraints.push_back(!(constraint_action_3_appeared) || getVar(var, "A_"+std::to_string(i)) != 3);
		}
		constraints.push_back(!(constraint_action_8_appeared) || getVar(var, "A_"+std::to_string(i)) != 8);
		constraints.push_back(!(constraint_action_9_appeared) || getVar(var, "A_"+std::to_string(i)) != 9);
		constraints.push_back(!(constraint_action_10_appeared) || getVar(var, "A_"+std::to_string(i)) != 10);
		constraints.push_back(!(constraint_action_11_appeared) || getVar(var, "A_"+std::to_string(i)) != 11);
		constraints.push_back(!(constraint_action_12_appeared) || getVar(var, "A_"+std::to_string(i)) != 12);
		constraints.push_back(!(constraint_action_13_appeared) || getVar(var, "A_"+std::to_string(i)) != 13);

		// First step
		if(i==1){

			// With var_time
			// phase 1 (prepare R1)
			constraints.push_back(!( !(constraint_action_8_appeared) && getVar(var, "A_1") == 3) || getVar(var, "score_1") == 110-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( !(constraint_action_8_appeared) && getVar(var, "A_1") == 7) || getVar(var, "score_1") == 120-getVar(var, "t_"+std::to_string(i)));
			// phase 2 (mount R1)
			constraints.push_back(!( getVar(var, "A_1") == 8) || getVar(var, "score_1") == 1100-getVar(var, "t_"+std::to_string(i)));
			// phase 3 (prepare R2)
			constraints.push_back(!( constraint_action_8_appeared && !(constraint_action_10_appeared) && getVar(var, "A_1") == 3) || getVar(var, "score_1") == 11000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( constraint_action_8_appeared && !(constraint_action_10_appeared) && getVar(var, "A_1") == 7) || getVar(var, "score_1") == 12000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_1") == 9) || getVar(var, "score_1") == 13000-getVar(var, "t_"+std::to_string(i)));
			// phase 4 (mount R2)
			constraints.push_back(!( getVar(var, "A_1") == 10) || getVar(var, "score_1") == 110000-getVar(var, "t_"+std::to_string(i)));
			// phase 5 (prepare R3)
			constraints.push_back(!( constraint_action_10_appeared && !(constraint_action_12_appeared) && getVar(var, "A_1") == 3) || getVar(var, "score_1") == 1100000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( constraint_action_10_appeared && !(constraint_action_12_appeared) && getVar(var, "A_1") == 7) || getVar(var, "score_1") == 1200000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_1") == 11) || getVar(var, "score_1") == 1300000-getVar(var, "t_"+std::to_string(i)));
			// phase 6 (mount R3)
			constraints.push_back(!( getVar(var, "A_1") == 12) || getVar(var, "score_1") == 11000000-getVar(var, "t_"+std::to_string(i)));
			// phase 7 (prepare C)
			constraints.push_back(!( constraint_action_12_appeared && getVar(var, "A_1") == 3) || getVar(var, "score_1") == 0-getVar(var, "t_"+std::to_string(i))-100000000);
			constraints.push_back(!( constraint_action_12_appeared && getVar(var, "A_1") == 7) || getVar(var, "score_1") == 0-getVar(var, "t_"+std::to_string(i))-100000000);
			constraints.push_back(!( getVar(var, "A_1") == 1) || getVar(var, "score_1") == 110000000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_1") == 2) || getVar(var, "score_1") == 120000000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_1") == 13) || getVar(var, "score_1") == 130000000-getVar(var, "t_"+std::to_string(i)));
			// phase 0 (final steps)
			constraints.push_back(!(getVar(var, "A_1") == 4) || getVar(var, "score_1") == 1100000000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_1") == 5) || getVar(var, "score_1") == 1200000000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_1") == 6) || getVar(var, "score_1") == 1300000000-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_1") == 0) || getVar(var, "score_1") == 0-getVar(var, "t_"+std::to_string(i)));


			// Add points depending on the action variable assignments
			// +0 for action 1,2,3,5,9,11,13
			constraints.push_back(!( 	getVar(var, "A_1") == 0 ||
										getVar(var, "A_1") == 1 ||
										getVar(var, "A_1") == 2 ||
										getVar(var, "A_1") == 3 ||
										getVar(var, "A_1") == 5 ||
										getVar(var, "A_1") == 9 ||
										getVar(var, "A_1") == 11 ||
										getVar(var, "A_1") == 13
									) || getVar(var, "points_1") == 0 + world_points);

			// +2 for action 7
			constraints.push_back(!(getVar(var, "A_1") == 7) || getVar(var, "points_1") == 2 + world_points);

			// +5 for action 8, 10, 12 if the corresponding rings cost 0 add bases
			constraints.push_back(!( 	(getVar(var, "A_1") == 8 && number_required_bases[0]==0) ||
										(getVar(var, "A_1") == 10 && number_required_bases[1]==0)
									) || getVar(var, "points_1") == 5 + world_points);

			// +10 for action 8, 10, 12 if the corresponding rings cost 1 add base
			constraints.push_back(!( 	(getVar(var, "A_1") == 8 && number_required_bases[0]==1) ||
										(getVar(var, "A_1") == 10 && number_required_bases[1]==1)
									) || getVar(var, "points_1") == 10 + world_points);

			// +20 for action 8, 10, 12 if the corresponding rings cost 2 add bases
			constraints.push_back(!( 	(getVar(var, "A_1") == 8 && number_required_bases[0]==2) ||
										(getVar(var, "A_1") == 10 && number_required_bases[1]==2)
									) || getVar(var, "points_1") == 20 + world_points);

			// +10 for action 8 (mounting last ring of a c1)
			// +30 for action 10 (mounting last ring of a c2)
			// +80 for action 12 (mounting last ring of a c3)
			constraints.push_back(!(getVar(var, "A_1") == 12 && number_required_bases[2]==0) || getVar(var, "points_1") == 85 + world_points);
			constraints.push_back(!(getVar(var, "A_1") == 12 && number_required_bases[2]==1) || getVar(var, "points_1") == 90 + world_points);
			constraints.push_back(!(getVar(var, "A_1") == 12 && number_required_bases[2]==2) || getVar(var, "points_1") == 100 + world_points);

			// +10 for action 4 (mounting cap)
			constraints.push_back(!(getVar(var, "A_1") == 4) || getVar(var, "points_1") == 10 + world_points);

			// +20 for action 6 (deliver product)
			constraints.push_back(!(getVar(var, "A_1") == 6) || getVar(var, "points_1") == 20 + world_points);

		}
		// Next step referring on the score one step before
		else {

			// With var_time
			// phase 1 (prepare R1)
			constraints.push_back(!( !(constraint_action_8_appeared) && getVar(var, "A_"+std::to_string(i)) == 3) || getVar(var, "score_"+std::to_string(i)) == 110/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( !(constraint_action_8_appeared) && getVar(var, "A_"+std::to_string(i)) == 7) || getVar(var, "score_"+std::to_string(i)) == 120/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			// phase 2 (mount R1)
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 8) || getVar(var, "score_"+std::to_string(i)) == 1100/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			// phase 3 (prepare R2)
			constraints.push_back(!( constraint_action_8_appeared && !(constraint_action_10_appeared) && getVar(var, "A_"+std::to_string(i)) == 3) || getVar(var, "score_"+std::to_string(i)) == 11000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( constraint_action_8_appeared && !(constraint_action_10_appeared) && getVar(var, "A_"+std::to_string(i)) == 7) || getVar(var, "score_"+std::to_string(i)) == 12000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 9) || getVar(var, "score_"+std::to_string(i)) == 13000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			// phase 4 (mount R2)
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 10) || getVar(var, "score_"+std::to_string(i)) == 110000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			//phase 5 (prepare R3)
			constraints.push_back(!( constraint_action_10_appeared && !(constraint_action_12_appeared) && getVar(var, "A_"+std::to_string(i)) == 3) || getVar(var, "score_"+std::to_string(i)) == 1100000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( constraint_action_10_appeared && !(constraint_action_12_appeared) && getVar(var, "A_"+std::to_string(i)) == 7) || getVar(var, "score_"+std::to_string(i)) == 1200000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 11) || getVar(var, "score_"+std::to_string(i)) == 1300000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			// phase 6 (mount R3)
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 12) || getVar(var, "score_"+std::to_string(i)) == 11000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			// phase 7 (prepare C)
			constraints.push_back(!( constraint_action_12_appeared && getVar(var, "A_"+std::to_string(i)) == 3) || getVar(var, "score_"+std::to_string(i)) == getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i))-100000000);
			constraints.push_back(!( constraint_action_12_appeared && getVar(var, "A_"+std::to_string(i)) == 7) || getVar(var, "score_"+std::to_string(i)) == getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i))-100000000);
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 1) || getVar(var, "score_"+std::to_string(i)) == 110000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 2) || getVar(var, "score_"+std::to_string(i)) == 120000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!( getVar(var, "A_"+std::to_string(i)) == 13) || getVar(var, "score_"+std::to_string(i)) == 130000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			// phase 0 (final steps)
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || getVar(var, "score_"+std::to_string(i)) == 1100000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 5) || getVar(var, "score_"+std::to_string(i)) == 1200000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || getVar(var, "score_"+std::to_string(i)) == 1300000000/i +getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 0) || getVar(var, "score_"+std::to_string(i)) == getVar(var, "score_"+std::to_string(i-1))-getVar(var, "t_"+std::to_string(i)));

			// Add points depending on the action variable assignments
			// +0 for action 1,2,3,5,9,11,13
			constraints.push_back(!( 	getVar(var, "A_"+std::to_string(i)) == 0 ||
										getVar(var, "A_"+std::to_string(i)) == 1 ||
										getVar(var, "A_"+std::to_string(i)) == 2 ||
										getVar(var, "A_"+std::to_string(i)) == 3 ||
										getVar(var, "A_"+std::to_string(i)) == 5 ||
										getVar(var, "A_"+std::to_string(i)) == 9 ||
										getVar(var, "A_"+std::to_string(i)) == 11 ||
										getVar(var, "A_"+std::to_string(i)) == 13
									) || getVar(var, "points_"+std::to_string(i)) == getVar(var, "points_"+std::to_string(i-1)));

			// +2 for action 7
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 7) || getVar(var, "points_"+std::to_string(i)) == 2+getVar(var, "points_"+std::to_string(i-1)));

			// +5 for action 8, 10, 12 if the corresponding rings cost 0 add bases
			constraints.push_back(!( 	(getVar(var, "A_"+std::to_string(i)) == 8 && number_required_bases[0]==0) ||
										(getVar(var, "A_"+std::to_string(i)) == 10 && number_required_bases[1]==0)
									) || getVar(var, "points_"+std::to_string(i)) == 5+getVar(var, "points_"+std::to_string(i-1)));

			// +10 for action 8, 10, 12 if the corresponding rings cost 1 add base
			constraints.push_back(!( 	(getVar(var, "A_"+std::to_string(i)) == 8 && number_required_bases[0]==1) ||
										(getVar(var, "A_"+std::to_string(i)) == 10 && number_required_bases[1]==1)
									) || getVar(var, "points_"+std::to_string(i)) == 10+getVar(var, "points_"+std::to_string(i-1)));

			// +20 for action 8, 10, 12 if the corresponding rings cost 2 add bases
			constraints.push_back(!( 	(getVar(var, "A_"+std::to_string(i)) == 8 && number_required_bases[0]==2) ||
										(getVar(var, "A_"+std::to_string(i)) == 10 && number_required_bases[1]==2)
									) || getVar(var, "points_"+std::to_string(i)) == 20+getVar(var, "points_"+std::to_string(i-1)));

			// +10 for action 8 (mounting last ring of a c1)
			// +30 for action 10 (mounting last ring of a c2)
			// +80 for action 12 (mounting last ring of a c3)
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 12 && number_required_bases[2]==0) || getVar(var, "points_"+std::to_string(i)) == 85+getVar(var, "points_"+std::to_string(i-1)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 12 && number_required_bases[2]==1) || getVar(var, "points_"+std::to_string(i)) == 90+getVar(var, "points_"+std::to_string(i-1)));
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 12 && number_required_bases[2]==2) || getVar(var, "points_"+std::to_string(i)) == 100+getVar(var, "points_"+std::to_string(i-1)));

			// +10 for action 4 (mounting cap)
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 4) || getVar(var, "points_"+std::to_string(i)) == 10+getVar(var, "points_"+std::to_string(i-1)));

			// +20 for action 6 (deliver product)
			constraints.push_back(!(getVar(var, "A_"+std::to_string(i)) == 6) || getVar(var, "points_"+std::to_string(i)) == 20+getVar(var, "points_"+std::to_string(i-1)));
		}
	}

	// logger->log_info(name(), "Add constraints for initial situation");

	// Specify initial situation for robots
	for(int i=1; i<number_robots+1; ++i){
		constraints.push_back(getVar(var, "initHold_"+std::to_string(i)) == world_initHold[i]);
		constraints.push_back(getVar(var, "initPos_"+std::to_string(i)) == world_initPos[i]);
	}

	// Specify initial situation for machines
	for(int i=min_machine_groups; i<max_machine_groups+1; ++i){
		constraints.push_back(getVar(var, "initInside_"+std::to_string(i)) == world_initInside[i]); 
		constraints.push_back(getVar(var, "initOutside_"+std::to_string(i)) == world_initOutside[i]);
	}
	constraints.push_back(getVar(var, "initAddRS1") == world_initAddRS1);
	constraints.push_back(getVar(var, "initAddRS2") == world_initAddRS2);


	z3::expr constraint_world_machine_down(var_true);
	for(int i=1; i<plan_horizon+1; ++i){

		for(int world_machine_down: world_machines_down) {
			constraint_world_machine_down = constraint_world_machine_down && getVar(var, "pos_"+std::to_string(i)) != world_machine_down;
		}
	}
	constraints.push_back(constraint_world_machine_down);

	// logger->log_info(name(), "Add constraints for distances between machines");

	// Specify distances between machines
	for(int k=0; k<number_machines+1; ++k){
		for(int l=k+1; l<number_machines+1; ++l){
			float distance = velocity_scaling_ * distances_[std::make_pair(node_names_[k], node_names_[l])];
			z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
			constraints.push_back(getVar(var, "initDist_"+std::to_string(k)+"_"+std::to_string(l)) == distance_z3);
		}
	}

	// Additional constraints
	// If action 6 occured only use action 0 later TODO

	// logger->log_info(name(), "Add constraints for final actions");

	// Constraints encoding that final_actions for each order have to be at least executed once (if desiered, during the delivery window)
	// On branch smt-planning-score we can not longer ensure to finish a product
	// z3::expr constraint_goal(var_true);
	// for(int o=0; o<number_orders; ++o){

	//     z3::expr constraint_subgoal(var_false);
	//     for(int i=number_required_actions; i<plan_horizon+1; ++i){

	//         z3::expr constraint_finalaction(getVar(var, "A_"+std::to_string(i)) == o*index_upper_bound_actions+index_delivery_action);

	//         if(add_temporal_constraint){
	//             constraints.push_back(!constraint_finalaction || (getVar(var, "t_"+std::to_string(i)) < (int) data.orders(o).delivery_period_end()
	//                                                                     && getVar(var, "t_"+std::to_string(i)) > (int) data.orders(o).delivery_period_begin()));
	//         }

	//         constraint_subgoal = constraint_subgoal || constraint_finalaction;
	//     }

	//     constraint_goal = constraint_goal && constraint_subgoal;
	// }
	// constraints.push_back(constraint_goal);

	return constraints;
}

/*
 * Solve encoding
 *	- Solve from z3 built formula
 *	- Optimize from z3 built formula
 *	- Solve from smt file
 *	- Optimize from smt file
 */

void
 ClipsSmtThread::clips_smt_solve_formula(z3::expr_vector formula)
{
	logger->log_info(name(), "Solve z3 formula");

	z3::solver z3Solver(_z3_context); // Use for solving

	for (unsigned i = 0; i < formula.size(); i++) {
		z3Solver.add(formula[i]);
	}

	// Begin measuring solving time
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	z3::set_param("pp.decimal", true);

	// Export formula into .smt file
	std::ofstream of_c0_formula("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/formula_smt.smt"); // TODO (Igor) Exchange path with config value
	of_c0_formula << z3Solver.to_smt2() << std::endl;
	of_c0_formula.close();

	if (z3Solver.check() == z3::sat){
		// End measuring solving time in case of sat
		end = std::chrono::high_resolution_clock::now();

		logger->log_info(name(), "Finished solving");
		clips_smt_extract_plan_from_model(z3Solver.get_model(), "/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats_smt.txt", begin, end);

	} else {
		// End measuring solving time in case of unsat
		end = std::chrono::high_resolution_clock::now();

		logger->log_info(name(), "Formula is UNSAT");
		clips_smt_extract_unsat_reason("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats_smt.txt", begin, end);
	}
}

void
 ClipsSmtThread::clips_smt_optimize_formula(z3::expr_vector formula, std::string var)
{
	logger->log_info(name(), "Optimize z3 formula");

	z3::optimize z3Optimizer(_z3_context); // Use for optimizing

	for (unsigned i = 0; i < formula.size(); i++) {
		z3Optimizer.add(formula[i]);
	}

	// Prepare optimizing
	z3::expr var_planhorizon = _z3_context.real_const((var.c_str()+std::to_string(plan_horizon)).c_str());
	z3Optimizer.maximize(var_planhorizon);

	// Begin measuring solving time
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

	z3::set_param("pp.decimal", true);

	// Export formula into .smt file
	std::ofstream of_c0_formula("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/formula_omt.smt"); // TODO (Igor) Exchange path with config value
	of_c0_formula << Z3_optimize_to_string(_z3_context, z3Optimizer);
	of_c0_formula.close();

	if (z3Optimizer.check() == z3::sat){
		// End measuring solving time
		end = std::chrono::high_resolution_clock::now();

		logger->log_info(name(), "Finished optimizing");
		clips_smt_extract_plan_from_model(z3Optimizer.get_model(), "/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats_omt.txt", begin, end);

	} else {
		// End measuring solving time
		end = std::chrono::high_resolution_clock::now();

		logger->log_info(name(), "Optimizing is not possible due to UNSAT formula");
		clips_smt_extract_unsat_reason("/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/clips_smt_thread_stats_omt.txt", begin, end);
	}
}

z3::expr ClipsSmtThread::clips_smt_extract_formula_from_smt_file(std::string path) {

	Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
	z3::expr e(_z3_context, a);

	return e;
}

void
ClipsSmtThread::clips_smt_extract_plan_from_model(z3::model model, std::string of_path, std::chrono::high_resolution_clock::time_point begin, std::chrono::high_resolution_clock::time_point end)
{
	clips_smt_clear_maps();

	// Export stats into clips_smt_thread_stats.txt
	std::ofstream of_stats;
	of_stats.open(of_path, std::ofstream::out | std::ofstream::app);

	// Add time and date stamp to stats
	time_t now = time(0);
	char* dt = ctime(&now);
	of_stats << std::endl << dt << std::endl;

	// Add sat to stats
	of_stats << "Formula is SAT" << std::endl;

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
				else if(function_name.compare("pos_1_"+std::to_string(j))==0) {
					model_positions_R1[j] = (int) interp;
				}
				else if(function_name.compare("pos_2_"+std::to_string(j))==0) {
					model_positions_R2[j] = (int) interp;
				}
				else if(function_name.compare("pos_3_"+std::to_string(j))==0) {
					model_positions_R3[j] = (int) interp;
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
				else if(function_name.compare("addRS1A_"+std::to_string(j))==0) {
					model_addRS1A[j] = (int) interp;
				}
				else if(function_name.compare("addRS2A_"+std::to_string(j))==0) {
					model_addRS2A[j] = (int) interp;
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
				else if(function_name.compare("addRS1B_"+std::to_string(j))==0) {
					model_addRS1B[j] = (int) interp;
				}
				else if(function_name.compare("addRS2B_"+std::to_string(j))==0) {
					model_addRS2B[j] = (int) interp;
				}
				else if(function_name.compare("score_"+std::to_string(j))==0) {
					model_score[j] = (int) interp;
				}
				else if(function_name.compare("points_"+std::to_string(j))==0) {
					model_points[j] = (int) interp;
				}

			}
		}
	}

	// Add plan specified by the model to stats
	of_stats << "O" << order_id+1 << ": B" << base_order << "R" << rings_order[0] << "R" << rings_order[1] << "R" << rings_order[2] << "C" << cap_order << " [" << number_required_bases[rings_order[0]] << number_required_bases[rings_order[1]] << number_required_bases[rings_order[2]] << "]";
	if(add_temporal_constraint){
		of_stats << " with bounds " << data.orders(order_id).delivery_period_begin() << "s < o0 < " << data.orders(order_id).delivery_period_end() << "s";
	}
	of_stats << std::endl;
	of_stats << std::endl << "World state" << std::endl;
	of_stats << "R-1 holds " << products_inverted[world_initHold[1]] << ", R-2 holds " << products_inverted[world_initHold[2]] << " and R-3 holds " << products_inverted[world_initHold[3]] << std::endl;
	of_stats << "R-1 is at " << node_names_[world_initPos[1]] << ", R-2 is at " << node_names_[world_initPos[2]] << " and R-3 is at " << node_names_[world_initPos[3]] << std::endl;
	of_stats << "Cap station has " << world_initInside[0] << " and " << products_inverted[world_initOutside[0]] << std::endl;
	of_stats << "Ring station has " << products_inverted[world_initOutside[2]] << std::endl << std::endl;

	for(int j=1; j<plan_horizon+1; ++j){
		of_stats << j <<". ";
		of_stats << "R" << model_robots[j]; //<< " for O" << ((model_actions[j]-1)/index_upper_bound_actions)+1;
		of_stats << " does (A" << model_actions[j] << ") " <<  // << description_actions[((model_actions[j]-1)%index_upper_bound_actions)+1];//  << " (A" << model_actions[j] << ")"; //of_stats  << " and holds " << products_inverted[model_holdB[j]] << " at "<< node_names_[model_positions[j]];
		":[H(" << model_holdA[j] << "-" << model_holdB[j] <<
		"), S2(" << model_insideA[j] << "-" << model_insideB[j] <<
		"), S3(" << model_outputA[j] << "-" << model_outputB[j] <<
		"), S4(" << model_addRS1A[j] << "-" << model_addRS1B[j] <<
		"), S5(" << model_addRS2A[j] << "-" << model_addRS2B[j] <<")]";
		of_stats << " [" << node_names_[model_positions[j]] << "]";
		of_stats << " [" << model_score[j] << "]";
		of_stats << " [" << model_points[j] << "]";
		of_stats << " [" << model_times[j] << "s]" << std::endl; // [R1: " << node_names_[model_positions_R1[j]] <<", R2: " << node_names_[model_positions_R2[j]] << ", R3: " << node_names_[model_positions_R3[j]] << "]" << std::endl;
	}
	// of_stats << "robot_permutation_ [" << robot_permutation_[1] << ", " << robot_permutation_[2] << ", " << robot_permutation_[3] << "]" << std::endl;
	// of_stats << "score of product is " <<  model_score[plan_horizon] << std::endl;

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	// logger->log_info(name(), "Time difference is %f ms", diff); // Measure time in nanoseconds but display in milliseconds for convenience

	of_stats << "Time used for solving: " << diff_ms << " ms, " << diff_m << " m" << std::endl << "__________ __________ __________";
	of_stats.close();

	// Export world state from model
	// initPos
	world_initPos[1] = model_positions_R1[plan_horizon];
	world_initPos[2] = model_positions_R2[plan_horizon];
	world_initPos[3] = model_positions_R3[plan_horizon];
	// initAddRS1
	world_initAddRS1 = model_addRS1B[plan_horizon];
	// initAddRS2
	world_initAddRS2 = model_addRS2B[plan_horizon];
	for(int j=1; j<plan_horizon+1; ++j){
		// initHold
		world_initHold[model_robots[j]] = model_holdB[j];
		// initInside
		world_initInside[model_machines[j]] = model_insideB[j];
		// initOutside
		world_initOutside[model_machines[j]] = model_outputB[j];
	}
	world_points = model_points[plan_horizon];

	// all_actions
	for(int j=1; j<plan_horizon+1; ++j){
		world_all_actions.push_back(model_actions[j]);
	}

	std::cout << "All world actions are: ";
	for(int action: world_all_actions) {
		std::cout << " " << action;
	}
	std::cout << std::endl;

}

// TODO read unsat core to determine time not meet or machine broken
void
ClipsSmtThread::clips_smt_extract_unsat_reason(std::string of_path, std::chrono::high_resolution_clock::time_point begin, std::chrono::high_resolution_clock::time_point end)
{
	// Export stats into clips_smt_thread_stats.txt
	std::ofstream of_stats;
	of_stats.open(of_path, std::ofstream::out | std::ofstream::app);

	// Add time and date stamp to stats
	time_t now = time(0);
	char* dt = ctime(&now);
	of_stats << std::endl << dt << std::endl;
	
	// Add unsat to stats
	of_stats << "Formula is UNSAT" << std::endl;

	// Compute time for solving
	double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count()/1000;
	double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count()/60;

	// logger->log_info(name(), "Time difference is %f ms", diff); // Measure time in nanoseconds but display in milliseconds for convenience

	of_stats << "Time used for solving: " << diff_ms << " ms, " << diff_m << " m" << std::endl << "__________ __________ __________";
	of_stats.close();
}

/*
 * Help methods
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

void ClipsSmtThread::clips_smt_clear_maps() 
{
	model_machines.clear();
	model_times.clear();
	model_positions.clear();
	model_positions_R1.clear();
	model_positions_R2.clear();
	model_positions_R3.clear();
	model_robots.clear();
	model_actions.clear();
	model_holdA.clear();
	model_insideA.clear();
	model_outputA.clear();
	model_addRS1A.clear();
	model_addRS2A.clear();
	model_holdB.clear();
	model_insideB.clear();
	model_outputB.clear();
	model_addRS1B.clear();
	model_addRS2B.clear();
	model_score.clear();

}
