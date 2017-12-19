
/***************************************************************************
 *  clips_smt_thread.h - Smt feature for CLIPS
 *
 *  Created: Created on Fry Dec 16 14:44 2016 by Igor Nicolai Bongartz
 *
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

#ifndef __PLUGINS_CLIPS_SMT_CLIPS_SMT_THREAD_H_
#define __PLUGINS_CLIPS_SMT_CLIPS_SMT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
//#include <aspect/tf.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>

#include <clipsmm.h>
#ifdef TRUE
#  undef TRUE
#endif
#ifdef FALSE
#  undef FALSE
#endif

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <math.h>
#include <memory>

// Leonard Korp approach
// #include <carl/numbers/numbers.h>
// #include <carl/core/VariablePool.h>
// #include <carl/formula/Formula.h>
// #include <carl/io/SMTLIBStream.h>
// #include "FormulaGenerator/FormulaGenerator.h"
// #include "FormulaGenerator/GameData.h"
// #include "FormulaGenerator/formulaGeneratorTest.cpp"

#include <llsf_msgs/ClipsSmtData.pb.h>
#include <z3++.h>
#include <boost/cerrno.hpp>


namespace fawkes {
	class NavGraphStaticListEdgeCostConstraint;
}

class ClipsSmtThread
:	public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::NavGraphAspect,
	public fawkes::NavGraph::ChangeListener,
	public fawkes::CLIPSFeature,
	public fawkes::CLIPSFeatureAspect
{
public:
	ClipsSmtThread();
	virtual ~ClipsSmtThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for CLIPSFeature
	virtual void clips_context_init(const std::string &env_name,
				fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	virtual void graph_changed() throw();


	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	protected: virtual void run() { Thread::run(); }

 /**
private:
	void clips_smt_load(fawkes::LockPtr<CLIPS::Environment> &clips);
	void clips_smt_block_edge(std::string env_name, std::string from, std::string to);
	void clips_smt_unblock_edge(std::string env_name, std::string from, std::string to);
**/
private:
	// Solver logic
	z3::context _z3_context;
	bool _solver_done;

	z3::expr clips_smt_extract_formula_from_smt_file(std::string path);

	void clips_smt_solve_formula(z3::expr_vector formula);
	void clips_smt_optimize_formula(z3::expr_vector formula, std::string var);
	// void clips_smt_solve_formula_from_fg_smt_file(std::string path, FormulaGenerator fg);
	
	void clips_smt_extract_plan_from_model(z3::model model, std::string of_path, std::chrono::high_resolution_clock::time_point begin, std::chrono::high_resolution_clock::time_point end);
	void clips_smt_extract_unsat_reason(std::string of_path, std::chrono::high_resolution_clock::time_point begin, std::chrono::high_resolution_clock::time_point end);

	// Francescos Leofantes formula encoding
	z3::expr_vector clips_smt_encoder(std::map<std::string, z3::expr>& varStartTime,
										std::map<std::string, z3::expr>& varRobotDuration,
										std::map<std::string, z3::expr>& varRobotPosition,
										std::map<std::string, z3::expr>& varMachineDuration,
										std::map<std::string, z3::expr>& varR,
										std::map<std::string, z3::expr>& varA,
										std::map<std::string, z3::expr>& varM,
										std::map<std::string, z3::expr>& varHold,
										std::map<std::string, z3::expr>& varS,
										std::map<std::string, z3::expr>& varRew,
										std::map<std::string, z3::expr>& varInit);

	/*
	 * The constants for orders are defined as follows:
	 *	- index_delivery_action_ci is the index of the action which have to be performed to deliver the product
	 *	- number_required_action_ci is the amount of steps necessary to create the order
	 *	- index_upper_bound_actions_ci is the upper bound of action_ids which have to be considered for the order (note that not all might be needed)
	 *
	 * Then we define variables based on the constants and the number of orders in the regarding complexity
	 *	- index_upper_bound_actions is set to the index_upper_bound_actions_ci of the highest available order
	 *	- number_total_actions is the amount of action_ids covering all orders
	 */

	// c0
	int number_required_actions_c0 = 6;
	const int index_upper_bound_actions_c0 = 6;

	// c1
	int number_required_actions_c1 = 8;
	const int index_upper_bound_actions_c1 = 9;

	// c2
	int number_required_actions_c2 = 10;
	const int index_upper_bound_actions_c2 = 11;

	// c3
	int number_required_actions_c3 = 12;
	const int index_upper_bound_actions_c3 = 13;

	// All ci
	int index_delivery_action = 6;
	int index_upper_bound_actions;
	int number_required_actions;
	int number_total_actions;
	int plan_horizon;

	// RingColor constants
	std::vector<int> number_required_bases;
	
	// Time constants // TODO evalutate realistic constants
	const int deadline = 900;
	const int time_to_prep = 5;
	const int time_to_fetch = 5;
	const int time_to_feed = 5;
	const int time_to_disc = 5;
	const int time_to_del = 5;

	// Consider delivery window for orders
	const bool add_temporal_constraint = false;

	// States of machines
	// State1 refers to the prepared state of a machine (DS, CS, RS, BS)
	std::map<std::string, int> state1_machines;
	const int min_state1_machines = 0, max_state1_machines = 13;

	// State2 refers to the possibility of mounting (CS, RS)
	std::map<std::string, int> state2_machines;
	const int min_state2_machines = 0, max_state2_machines = 6;

	// State3 refers to the possibilty of retrieving (CS, RS)
	std::map<std::string, int> state3_machines;
	const int min_state3_machines = -1, max_state3_machines = 765;

	// State4,5 refers to the number of bases fed into the ring stations RS1 and RS2
	const int min_state45_machines = 0, max_state45_machines = 3;

	// Products refers to all possible products
	std::map<std::string, int> products;
	std::map<int, std::string> products_inverted;
	const int min_products = 0, max_products = 765;

	// Machine_groups refers to all possible machines (DS, CS, RS, BS)
	std::map<std::string, int> machine_groups;
	const int min_machine_groups = 0, max_machine_groups = 3;

	// Visualization of computed plan
	std::vector<std::string> description_actions;
	std::map<int, float> model_times;
	std::map<int, int> model_positions;
	std::map<int, int> model_positions_R1;
	std::map<int, int> model_positions_R2;
	std::map<int, int> model_positions_R3;
	std::map<int, int> model_robots;
	std::map<int, int> model_actions;
	std::map<int, int> model_holdA;
	std::map<int, int> model_state1A;
	std::map<int, int> model_state2A;
	std::map<int, int> model_state3A;
	std::map<int, int> model_state4A;
	std::map<int, int> model_state5A;
	std::map<int, int> model_holdB;
	std::map<int, int> model_state1B;
	std::map<int, int> model_state2B;
	std::map<int, int> model_state3B;
	std::map<int, int> model_state4B;
	std::map<int, int> model_state5B;

	// Leonard Korps formula encoding
	// GameData clips_smt_convert_protobuf_to_gamedata();
	// std::vector<int> actions_robot_fg_1;
	// std::vector<int> actions_robot_fg_2;
	// std::vector<int> actions_robot_fg_3;

	// std::vector<std::shared_ptr<Robot>> gamedata_robots;
	// std::vector<std::shared_ptr<BaseStation>> gamedata_basestations;
	// std::vector<std::shared_ptr<RingStation>> gamedata_ringstations;
	// std::vector<std::shared_ptr<CapStation>> gamedata_capstations;
	// std::vector<std::shared_ptr<DeliveryStation>> gamedata_deliverystations;

	// Communication with the agent API
	CLIPS::Value clips_smt_request(std::string env_name, std::string handle, void *msgptr);
	CLIPS::Value clips_smt_get_plan(std::string env_name, std::string handle);
	CLIPS::Value clips_smt_done(std::string env_name, std::string bar);
	llsf_msgs::ClipsSmtData data;

	std::string data_env;
	std::string data_handle;
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

	// Global variables
	int number_machines;
	int number_robots;
	int number_orders_protobuf;
	int number_orders;
	int number_orders_c0 = 0;
	int number_orders_c1 = 0;
	int number_orders_c2 = 0;
	int number_orders_c3 = 0;
	int order_id = 0;
	std::string team;

	int base_order;
	std::vector<int> rings_order;
	int cap_order;

	// Protobuf
	void clips_smt_fill_general_info();
	void clips_smt_fill_order_details();
	void clips_smt_fill_ringstation_details();
	void clips_smt_fill_capstation_details();
	// Manage numbers
	void clips_smt_initialize_numbers();
	// Navgraph
	void clips_smt_fill_node_names();
	void clips_smt_fill_robot_names();
	void clips_smt_compute_distances_robots();
	void clips_smt_compute_distances_machines();

	std::map<int, std::string> robot_names_;
	std::map<int, int> robot_permutation_;
	std::map<int, std::string> node_names_;
	std::map<std::string, int> node_names_inverted;
	std::map<std::pair<std::string, std::string>, float> distances_;
	float velocity_scaling_ = 1;

	std::map<std::string, std::string> colors_input;
	std::map<std::string, std::string> colors_output;
	std::vector<bool> shelf_position;

	fawkes::NavGraphStaticListEdgeCostConstraint *edge_cost_constraint_;
	std::string cfg_base_frame_;
	std::string cfg_global_frame_;

	// Test
	// void clips_smt_test_z3();
	// void clips_smt_test_formulaGenerator();

	// Help
	z3::expr getVar(std::map<std::string, z3::expr>& vars, std::string var_id);
	std::string getCapColor(int product_id);
	std::string getBaseColor(int product_id);
	std::string getRingColor(int product_id);
	void initShelf();
	std::string getNextShelf();
};

#endif
