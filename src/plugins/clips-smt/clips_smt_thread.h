
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
#include <plugins/clips/aspect/clips_feature.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>

#include <clipsmm.h>

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <math.h>
#include <memory>

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

	/*
	 * Global game variables
	 */
	int number_machines;
	int number_robots;
	int number_orders_protobuf;
	int number_orders;
	int number_orders_c0 = 0;
	int number_orders_c1 = 0;
	int number_orders_c2 = 0;
	int number_orders_c3 = 0;
	int order_id = 0;
	int base_order;
	std::vector<int> rings_order;
	int cap_order;

	std::string team;
	std::map<int, std::string> robot_names_;
	std::map<int, std::string> node_names_;
	std::map<std::string, int> node_names_inverted;
	std::map<std::pair<std::string, std::string>, float> distances_;

	std::map<std::string, std::string> colors_input;
	std::map<std::string, std::string> colors_output;
	std::vector<bool> shelf_position;

	std::map<unsigned, std::string> base_colors;
	std::map<unsigned, std::string> ring_colors;
	std::map<unsigned, std::string> cap_colors;

	/*
	 * Solver
	 */

	// Parameter
	z3::context _z3_context;
	bool _solver_done;

	// Extract z3 formula from .smt file
	z3::expr clips_smt_extract_formula_from_smt_file(std::string path);

	// Solve or optimize z3 formula
	void clips_smt_solve_formula(z3::expr_vector formula);
	void clips_smt_optimize_formula(z3::expr_vector formula, std::string var);
	
	// React on solving/optimizing of z3 formula
	void clips_smt_extract_plan_from_model(z3::model model, std::string of_path, std::chrono::high_resolution_clock::time_point begin, std::chrono::high_resolution_clock::time_point end);
	void clips_smt_extract_unsat_reason(std::string of_path, std::chrono::high_resolution_clock::time_point begin, std::chrono::high_resolution_clock::time_point end);

	/*
	 * Francescos Leofantes formula encoding
	 */
	// Encode formula
	z3::expr_vector clips_smt_encoder(std::map<std::string, z3::expr>& var);

	/*
	 * The constants for orders are defined as follows:
	 *	- index_delivery_action_ci is the index of the action which has to be performed to deliver the product
	 *	- number_required_action_ci is the amount of steps necessary to create the order
	 *	- index_upper_bound_actions_ci is the upper bound of action_ids which have to be considered for the order (note that not all might be needed)
	 *
	 * Then we define variables based on the constants and the number of orders in the regarding complexity
	 *	- index_upper_bound_actions is set to the index_upper_bound_actions_ci of the highest available order
	 *	- number_total_actions is the amount of action_ids covering all orders
	 */

	// Action variables
	int number_required_actions_c0 = 6;
	const int index_upper_bound_actions_c0 = 6;
	int number_required_actions_c1 = 8;
	const int index_upper_bound_actions_c1 = 9;
	int number_required_actions_c2 = 10;
	const int index_upper_bound_actions_c2 = 11;
	int number_required_actions_c3 = 12;
	const int index_upper_bound_actions_c3 = 13;

	const int index_delivery_action = 6;
	int index_upper_bound_actions;
	int number_required_actions;
	int number_total_actions;

	// PlanHorizon variable
	const int plan_horizon = 4;

	// RingColor variables
	std::vector<int> number_required_bases;
	int max_number_required_bases_rs1;
	int max_number_required_bases_rs2;
	
	// Time constants // TODO evalutate realistic constants
	const int deadline = 900;
	const int time_to_prep = 5;
	const int time_to_fetch = 5;
	const int time_to_feed = 5;
	const int time_to_disc = 5;
	const int time_to_del = 5;
	const float velocity_scaling_ = 1;

	// Consider delivery window for orders
	const bool add_temporal_constraint = false;

	/*
	 * States of machines and robots
	 *
	 * Inside refers to the possibility of mounting for the CS
	 * Outside refers to the possibility of retrieving at the CS, RS
	 * AddRS1, AddRS2 refers to the add bases fed into the RS
	 *
	 * Products (similar to Outside) refers to the possibiltiy of robots holding
	 *
	 * MachineGroup refers to the current machine group
	 */
	std::map<std::string, int> inside_capstation;
	const int min_inside_capstation = 0, max_inside_capstation = 2;
	const int min_add_bases_ringstation = 0, max_add_bases_ringstation = 3;

	std::map<std::string, int> products;
	std::map<int, std::string> products_inverted;
	const int min_products = -1, max_products = 766;

	std::map<std::string, int> machine_groups;
	const int min_machine_groups = 0, max_machine_groups = 3;

	// Visualization of computed plan
	std::vector<std::string> description_actions;

	std::map<int, int> model_machines;
	std::map<int, float> model_times;
	std::map<int, int> model_positions;
	std::map<int, int> model_positions_R1;
	std::map<int, int> model_positions_R2;
	std::map<int, int> model_positions_R3;
	std::map<int, int> model_robots;
	std::map<int, int> model_actions;
	std::map<int, int> model_holdA;
	std::map<int, int> model_insideA;
	std::map<int, int> model_outputA;
	std::map<int, int> model_addRS1A;
	std::map<int, int> model_addRS2A;
	std::map<int, int> model_holdB;
	std::map<int, int> model_insideB;
	std::map<int, int> model_outputB;
	std::map<int, int> model_addRS1B;
	std::map<int, int> model_addRS2B;
	std::map<int, int> model_score;
	std::map<int, int> model_points;

	// Visualization of world state
	std::vector<int> world_initHold;
	std::vector<int> world_initPos;
	std::vector<int> world_initInside;
	std::vector<int> world_initOutside;
	int world_initAddRS1;
	int world_initAddRS2;
	std::vector<int> world_all_actions;
	int world_points;
	std::vector<int> world_machines_down;

	/*
	 * Communication with the agent API
	 */

	CLIPS::Value clips_smt_request(std::string env_name, std::string handle, void *msgptr);
	CLIPS::Value clips_smt_get_plan(std::string env_name, std::string handle);
	CLIPS::Value clips_smt_done(std::string env_name, std::string bar);
	llsf_msgs::ClipsSmtData data;

	std::string data_env;
	std::string data_handle;
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;


	/*
	 * Methods to handle given world model information via protobuf
	 */

	// Game information
	void clips_smt_fill_general_info();
	void clips_smt_fill_order_details(int desired_complexity);
	void clips_smt_fill_ringstation_details();
	void clips_smt_fill_ringstation_details_extended();
	void clips_smt_fill_capstation_details();
	// Manage numbers
	void clips_smt_initialize_numbers();
	// Navgraph
	void clips_smt_fill_node_names();
	void clips_smt_fill_robot_names();
	void clips_smt_compute_distances_robots();
	void clips_smt_compute_distances_machines();

	fawkes::NavGraphStaticListEdgeCostConstraint *edge_cost_constraint_;
	std::string cfg_base_frame_;
	std::string cfg_global_frame_;

	// Help
	z3::expr getVar(std::map<std::string, z3::expr>& vars, std::string var_id);
	void initShelf();
	std::string getNextShelf();
	void clips_smt_clear_maps();
};

#endif
