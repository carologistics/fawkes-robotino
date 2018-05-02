
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

#include <llsf_msgs/ClipsSmtData.pb.h>

#include <z3++.h>
#include <boost/cerrno.hpp>

/**
 * Constants
 */
const int amount_machines = 10;

// Time
const int deadline = 900;
const int time_to_prep = 5;
const int time_to_fetch = 5;
const int time_to_feed = 5;
const int time_to_disc = 5;
const int time_to_del = 5;
const float velocity_scaling_ = 1;

// Consider delivery window for orders
const bool consider_temporal_constraint = false;

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

	// CLIPSFeature
	virtual void clips_context_init(const std::string &env_name,
				fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	// Navgraph
	virtual void graph_changed() throw();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	protected: virtual void run() { Thread::run(); }

private:
	// Solver logic
	z3::context _z3_context;
	z3::expr_vector clips_smt_encoder();
	void clips_smt_solve_formula(z3::expr_vector formula);
	void clips_smt_optimize_formula(z3::expr_vector formula, std::string var);
	void clips_smt_extract_plan_from_model(z3::model model);

	// Init methods
	void clips_smt_init_game();
	void clips_smt_init_navgraph();
	void clips_smt_init_post();

	// General
	int amount_robots;
	std::string team;

	// Order
	int desired_complexity = 3;
	int order_id;
	int base;
	std::vector<int> rings;
	int cap;
	int delivery_period_begin;
	int delivery_period_end;

	// Navgraph
	std::map<int, std::string> node_names;
	std::map<std::string, int> node_names_inverted;
	std::map<std::pair<std::string, std::string>, float> distances;

	// Rings and Caps
	std::map<std::string, std::string> station_colors;
	std::map<std::string, int> base_colors;
	std::map<int, std::string> base_colors_inverted;
	std::map<std::string, int> rings_colors;
	std::map<int, std::string> rings_colors_inverted;
	std::map<std::string, int> cap_colors;
	std::map<int, std::string> cap_colors_inverted;
	std::map<int, std::string> add_bases_description;
	std::map<std::string, std::string> cap_carrier_colors;
	// How many additional bases are required for rings
	std::map<int, int> rings_req_add_bases;

	// PlanHorizon
	int plan_horizon;
	std::vector<int> amount_min_req_actions; // 6,8,10,12
	std::vector<int> index_upper_bound_actions; // 6,9,11,13
	int amount_req_actions_add_bases; // 2

	// States of machines
	std::map<std::string, int> inside_capstation;
	const int min_inside_capstation = 0, max_inside_capstation = 2;
	const int min_add_bases_ringstation = 0, max_add_bases_ringstation = 3;

	std::map<std::string, int> products;
	std::map<int, std::string> products_inverted;
	const int min_products = -1, max_products = 768;

	std::map<std::string, int> machine_groups;
	const int min_machine_groups = 0, max_machine_groups = 4;

	// Visualization of computed plan
	const int index_delivery_action = 6;
	std::map<int, int> model_machines;
	std::map<int, float> model_times;
	std::map<int, int> model_positions;
	std::map<int, int> model_robots;
	std::map<int, int> model_actions;
	std::map<int, int> model_holdA;
	std::map<int, int> model_insideA;
	std::map<int, int> model_outputA;
	std::map<int, int> model_holdB;
	std::map<int, int> model_insideB;
	std::map<int, int> model_outputB;
	std::map<int, int> model_rew;

	// Communication with the agent API
	CLIPS::Value clips_smt_request(std::string env_name, std::string handle, void *msgptr);
	CLIPS::Value clips_smt_get_plan(std::string env_name, std::string handle);
	CLIPS::Value clips_smt_done(std::string env_name, std::string bar);
	llsf_msgs::ClipsSmtData data;

	std::string data_env;
	std::string data_handle;
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

	fawkes::NavGraphStaticListEdgeCostConstraint *edge_cost_constraint_;
	std::string cfg_base_frame_;
	std::string cfg_global_frame_;

	// Help
	z3::expr getVar(std::map<std::string, z3::expr>& vars, std::string var_id);
	std::vector<bool> shelf_position;
	void initShelf();
	std::string getNextShelf();
};

#endif
