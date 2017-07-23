
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

#include <z3++.h>
#include <carl/numbers/numbers.h>
// #include <carl/core/VariablePool.h>
#include <carl/formula/Formula.h>
#include <carl/io/SMTLIBStream.h>

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <math.h>
#include <memory>

// #include "FormulaGenerator/FormulaGenerator.h"
#include "FormulaGenerator/GameData.h"
#include "FormulaGenerator/formulaGeneratorTest.cpp"

#include <llsf_msgs/ClipsSmtData.pb.h>


#include <boost/cerrno.hpp>


namespace fawkes {
	class SubProcess;
	class NavGraphStaticListEdgeCostConstraint;
}

class ClipsSmtThread
:	public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::NavGraphAspect,
	public fawkes::NavGraph::ChangeListener,
	//public fawkes::TransformAspect,
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

	// Francescos formula encoding
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
										std::map<std::string, z3::expr>& varInit,
										std::map<std::string, z3::expr>& varO);
	void clips_smt_solve_formula(z3::expr_vector formula);

	void clips_smt_solve_formula_from_smt_file(std::string path);
	void clips_smt_optimize_formula_from_smt_file(std::string path, std::string var);
	void clips_smt_solve_formula_from_fg_smt_file(std::string path, FormulaGenerator fg);

	std::map<int ,std::string> actions_robot_1;
	std::map<int ,std::string> actions_robot_2;
	std::map<int ,std::string> actions_robot_3;

	// Constants for C0-C1
	const int number_actions_c0 = 11;
	const int number_actions_c1 = 18;
	const int plan_horizon = 11;
	const int deadline = 80;
	const int time_to_prep = 5;
	const int time_to_fetch = 5;
	const int time_to_feed = 5;
	const int time_to_disc = 5;
	const int time_to_del = 5;

	// States of machines for C0-C1
	std::map<std::string, int> state1_machines;
	const int min_state1_machines = 0, max_state1_machines = 20;

	std::map<std::string, int> state2_machines;
	const int min_state2_machines = 0, max_state2_machines = 3;

	std::map<std::string, int> state3_machines;
	const int min_state3_machines = -1, max_state3_machines = 20;

	std::map<std::string, int> products;
	const int min_products = 0, max_products = 20;

	std::map<std::string, int> machine_groups;
	const int min_machine_groups = 0, max_machine_groups = 3;

	// Leonards formula encoding
	GameData clips_smt_convert_protobuf_to_gamedata();
	std::vector<int> actions_robot_fg_1;
	std::vector<int> actions_robot_fg_2;
	std::vector<int> actions_robot_fg_3;

	std::vector<std::shared_ptr<Robot>> gamedata_robots;
	std::vector<std::shared_ptr<BaseStation>> gamedata_basestations;
	std::vector<std::shared_ptr<RingStation>> gamedata_ringstations;
	std::vector<std::shared_ptr<CapStation>> gamedata_capstations;
	std::vector<std::shared_ptr<DeliveryStation>> gamedata_deliverystations;

	// Communication with the agent API
	CLIPS::Value clips_smt_request(std::string env_name, std::string handle, void *msgptr);
	CLIPS::Value clips_smt_get_plan(std::string env_name, std::string handle);
	CLIPS::Value clips_smt_done(std::string env_name, std::string bar);
	llsf_msgs::ClipsSmtData data;

	std::string data_env;
	std::string data_handle;

	// Navgraph
	int number_machines;
	int number_bits;
	int number_robots;
	int number_orders;
	void clips_smt_fill_node_names();
	void clips_smt_fill_robot_names();
	void clips_smt_compute_distances_robots();
	void clips_smt_compute_distances_machines();
	std::map<int, std::string> robot_names_;
	std::map<int, std::string> node_names_;
	std::map<std::string, int> node_names_inverted;
	std::map<std::pair<std::string, std::string>, float> distances_;

	fawkes::NavGraphStaticListEdgeCostConstraint *edge_cost_constraint_;
	std::string cfg_base_frame_;
	std::string cfg_global_frame_;

	// Test
	void clips_smt_test_python();
	void clips_smt_test_z3();
	void clips_smt_test_carl();
	void clips_smt_test_formulaGenerator();

	std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;
};

#endif
