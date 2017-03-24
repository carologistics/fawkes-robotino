
/***************************************************************************
 *  clips_smt_thread.cpp -  Smt feature for CLIPS
 *
 *  Created: Created on Fry Dec 16 14:48 2016 by Igor Nicolai Bongartz
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

#include "clips_smt_thread.h"
#include "../../../fawkes/src/plugins/openprs/utils/proc.h"
//#include <utils/sub_process/proc.h>
#include <core/threading/mutex_locker.h>

#include <navgraph/navgraph.h>
#include <navgraph/yaml_navgraph.h>
//#include <navgraph/constraints/static_list_edge_constraint.h>
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
    // Init navgraph
	  //edge_cost_constraint_ = new NavGraphStaticListEdgeCostConstraint("static-edge-cost");
	  //navgraph->constraint_repo()->register_constraint(edge_cost_constraint_);
	  //navgraph->add_change_listener(this);

    cfg_base_frame_      = config->get_string("/frames/base");
    cfg_global_frame_    = config->get_string("/frames/fixed");


    // Test z3 extern binary
    //proc_z3_ = NULL;
    clips_smt_test_z3();

    // Test python
    proc_python_ = NULL;
    //clips_smt_test_python();
}


void
ClipsSmtThread::finalize()
{
	//navgraph->remove_change_listener(this);
	//navgraph->constraint_repo()->unregister_constraint(edge_cost_constraint_->name());
	//delete edge_cost_constraint_;

    // Handle z3 extern binary
    if (proc_z3_) {
      logger->log_info(name(), "Killing z3 extern bianry proc");
      proc_z3_->kill(SIGINT);
    }
    delete proc_z3_;

    // Handle python
    if (proc_python_) {
      logger->log_info(name(), "Killing python proc");
      proc_python_->kill(SIGINT);
    }
    delete proc_python_;

    // Handle output of formula generation
    //std::remove("carl_formula.smt");

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
                    sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_request), env_name))
  );

  clips->add_function("smt-get-plan",
    sigc::slot<CLIPS::Value, std::string>(
      sigc::bind<0>(
                    sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_get_plan), env_name))
  );

  clips->add_function("smt-done",
    sigc::slot<CLIPS::Value, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_done),
        env_name)
    )
  );

 /**
  clips->add_function("navgraph-block-edge",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_block_edge),
	env_name)
    )
  );

  clips->add_function("navgraph-unblock-edge",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_unblock_edge),
	env_name)
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
void
ClipsSmtThread::clips_smt_load(LockPtr<CLIPS::Environment> &clips)
{
  try {
    const std::vector<NavGraphNode> &nodes = navgraph->nodes();
    const std::vector<NavGraphEdge> &edges = navgraph->edges();

    clips->assert_fact_f("(navgraph (name \"%s\"))", navgraph->name().c_str());

    for (const NavGraphNode &n : nodes) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = n.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-node (name \"%s\") (pos %f %f) (properties %s))",
			   n.name().c_str(), n.x(), n.y(), props_string.c_str());
    }

    for (const NavGraphEdge &e : edges) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = e.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-edge (from \"%s\") (to \"%s\") (directed %s) "
			   "(properties %s))",
			   e.from().c_str(), e.to().c_str(),
			   e.is_directed() ? "TRUE" : "FALSE", props_string.c_str());
    }

  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to assert navgraph, exception follows");
    logger->log_warn(name(), e);
    clips->assert_fact_f("(navgraph-load-fail %s)", *(e.begin()));
  }
}


void
ClipsSmtThread::clips_smt_block_edge(std::string env_name,
					       std::string from, std::string to)
{
  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();

  for (const NavGraphEdge &edge : graph_edges) {
    if (edge.from() == from && edge.to() == to) {
      edge_constraint_->add_edge(edge);
      return;
    }
  }

  logger->log_warn(name(), "Environment %s tried to block edge %s--%s, "
		   "which does not exist in graph", env_name.c_str(),
		   from.c_str(), to.c_str());
}


void
ClipsSmtThread::clips_smt_unblock_edge(std::string env_name,
						 std::string from, std::string to)
{
  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();

  for (const NavGraphEdge &edge : graph_edges) {
    if (edge.from() == from && edge.to() == to) {
      edge_constraint_->remove_edge(edge);
      return;
    }
  }

  logger->log_warn(name(), "Environment %s tried to unblock edge %s--%s, "
		   "which does not exist in graph", env_name.c_str(),
		   from.c_str(), to.c_str());
}
**/

/**
 * Solver logic
 *  - convertToGameData fills the data structures used for formula generation from the protobuf message ClipsSmtData // TODO (Igor) Add
 *  - Create_formula uses knowledge from protobuf to construct an input for the solver
 *  - Solve_formula uses above created formula as input and outputs satisfiability
 *  - React_on_formula let us control the executeable due to the solver solution
 **/

z3::expr_vector
ClipsSmtThread::clips_smt_create_formula()
{
    logger->log_info(name(), "Create z3 formula");

    z3::expr_vector constraints(_z3_context);
	z3::expr_vector variables(_z3_context);
	//std::cout << "Variables.size() " << variables.size() << std::endl;
	for(unsigned i = 0; i < 2; ++i){
		z3::expr var(_z3_context);
		const char* varName = ("x_" + std::to_string(i)).c_str();
		var=_z3_context.real_const(varName);
		variables.push_back(var);
		//std::cout << "Created z3 Variable " << var << std::endl;
		//std::cout << "Variables.size() " << variables.size() << std::endl;
	}

	//std::cout << "Variables.size() " << variables.size() << std::endl;

	z3::expr polynomial1(_z3_context);
	z3::expr polynomial2(_z3_context);
	polynomial1 = _z3_context.int_val(0);
	polynomial2 = _z3_context.int_val(0);

    z3::expr coeff11(_z3_context);
    z3::expr coeff12(_z3_context);
    z3::expr coeff21(_z3_context);
    z3::expr coeff22(_z3_context);
    coeff11=_z3_context.real_val(2);
    coeff12=_z3_context.real_val(1);
    coeff21=_z3_context.real_val(0);
    coeff22=_z3_context.real_val(1);

    z3::expr term11(_z3_context);
    z3::expr term12(_z3_context);
    z3::expr term21(_z3_context);
    z3::expr term22(_z3_context);
    term11=variables[0]*coeff11;
    term12=variables[1]*coeff12;
    term21=variables[0]*coeff21;
    term22=variables[1]*coeff22;

    polynomial1 = term11 + term12;
    polynomial2 = term21 + term22;

	//std::cout << "Constant: " << _constants(i) << std::endl;
	z3::expr constant1 = _z3_context.real_val(2);
	z3::expr constraint1(polynomial1 <= constant1);
	z3::expr constant2 = _z3_context.real_val(2);
	z3::expr constraint2(polynomial1 <= constant2);

	constraints.push_back(constraint1);
	constraints.push_back(constraint2);

	return constraints;
}

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
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = -3; j < data.machines().size()+1; ++j) {
    		z3::expr var(_z3_context);
    		std::string varName = "pos_" + std::to_string(i) + "_" + std::to_string(j);
    		var=_z3_context.int_const((varName).c_str());
    		variables_pos.insert(std::make_pair(varName, var));
        }
	}

    // Variables d_i_j
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = 0; j < data.machines().size()+1; ++j) {
    		z3::expr var(_z3_context);
    		std::string varName = "d_" + std::to_string(i) + "_" + std::to_string(j);
    		var=_z3_context.real_const((varName).c_str());
    		variables_d.insert(std::make_pair(varName, var));
        }
	}

    // Variables m_i
    for(int i = 1; i < data.robots().size()+1; ++i){
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

    // Constraint: d_i_0 = 0
    for(int i = 1; i < data.robots().size()+1; ++i){

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
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = 0; j < data.machines().size()+1; ++j) {

            z3::expr var_d_i_j(_z3_context);
            z3::expr var_d_i_M(_z3_context);

            it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
            if(it_map != variables_d.end()) {
                var_d_i_j = it_map->second;
            }
            else {
                logger->log_error(name(), "var_d_%i_%i not found", i, j);
            }

            it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(data.machines().size()));
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

    // TODO (Igor) Is this constraint necessary?
    // // Constraint: pos_i_j == n for all n=-4, .., m
    // for(it_map = variables_pos.begin(); it_map != variables_pos.end(); ++it_map){
    //     for(int n=-4; n < data.machines().size()+1; ++n) {
    //         z3::expr variable = it_map->second;
    //         z3::expr constraint( variable == n);
    //         //constraints.push_back(constraint);
    //     }
    // }

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
    for(int i = 1; i < data.robots().size()+1; ++i){

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
               z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
               constraint = constraint || (var_pose_i_1 == it_node_names->first && var_d_i_1 == distance_z3);
            }
        }

        constraints.push_back(constraint);
    }

    // Other moves constraint:
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = 2; j < data.machines().size()+1; ++j) {

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

            it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(data.machines().size()));
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

            for(int k = 1; k < data.machines().size()+1; ++k) {
                for(int l = 1; l < data.machines().size()+1; ++l) {
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


    for(int i = 1; i < data.machines().size()+1; ++i){
        z3::expr constraint1(var_false);

        for(int j = 1; j < data.robots().size()+1; ++j) {
            for(int k = 1; k < data.machines().size()+1; ++k) {

                z3::expr var_pos_i_k(_z3_context);

                it_map = variables_pos.find("pos_"+std::to_string(j)+"_"+std::to_string(k));
                if(it_map != variables_pos.end()) {
                    var_pos_i_k = it_map->second;
                }
                else {
                    logger->log_error(name(), "var_pos_%i_%i not found", j, k);

                }

                z3::expr constraint2(var_true);

                for(int u = 1; u < data.robots().size()+1;++u){
                  for(int v = 1; v < data.machines().size()+1; ++v){

                      z3::expr var_pos_u_v(_z3_context);

                      it_map = variables_pos.find("pos_"+std::to_string(u)+"_"+std::to_string(v));
                      if(it_map != variables_pos.end()) {
                          var_pos_u_v = it_map->second;
                      }
                      else {
                          logger->log_error(name(), "var_pos_%i_%i not found", u, v);
                      }

                      z3::expr var_j_u_k_v(_z3_context);
                       if(j== u && k==v) {
                           var_j_u_k_v = _z3_context.bool_val(true);
                       } else {
                           var_j_u_k_v = _z3_context.bool_val(false);
                       }
                       constraint2 = constraint2 && (var_pos_u_v != i || var_j_u_k_v);
                  }
                }
                constraint2 = constraint2 && (var_pos_i_k == i);
                constraint1 = constraint1 || constraint2;
            }
        }
        constraints.push_back(constraint1);
    }

    // Encoding maximum distance <- Problem
    for(int i = 1; i < data.robots().size()+1; ++i){

        z3::expr var_m_i(_z3_context);
        z3::expr var_d_i_M(_z3_context);

        it_map = variables_m.find("m_"+std::to_string(i));
        if(it_map != variables_m.end()) {
            var_m_i = it_map->second;
        }
        else {
            logger->log_error(name(), "var_m_%i not found", i);
        }

        it_map = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(data.machines().size()));
        if(it_map != variables_d.end()) {
            var_d_i_M = it_map->second;
        }
        else {
            logger->log_error(name(), "var_d_%i_M not found", i);
        }

        z3::expr constraint1(var_m_i==0);
        z3::expr constraint2(var_m_i==1);

        for(int j = 1; j < data.robots().size()+1; ++j) {

            if(j!=i) {
                z3::expr var_d_j_M(_z3_context);

                it_map = variables_d.find("d_"+std::to_string(j)+"_"+std::to_string(data.machines().size()));
                if(it_map != variables_d.end()) {
                    var_d_j_M = it_map->second;
                }
                else {
                    logger->log_error(name(), "var_d_%i_M not found", j);
                }

                constraint2 = constraint2 && (var_d_j_M < var_d_i_M);
            }
        }

        constraints.push_back(constraint1 || constraint2);
    }

    return constraints;
}

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

    it_map = variables_d.find("d_"+std::to_string(1)+"_"+std::to_string(data.machines().size()));
    if(it_map != variables_d.end()) {
        d_1_M = it_map->second;
    }
    else {
        logger->log_error(name(), "var_d_1_M not found");
    }

    it_map = variables_d.find("d_"+std::to_string(2)+"_"+std::to_string(data.machines().size()));
    if(it_map != variables_d.end()) {
        d_2_M = it_map->second;
    }
    else {
        logger->log_error(name(), "var_d_2_M not found");
    }

    it_map = variables_d.find("d_"+std::to_string(3)+"_"+std::to_string(data.machines().size()));
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

    if (z3Optimizer.check() == z3::sat){
        logger->log_info(name(), "Finished solving and optimizing formula with SAT");

        z3::model model = z3Optimizer.get_model();

        for(unsigned i=0; i<model.size(); ++i) {
            z3::func_decl function = model[i];
            // std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;

            std::string function_name = function.name().str();
            int interp;
            Z3_get_numeral_int(_z3_context, model.get_const_interp(function), &interp);

            for(int j=1; j<data.machines().size()+1; ++j){
                std::string compare_with_1 = "pos_1_";
                compare_with_1 += std::to_string(j);
                std::string compare_with_2 = "pos_2_";
                compare_with_2 += std::to_string(j);
                std::string compare_with_3 = "pos_3_";
                compare_with_3 += std::to_string(j);

                if(interp>0) {
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
    } else {
        logger->log_info(name(), "Finished solving and optimizing formula with UNSAT");
    }
}

/**
 * Methods for Communication with the agent
 *  - Request performs an activation of the loop function
 *  - Done asks weather the loop is finisihed or not
 **/

CLIPS::Value
ClipsSmtThread::clips_smt_request(std::string env_name, std::string handle, void *msgptr)
{
    // Cast clips msgptr to protobuf_data
    std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
    if (!*m) return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);

    data.CopyFrom(**m); // Use data with subpoint-methods, e.g. data.robots(0).name() OR data.machines().size()
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
 *  - Create formula
 *  - Give formula to solver
 *  - React on solver solution
 **/

void
ClipsSmtThread::loop()
{
    logger->log_info(name(), "Thread performs loop and is running [%d]", running());

    // Compute distances between nodes using navgraph
    clips_smt_fill_node_names();
    clips_smt_compute_distances();

    //Declare variable for the Encoding
    std::map<std::string, z3::expr> variables_pos;
    std::map<std::string, z3::expr> variables_d;
    std::map<std::string, z3::expr> variables_m;

    //z3::expr_vector formula = clips_smt_create_formula();
    z3::expr_vector formula = clips_smt_encoder(variables_pos, variables_d, variables_m);

    // Give it to z3 solver
    clips_smt_solve_formula(variables_pos, variables_d, variables_m,formula);

    logger->log_info(name(), "Thread reached end of loop");

    envs_[data_env].lock();
    envs_[data_env]->assert_fact_f("(smt-plan-complete \"%s\")", data_handle.c_str());
    envs_[data_env].unlock();
}

/**
 * Navgraph Methods
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

    for(int i=0; i<data.machines().size(); ++i){
        std::string machine_name = data.machines(i).name().c_str();
        machine_name += "-I";
        node_names_[i+1] = machine_name;
    }

}

 void
 ClipsSmtThread::clips_smt_compute_distances()
 {
     logger->log_info(name(), "Compute distances between machines using navgraph");

	 MutexLocker lock(navgraph.objmutex_ptr());

    // Compute distances between robotos positions and machines
    for(int r = 0; r < data.robots().size(); ++r){
        std::string robot_node_name = "Robot-";
        robot_node_name += std::to_string(r+1);

        NavGraphNode robot_node(robot_node_name, data.robots(r).pose().x(), data.robots(r).pose().y());
        NavGraphNode from = navgraph->closest_node(robot_node.x(), robot_node.y());

        for (unsigned int i = 1; i < node_names_.size(); ++i) {
            std::pair<std::string, std::string> nodes_pair(robot_node.name(), node_names_[i]);

            NavGraphNode to = navgraph->node(node_names_[i]);
            NavGraphPath p = navgraph->search_path(from, to);

            // logger->log_info(name(), "Distance between node %s and node %s is %f", robot_node.name().c_str(), node_names_[i].c_str(), p.cost()+navgraph->cost(from, robot_node));
            distances_[nodes_pair] = p.cost() + navgraph->cost(from, robot_node); // Use Robot-index to identify robots in distances_
        }
    }

    // Compute distances between unconnected C-ins-in and all other machines
    NavGraphNode ins_node(navgraph->node("C-ins-in"));
    NavGraphNode from = navgraph->closest_node(ins_node.x(), ins_node.y());

    for (unsigned int i = 1; i < node_names_.size(); ++i) {
        std::pair<std::string, std::string> nodes_pair(from.name(), node_names_[i]);

        NavGraphNode to = navgraph->node(node_names_[i]);
        NavGraphPath p = navgraph->search_path(from, to);

        // logger->log_info(name(), "Distance between node %s and node %s is %f", from.name().c_str(), node_names_[i].c_str(), p.cost());
        distances_[nodes_pair] = p.cost() + navgraph->cost(from, ins_node);
    }

    // Compute distances between machines
 	for (unsigned int i = 1; i < node_names_.size(); ++i) {
 		for (unsigned int j = 2; j < node_names_.size(); ++j) {
 			if (i == j) continue;
            std::pair<std::string, std::string> nodes_pair(node_names_[i], node_names_[j]);

 			NavGraphPath p = navgraph->search_path(node_names_[i], node_names_[j]);
            //logger->log_info(name(), "Distance between node %s and node %s is %f", node_names_[i].c_str(), node_names_[j].c_str(), p.cost());
 			distances_[nodes_pair] = p.cost();
 		}
 	}
 }

/**
 * Test methods
 * - z3: Create a SubProcess and fill the z3 solver with the smtlib format of a carl formula
 * - carl: Call a carl method and compare computed to known output
 * - data: Fill own data structure with dummy values
 * - navgraph: Count nodes and edges of navgraph and compute its distances
 **/

 void
 ClipsSmtThread::clips_smt_test_z3()
 {
     /**
    logger->log_info(name(), "Test z3 extern binary");

     carl::Variable x = carl::freshRealVariable("x");
 	 Rational r = 4;
 	 carl::MultivariatePolynomial<Rational> mp = Rational(r*r)*x*x + r*x + r;
 	 carl::Formula<carl::MultivariatePolynomial<Rational>> f(mp, carl::Relation::GEQ);

     std::ofstream outputFile("carl_formula.smt");
     outputFile << carl::outputSMTLIB(carl::Logic::QF_NRA, {f});
     outputFile.close();

     const char *argv[] = { "/home/robosim/z3/bin/z3",
                            "-smt2",
                            "/home/robosim/carl_test/carl_formula.smt",
                            NULL };
     proc_z3_ = new SubProcess("z3 binary", argv[0], argv, NULL, logger);
     proc_z3_->check_proc();
      **/

      /**
      Z3_ast a = Z3_parse_smtlib2_file(_z3_context, "/home/robosim/carl_test/carl_formula.smt", 0, 0, 0, 0, 0, 0);
      z3::expr e(_z3_context, a);

      z3::solver s(_z3_context);
      s.add(e);
      if(s.check() == z3::sat) logger->log_info(name(), "Test of import .smt file into z3 constraint worked");
      **/

 }

 void
 ClipsSmtThread::clips_smt_test_python()
 {
     /**
     logger->log_info(name(), "Test python");

     // Call python script
     const char *argv[] = { "python",
                            "/home/robosim/robotics/fawkes-robotino/src/plugins/clips-smt/Main.py",
                            NULL };
     proc_python_ = new SubProcess("python", argv[0], argv, NULL, logger);
     proc_python_->check_proc();

     //boost::asio::posix::stream_descriptor &stream_descriptor = proc_python_->sd_stdout();
     //stream_descriptor.async_read_some(boost::asio::buffer(data_, 100), handler);
     **/
 }

 void
 ClipsSmtThread::clips_smt_test_carl()
 {
     /**
     // Test carl
     logger->log_info(name(), "Test carl");
     bool b=false;
     if(carl::highestPower(64)==64) b=true;
     logger->log_info(name(), "Hello carl, you are %b", b);
     **/
 }

void
ClipsSmtThread::clips_smt_test_data()
{
    try
    {
      // Test SmtData
      WorkingPiece workingPieceRobot("5345447");
      Robot robot(42, 1, 2, workingPieceRobot);
      _smtData._robots.push_back(robot);

      WorkingPiece workingPieceMachine("13467");
      std::vector<WorkingPieceComponent> inputWpType = {RING_BLUE, RING_GREEN};
      std::vector<WorkingPieceComponent> inputWpContainer = {RING_GREEN};
      WorkingPieceComponent outputWP = RING_ORANGE;
      Machine machine(27,2,3,10,MachineType::cap, workingPieceMachine, inputWpType, inputWpContainer, outputWP);
      _smtData._machines.push_back(machine);
      if (machine.hasRecievedWorkPieceComponent(4)) logger->log_info(name(), "WorkingPieceComponent is already in Machine");

      WorkingPiece targetPieceOrder("24468");
      Order order(30, targetPieceOrder);
      _smtData._currentOrders.push_back(order);


      std::cout << _smtData.toString() << std::endl;
    }
    catch (const runtime_error& error)
    {
        logger->log_error(name(), "Something bad happend, %s", error.what());
    }
}

void
ClipsSmtThread::clips_smt_test_navgraph()
{
    logger->log_info(name(), "Navgraph name: %s", navgraph->name().c_str());
    std::vector<NavGraphEdge> edges = navgraph->edges();
    logger->log_info(name(), "Navgraph has %i many edges", edges.size());
    for(NavGraphEdge edge: edges){
        logger->log_info(name(), "Navgraph has edge from %s to %s", edge.from().c_str(), edge.to().c_str());
    }
    clips_smt_compute_distances();
}


GameData::GameData 
ClipsSmtThread::clips_smt_convert_protobuf_to_gamedata( llsf_msgs::ClipsSmtData data)
{
  
  GameData::GameData _generatorData = GameData::GameData();

  //machines
  for (int i = 0; i < data.machines().size(); i++)
  {
    //name -> id
    GameData::Machine _tmpMachine = GameData::Machine(atoi(data.machines(i).name().c_str()));
    //type -> type
    _tmpMachine.setType(data.machines(i).type());
    //TODO (Lukas) WorkingPiece
    //TODO (Lukas) Distances
    //_generatorData.addMachine(_tmpMachine);
  }


  //Robots
  for (int i = 0; i < data.robots().size(); i++)
  {
    //name -> id
    GameData::Robot _tmpRobot = GameData::Robot(atoi(data.robots(i).name().c_str()));
    //TODO (Lukas) Distances
  }


  //Orders
  for (int i = 0; i < data.orders().size(); i++)
  {
    GameData::Workpiece _tmpWorkPiece = GameData::Workpiece();
    //Color conversions
    int _baseColor = data.orders(i).base_color()+1;
    int _capColor = data.orders(i).cap_color();


    std::vector<int> _ringColors;
    for (int j = 0; j < data.orders(i).ring_colors().size(); j++)
    {
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
