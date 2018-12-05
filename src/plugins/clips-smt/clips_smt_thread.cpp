
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

#include <utils/sub_process/proc.h>
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
    proc_z3_ = NULL;
    //clips_smt_test_z3();

    // Test python
    proc_python_ = NULL;
    //clips_smt_test_python();

    // Init indices_ mapping int to name of machines
    indices_[0] = "C-ins-in";
    indices_[1] = "C-BS-I";
    indices_[2] = "C-BS-O";
    indices_[3] = "C-CS1-I";
    indices_[4] = "C-CS1-O";
    indices_[5] = "C-CS2-I";
    indices_[6] = "C-CS2-O";
    indices_[7] = "C-DS-I";
    indices_[8] = "C-DS-O";
    indices_[9] = "C-RS1-I";
    indices_[10] = "C-RS1-O";
    indices_[11] = "C-RS2-I";
    indices_[12] = "C-RS2-O";
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
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_request))
  );

  clips->add_function("smt-get-plan",
    sigc::slot<CLIPS::Value, std::string>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_get_plan))
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
    z3::expr_vector constraints(_z3_context);

    // std::map<std::string, z3::expr> variables_pos;
    // std::map<std::string, z3::expr> variables_d;
    // std::map<std::string, z3::expr> variables_m;

    std::map<std::string, z3::expr>::iterator it;
    std::map<int, std::string>::iterator it2;

    logger->log_info(name(), "Add variables pos_i_j");
    // Variables pos_i_j
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = -3; j < data.machines().size()+1; ++j) {
    		z3::expr var(_z3_context);
    		std::string varName = "pos_" + std::to_string(i) + "_" + std::to_string(j);
    		var=_z3_context.int_const((varName).c_str());
    		variables_pos.insert(std::make_pair(varName, var));
        }
	}

    logger->log_info(name(), "Add variables d_i_j");
    // Variables d_i_j
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = 0; j < data.machines().size()+1; ++j) {
    		z3::expr var(_z3_context);
    		std::string varName = "d_" + std::to_string(i) + "_" + std::to_string(j);
    		var=_z3_context.real_const((varName).c_str());
    		variables_d.insert(std::make_pair(varName, var));
        }
	}

    logger->log_info(name(), "Add variables m_i");
    // Variables m_i
    for(int i = 1; i < data.robots().size()+1; ++i){
		z3::expr var(_z3_context);
		std::string varName = "m_" + std::to_string(i);
		var=_z3_context.int_const((varName).c_str());
        variables_m.insert(std::make_pair(varName, var));
	}
    logger->log_info(name(), "Finished creating variables");

    logger->log_info(name(), "Add constraint: d_i_0 = 0");
    // Constraint: d_i_0 = 0
    for(int i = 1; i < data.robots().size()+1; ++i){
        it = variables_d.find("d_"+std::to_string(i)+"_0");
        if(it != variables_d.end()) {

            z3::expr variable = it->second;
            z3::expr constraint( variable == 0);
            constraints.push_back(constraint);
        }
        else {
            std::cout << " Variable not found!" << std::endl;
        }
	}

    logger->log_info(name(), "Add constraint: d_i_j <= d_i_m");
    // Constraint: d_i_j <= d_i_m
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = 0; j < data.machines().size()+1; ++j) {

            z3::expr variable1(_z3_context);
            z3::expr variable2(_z3_context);

            it = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
            if(it != variables_d.end()) {
                variable1 = it->second;
            }
            else {
                std::cout << " Variable1 not found!" << std::endl;
            }

            it = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(data.machines().size()));
            if(it != variables_d.end()) {
                variable2 = it->second;
            }
            else {
                std::cout << " Variable not found!" << std::endl;
            }

            // TODO (Igor) Check if variables are set properly

            z3::expr constraint( variable1 <= variable2);
            constraints.push_back(constraint);
        }
	}

    // logger->log_info(name(), "Add constraint: pos_i_j == n for all n=-4, .., m");
    // // Constraint: pos_i_j == n for all n=-4, .., m
    // for(it = variables_pos.begin(); it != variables_pos.end(); ++it){
    //     for(int n=-4; n < data.machines().size()+1; ++n) {
    //         z3::expr variable = it->second;
    //         z3::expr constraint( variable == n);
    //         //constraints.push_back(constraint);
    //     }
    // }

    logger->log_info(name(), "Add constraint: pos_1_n == -1 for all n=-3,..., -1");
    // Constraint: pos_1_n == -1 for all n=-3,..., -1
    for(it = variables_pos.begin(); it != variables_pos.end(); ++it){
        if(it->first.compare("pos_1_-1")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -1);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_1_-2")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -1);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_1_-3")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -1);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_2_-1")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -1);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_2_-2")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -2);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_2_-3")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -2);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_3_-1")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -1);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_3_-2")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -2);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_3_-3")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == -3);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_1_0")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == 0);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_2_0")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == 0);
            constraints.push_back(constraint);
        }
        else if(it->first.compare("pos_3_0")==0){
            z3::expr variable = it->second;
            z3::expr constraint( variable == 0);
            constraints.push_back(constraint);
        }
    }

    logger->log_info(name(), "Add first move constraint");
    // First move constraint:
    for(int i = 1; i < data.robots().size()+1; ++i){

        z3::expr variable1(_z3_context);
        z3::expr variable2(_z3_context);

        it = variables_pos.find("pos_"+std::to_string(i)+"_1");
        if(it != variables_pos.end()) {
            variable1 = it->second;
        }
        else {
            std::cout << " Variable not found!" << std::endl;
        }

        it = variables_d.find("d_"+std::to_string(i)+"_1");
        if(it != variables_d.end()) {
            variable2 = it->second;
        }
        else {
            std::cout << " Variable not found!" << std::endl;
        }

        z3::expr var(_z3_context);
        var = _z3_context.bool_val(false);
        z3::expr constraint2(var);

        for(it2 = indices_.begin(); it2 != indices_.end(); ++it2){
            float distance = distances_[std::make_pair("C-ins-in", it2->second)];
            z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
            std::cout << "Distance between C-ins-in and " << it2->second << " is " << distance << "/" << distance_z3 << std::endl;

            constraint2 = constraint2 || (variable1 == it2->first && variable2 == distance_z3);
        }

        constraints.push_back(constraint2);
    }

    logger->log_info(name(), "Add other moves constraint");
    // Other moves constraint:
    for(int i = 1; i < data.robots().size()+1; ++i){
        for(int j = 2; j < data.machines().size()+1; ++j) {

            z3::expr variable1(_z3_context);
            z3::expr variable2(_z3_context);
            z3::expr variable3(_z3_context);
            z3::expr variable4(_z3_context);
            z3::expr variable5(_z3_context);

            it = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j));
            if(it != variables_pos.end()) {
                variable1 = it->second;
            }
            else {
                std::cout << " Variable not found!" << std::endl;
            }

            it = variables_pos.find("pos_"+std::to_string(i)+"_"+std::to_string(j-1));
            if(it != variables_pos.end()) {
                variable2 = it->second;
            }
            else {
                std::cout << " Variable not found!" << std::endl;
            }

            it = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j));
            if(it != variables_d.end()) {
                variable3 = it->second;
            }
            else {
                std::cout << " Variable not found!" << std::endl;
            }

            it = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(j-1));
            if(it != variables_d.end()) {
                variable4 = it->second;
            }
            else {
                std::cout << " Variable not found!" << std::endl;
            }

            it = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(data.machines().size()));
            if(it != variables_d.end()) {
                variable5 = it->second;
            }
            else {
                std::cout << " Variable not found!" << std::endl;
            }

            z3::expr constraint1(variable1==-4 && variable5==variable4);
            z3::expr var(_z3_context);
            var = _z3_context.bool_val(false);
            z3::expr constraint2(var);

            // z3::expr constraint1( variable2 != 0 && variable2 != -1 && variable2 != -2 && variable2 != -3 );
            // constraints.push_back(constraint1);
            //
            // z3::expr constraint2( variable1 != -4 || (variable2 == -4 && variable4 == variable3));
            // constraints.push_back(constraint2);
            //
            // z3::expr constraint3( variable2 != -4 || variable4 == variable3);
            // constraints.push_back(constraint3);
            //
            for(int k = 1; k < data.machines().size()+1; ++k) {
                for(int l = 1; l < data.machines().size()+1; ++l) {
                    if(k!=l) {
                        float distance = distances_[std::make_pair(indices_[k], indices_[l])];
                        z3::expr distance_z3 = _z3_context.real_val((std::to_string(distance)).c_str());
                        constraint2 = constraint2 || (variable2==k && variable1==l && variable3==(variable4+distance_z3));
                    }
                }
            }

            constraints.push_back(constraint1 || constraint2);
        }
    }


    logger->log_info(name(), "Add constraint not to visit the same machine twice");
    // Robot can not visit the same machine twice <- Problem
    // Added initialization of constraint to false.
    // it might be the case that concat on uninitialized expr gives problems.
    z3::expr var(_z3_context);
    var = _z3_context.bool_val(false);

    for(int i = 1; i < data.machines().size()+1; ++i){
        z3::expr constraintNotVisitTwice(var);

        for(int j = 1; j < data.robots().size()+1; ++j) {
            for(int k = 1; k < data.machines().size()+1; ++k) {
                z3::expr variable1(_z3_context);

                it = variables_pos.find("pos_"+std::to_string(j)+"_"+std::to_string(k));
                if(it != variables_pos.end()) {
                    variable1 = it->second;
                    logger->log_info(name(), "Variable1 added");
                }
                else {
                    logger->log_info(name(), "Variable1 not found");

                }

                z3::expr var(_z3_context);
                var = _z3_context.bool_val(true);
                z3::expr constraint2(var);
                for(int u = 1; u < data.robots().size()+1;++u){
                  for(int v = 1; v < data.machines().size()+1; ++v){
                      z3::expr variable2(_z3_context);

                      it = variables_pos.find("pos_"+std::to_string(u)+"_"+std::to_string(v));
                      if(it != variables_pos.end()) {
                          variable2 = it->second;
                          logger->log_info(name(), "Variable2 added");
                      }
                      else {
                          logger->log_info(name(), "Variable2 not found");
                      }
                      z3::expr var2(_z3_context);
                       if(j== u && k==v) {
                           var2 = _z3_context.bool_val(true);
                       } else {
                           var2 = _z3_context.bool_val(false);
                       }
                       constraint2 = constraint2 && (variable2 != i || var2);
                       logger->log_info(name(), "Constraint2 updated");
                  }
                }
                constraint2 = constraint2 && (variable1 == i);
                constraintNotVisitTwice = constraintNotVisitTwice || constraint2;
                logger->log_info(name(), "ConstraintNotVisitTwice updated");

            }
        }
        constraints.push_back(constraintNotVisitTwice);
    }

    logger->log_info(name(), "ConstraintNotVisitTwice added");


    logger->log_info(name(), "Add constraint encoding maximum distance");
    // Encoding maximum distance <- Problem
    for(int i = 1; i < data.robots().size()+1; ++i){

        z3::expr variable1(_z3_context);
        z3::expr variable2(_z3_context);

        it = variables_m.find("m_"+std::to_string(i));
        if(it != variables_m.end()) {
            variable1 = it->second;
            logger->log_info(name(), "Variable1 added");
        }
        else {
            logger->log_info(name(), "Variable1 not found");
        }

        it = variables_d.find("d_"+std::to_string(i)+"_"+std::to_string(data.machines().size()));
        if(it != variables_d.end()) {
            variable2 = it->second;
            logger->log_info(name(), "Variable2 added");
        }
        else {
            logger->log_info(name(), "Variable2 not found");
        }

        // z3::expr var(_z3_context);
        // var = _z3_context.bool_val(true);

        z3::expr constraint1(variable1==0);
        logger->log_info(name(), "Constraint1 updated");
        z3::expr constraint2(variable1==1);
        logger->log_info(name(), "Constraint2 updated");

        for(int j = 1; j < data.robots().size()+1; ++j) {

            if(j!=i) {
                z3::expr variable3(_z3_context);

                it = variables_d.find("d_"+std::to_string(j)+"_"+std::to_string(data.machines().size()));
                if(it != variables_d.end()) {
                    variable3 = it->second;
                    logger->log_info(name(), "Variable3 added");
                }
                else {
                    logger->log_info(name(), "Variable3 not found");
                }

                constraint2 = constraint2 && (variable3 < variable2);
                logger->log_info(name(), "Constraint2 updated");
            }
        }

        constraints.push_back(constraint1 || constraint2);
    }


    logger->log_info(name(), "Finished collecting constraints");
    return constraints;
}

 z3::check_result
 ClipsSmtThread::clips_smt_solve_formula(std::map<std::string, z3::expr>& variables_pos, std::map<std::string, z3::expr>& variables_d, std::map<std::string, z3::expr>& variables_m,z3::expr_vector formula)
{
    z3::optimize z3Optimizer(_z3_context);

    std::map<std::string, z3::expr>::iterator it;
    //std::cout << "constraints " << formula << std::endl;
    //std::cout << constraints << std::endl << constants << std::endl;
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

    it = variables_d.find("d_"+std::to_string(1)+"_"+std::to_string(data.machines().size()));
    if(it != variables_d.end()) {
        d_1_M = it->second;
    }
    else {
        std::cout << " Variable d_1_M not found!" << std::endl;
    }

    it = variables_d.find("d_"+std::to_string(2)+"_"+std::to_string(data.machines().size()));
    if(it != variables_d.end()) {
        d_2_M = it->second;
    }
    else {
        std::cout << " Variable d_2_M not found!" << std::endl;
    }

    it = variables_d.find("d_"+std::to_string(3)+"_"+std::to_string(data.machines().size()));
    if(it != variables_d.end()) {
        d_3_M = it->second;
    }
    else {
        std::cout << " Variable d_3_M not found!" << std::endl;
    }

    it = variables_m.find("m_"+std::to_string(1));
    if(it != variables_m.end()) {
        m_1 = it->second;
    }
    else {
        std::cout << " Variable m_1 not found!" << std::endl;
    }

    it = variables_m.find("m_"+std::to_string(2));
    if(it != variables_m.end()) {
        m_2 = it->second;
    }
    else {
        std::cout << " Variable m_2 not found!" << std::endl;
    }

    it = variables_m.find("m_"+std::to_string(3));
    if(it != variables_m.end()) {
        m_3 = it->second;
    }
    else {
        std::cout << " Variable m_3 not found!" << std::endl;
    }

    //z3Optimizer.minimize(m_1*d_1_M + m_2*d_2_M + m_3*d_3_M);
    //z3Optimizer.minimize(d_1_M + d_2_M + d_3_M);

    //TODO
    // std::ofstream outputFile("smt_benchmarkPlugin.smt");
	// outputFile << z3Optimizer.to_smt2();
	// outputFile.close();

    std::cout << "Result of z3Optimizer.check() is " << z3Optimizer.check() << std::endl;
    if (z3Optimizer.check() == z3::sat){
        z3::model model = z3Optimizer.get_model();
    }
    return z3Optimizer.check();
}

void
ClipsSmtThread::clips_smt_react_on_result(z3::check_result result)
{
    logger->log_info(name(), "Result on solving formula is %s", result);
}

/**
 * Methods for Communication with the agent
 *  - Request performs an activation of the loop function
 *  - Done asks weather the loop is finisihed or not
 **/

CLIPS::Value
ClipsSmtThread::clips_smt_request(std::string handle, void *msgptr)
{
    // Cast clips msgptr to protobuf_data
    std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
    if (!*m) return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);

    data.CopyFrom(**m); // Use data with subpoint-methods, e.g. data.robots(0).name() OR data.machines().size()

    // Use handle to associate request to solution
    logger->log_info(name(),"Handle request: %s", handle.c_str());

    // Wakeup the loop function
    logger->log_info(name(), "At end of request, wake up the loop" );
    wakeup();

    return CLIPS::Value("RUNNING", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsSmtThread::clips_smt_get_plan(std::string handle)
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

	    action = plan->add_actions();
	    action->set_name("move");
	    param = action->add_params();
	    param->set_key("to");
	    param->set_value("C-BS-I");

	    action = plan->add_actions();
	    action->set_name("move");
	    param = action->add_params();
	    param->set_key("to");
	    param->set_value("C-CS1-I");
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

    logger->log_info(name(), "Compute distances between nodes in navgraph");
    clips_smt_compute_distances();

    // Build simple formula
    logger->log_info(name(), "Create z3 formula");
    //Declare variable for the Encoding
    std::map<std::string, z3::expr> variables_pos;
    std::map<std::string, z3::expr> variables_d;
    std::map<std::string, z3::expr> variables_m;

    //z3::expr_vector formula = clips_smt_create_formula();
    z3::expr_vector formula = clips_smt_encoder(variables_pos, variables_d, variables_m);

    // Give it to z3 solver
    logger->log_info(name(), "Solve z3 formula");
    z3::check_result result = clips_smt_solve_formula(variables_pos, variables_d, variables_m,formula);

    // Evaluate
    logger->log_info(name(), "React on solved z3 formula");
    clips_smt_react_on_result(result);

    logger->log_info(name(), "Thread reached end of loop");

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
 ClipsSmtThread::clips_smt_compute_distances()
 {
	 MutexLocker lock(navgraph.objmutex_ptr());
	 
    std::vector<std::string> nodes = {
        "C-ins-in",
        "C-BS-I","C-BS-O",
        "C-CS1-I","C-CS1-O","C-CS2-I","C-CS2-O",
        "C-DS-I","C-DS-O",
        "C-RS1-I","C-RS1-O","C-RS2-I","C-RS2-O"
    };

    fawkes::tf::Stamped<fawkes::tf::Pose> pose_;
    tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose_);
    NavGraphNode from = navgraph->closest_node(pose_.getOrigin().x(), pose_.getOrigin().y());
    // std::cout << "CSMT_test: 'from' node is " << from.name() << " with coordinates: (" << from.x() << ", " << from.y()
    // << ") // Closest to ("<< pose_.getOrigin().x() << "," << pose_.getOrigin().y() << ")" << std::endl;

    // for (unsigned int i = 0; i < nodes.size(); ++i) {
    //     std::pair<std::string, std::string> nodes_pair(from.name(), nodes[i]);
    //
    //     NavGraphNode to = navgraph->node(nodes[i]);
    //     NavGraphPath p = navgraph->search_path(from, to);
    //
    //     logger->log_info(name(), "Distance between node %s and node %s is %f", from.name().c_str(), nodes[i].c_str(), p.cost());
    //     distances_[nodes_pair] = p.cost();
    // }

    distances_[std::make_pair("C-ins-in","C-BS-I")] = 5;
    distances_[std::make_pair("C-ins-in","C-BS-O")] = 5;
    distances_[std::make_pair("C-ins-in","C-CS1-I")] = 5;
    distances_[std::make_pair("C-ins-in","C-CS1-O")] = 5;
    distances_[std::make_pair("C-ins-in","C-CS2-I")] = 5;
    distances_[std::make_pair("C-ins-in","C-CS2-O")] = 5;
    distances_[std::make_pair("C-ins-in","C-DS-I")] = 5;
    distances_[std::make_pair("C-ins-in","C-DS-O")] = 5;
    distances_[std::make_pair("C-ins-in","C-RS1-I")] = 5;
    distances_[std::make_pair("C-ins-in","C-RS1-O")] = 5;
    distances_[std::make_pair("C-ins-in","C-RS2-I")] = 5;
    distances_[std::make_pair("C-ins-in","C-RS2-O")] = 5;



 	for (unsigned int i = 1; i < nodes.size(); ++i) {
 		for (unsigned int j = 2; j < nodes.size(); ++j) {
 			if (i == j) continue;
            std::pair<std::string, std::string> nodes_pair(nodes[i], nodes[j]);

 			NavGraphPath p = navgraph->search_path(nodes[i], nodes[j]);
            logger->log_info(name(), "Distance between node %s and node %s is %f", nodes[i].c_str(), nodes[j].c_str(), p.cost());
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
