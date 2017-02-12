
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

#include <navgraph/navgraph.h>
#include <navgraph/constraints/static_list_edge_constraint.h>
#include <navgraph/constraints/constraint_repo.h>

#include <llsf_msgs/Pose2D.pb.h> // TODO [Igor] Why is this include necessary for google::protobuf?

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

  clips->add_function("clips_smt_request",
    sigc::slot<CLIPS::Value, void *>(
        sigc::mem_fun(*this, &ClipsSmtThread::clips_smt_request))
  );

  clips->add_function("clips_smt_done",
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

 z3::check_result
 ClipsSmtThread::clips_smt_solve_formula(z3::expr_vector formula)
{
    z3::optimize z3Optimizer(_z3_context);
    //std::cout << "constraints " << formula << std::endl;
    //std::cout << constraints << std::endl << constants << std::endl;
    for (unsigned i = 0; i < formula.size(); i++) {
        z3Optimizer.add(formula[i]);
        std::cout << "CSMT_solve:       Constraint " << formula[i] << std::endl;
    }

    return z3Optimizer.check();
}

void
ClipsSmtThread::clips_smt_react_on_result(z3::check_result result)
{
    std::cout << "CSMT_react:       Result: " << result << std::endl;
}

/**
 * Methods for Communication with the agent
 *  - Request performs an activation of the loop function
 *  - Done asks weather the loop is finisihed or not
 **/

CLIPS::Value
ClipsSmtThread::clips_smt_request(void *msgptr)
{
    // Cast clips msgptr to protobuf_data
    std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
    if (!*m) return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);

    // Wakeup the loop function
    std::cout << "CSMT_request:     Wake up the loop" << std::endl;
    wakeup();

    return CLIPS::Value("Correct-Message", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsSmtThread::clips_smt_done(std::string foo, std::string bar)
{
    std::string answer = "Solver is running (";
    answer += std::to_string(running());
    answer += ")";
    return CLIPS::Value(answer, CLIPS::TYPE_SYMBOL);
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
    std::cout << "CSMT_loop:        Test solving z3 formula with running() "<< running() << std::endl;

    // Build simple formula
    std::cout << "CSMT_loop:        Create z3 formula" << std::endl;
    z3::expr_vector formula = clips_smt_create_formula();

    // Give it to z3 solver
    std::cout << "CSMT_loop:        Solve z3 formula" << std::endl;
    z3::check_result result = clips_smt_solve_formula(formula);

    // Evaluate
    std::cout << "CSMT_loop:        React on solved z3 formula" << std::endl;
    clips_smt_react_on_result(result);
}

/**
 * Test methods
 **/

void
ClipsSmtThread::clips_smt_test(std::string foo, std::string bar)
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
      if (machine.hasRecievedWorkPieceComponent(4)) std::cout<< "CSMT_test:     WorkingPieceComponent is already in Machine" << std::endl;

      WorkingPiece targetPieceOrder("24468");
      Order order(30, targetPieceOrder);
      _smtData._currentOrders.push_back(order);


      std::cout << _smtData.toString() << std::endl;

      // Test carl
      std::cout << "CSMT_test:      Test carl" << std::endl;
      bool b=false;
      if(carl::highestPower(64)==64) b=true;
      std::cout << "CSMT_test:      Hello Carl! You are " << b << std::endl;
    }
    catch (const runtime_error& error)
    {
      std::cout << "CSMT_test:      Someting Bad Happend:" << std::endl;
      std::cout << error.what() << std::endl;
    }
}
