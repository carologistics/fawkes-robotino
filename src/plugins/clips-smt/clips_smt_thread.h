
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

//#ifndef HAVE_LIBZ3
//#  error Cannot use create and solve formula without z3
//#endif

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/tf.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>
#include <clipsmm.h>

#include <z3++.h>
//#include <carl/numbers/numbers.h>
//#include <carl/core/VariablePool.h>
//#include <carl/formula/Formula.h>
//#include <carl/io/SMTLIBStream.h>

#include <vector>
#include <string>
#include <map>
#include <iostream>

//#include "FormulaGenerator/FormulaGenerator.h"
#include "clips_smt_data.h"
#include <llsf_msgs/ClipsSmtData.pb.h>

namespace fawkes {
    class SubProcess;
    class NavGraphStaticListEdgeCostConstraint;
}

class ClipsSmtThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect,
  public fawkes::NavGraph::ChangeListener,
  public fawkes::TransformAspect,
  public fawkes::CLIPSFeature,
  public fawkes::CLIPSFeatureAspect
{
 public:

  typedef float smtTime;
  //time format used for the SMT solving is defined here

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
  z3::expr_vector clips_smt_create_formula();
  z3::check_result clips_smt_solve_formula(z3::expr_vector formula);
  void clips_smt_react_on_result(z3::check_result result);

  // Communication with the agent API
  CLIPS::Value clips_smt_request(void *msgptr, std::string handle);
  CLIPS::Value clips_smt_get_plan(void *msgptr, std::string handle);
  CLIPS::Value clips_smt_done(std::string foo, std::string bar);
  llsf_msgs::ClipsSmtData data;


  // Navgraph
  void clips_smt_compute_distances();
  std::map<std::pair<std::string, std::string>, float> distances_;
  fawkes::NavGraphStaticListEdgeCostConstraint *edge_cost_constraint_;
  std::string cfg_base_frame_;
  std::string cfg_global_frame_;
  //tf::TransformListener* tf_listener_;


  // Test
  void clips_smt_test_z3();
  void clips_smt_test_carl();
  void clips_smt_test_data();
  void clips_smt_test_navgraph();
  SmtData _smtData;

  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

  // SubProcess to call extern binary of z3
  fawkes::SubProcess *proc_z3_;

};

#endif
