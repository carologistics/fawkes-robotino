
/***************************************************************************
 *  clips_thread.cpp -  RobotinoClipsAgent environment providing Thread
 *
 *  Created: Sat Jun 16 14:40:56 2012 (Mexico City)
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "robotino_agent_thread.h"

#include <interfaces/SkillerInterface.h>
#include <interfaces/RobotinoWorldModelInterface.h>

#define USE_SKILLER false

using namespace fawkes;

/** @class RobotinoClipsAgentThread "clips_thread.h"
 * RobotinoClipsAgent environment thread.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoClipsAgentThread::RobotinoClipsAgentThread()
  : Thread("RobotinoClipsAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
}


/** Destructor. */
RobotinoClipsAgentThread::~RobotinoClipsAgentThread()
{
}


void
RobotinoClipsAgentThread::init()
{
  cfg_clips_debug_ = false;
  cfg_skill_sim_time_ = 2.0;
  cfg_use_sim_ = false;
  cfg_sim_randomize_ = false;
  try {
    cfg_clips_debug_ = config->get_bool("/plugins/robotino-agent/clips-debug");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_use_sim_ = config->get_bool("/plugins/robotino-agent/use-sim");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_sim_randomize_ =
      config->get_bool("/plugins/robotino-agent/sim-randomize-machines");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_skill_sim_time_ =
      config->get_float("/plugins/robotino-agent/skill-sim-time");
  } catch (Exception &e) {} // ignore, use default

  cfg_clips_dir_ = std::string(SRCDIR) + "/clips/";

  skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

  if (! skiller_if_->has_writer()) {
    blackboard->close(skiller_if_);
    throw Exception("Skiller has no writer, aborting");
    
  } else if (skiller_if_->exclusive_controller() != 0) {
    blackboard->close(skiller_if_);
    throw Exception("Skiller already has a different exclusive controller");
  }

  wm_in_if_ =
    blackboard->open_for_reading<RobotinoWorldModelInterface>("Model fll merged");
  wm_out_if_ =
    blackboard->open_for_writing<RobotinoWorldModelInterface>("Agent Belief");


  clips->add_function("get-clips-dir", sigc::slot<std::string>(sigc::mem_fun(*this, &RobotinoClipsAgentThread::clips_get_clips_dir)));
  clips->add_function("now", sigc::slot<CLIPS::Values>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_now)));
  clips->add_function("goto-machine", sigc::slot<void, std::string, std::string>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_goto_machine)));
  clips->add_function("get-s0", sigc::slot<void>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_get_s0)));
  clips->add_function("wm-publish", sigc::slot<void, std::string, std::string, CLIPS::Values, int>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_wm_pub)));

  if (!clips->batch_evaluate(cfg_clips_dir_ + "init.clp")) {
    logger->log_error(name(), "Failed to initialize CLIPS environment, "
                      "batch file failed.");
    blackboard->close(skiller_if_);
    blackboard->close(wm_in_if_);
    blackboard->close(wm_out_if_);
    throw Exception("Failed to initialize CLIPS environment, batch file failed.");
  }

  if (cfg_clips_debug_) {
    clips->assert_fact("(enable-debug)");
  }

  if (cfg_sim_randomize_) {
    clips->assert_fact_f("(enable-sim  %s)",
                         cfg_sim_randomize_ ? "randomize" : "ordered");
  } else {
    clips->assert_fact("(enable-skills)");
  }
  clips->refresh_agenda();
  clips->run();

  ctrl_recheck_ = true;

  started_ = false;
  goto_started_ = false;
  goto_start_time_ = new Time(clock);

  get_s0_started_ = false;
  get_s0_start_time_ = new Time(clock);
}


void
RobotinoClipsAgentThread::finalize()
{
  delete goto_start_time_;
  delete get_s0_start_time_;

  if (skiller_if_->has_writer()) {
    SkillerInterface::ReleaseControlMessage *msg =
      new SkillerInterface::ReleaseControlMessage();
    skiller_if_->msgq_enqueue(msg);
  }

  blackboard->close(skiller_if_);
  blackboard->close(wm_in_if_);
  blackboard->close(wm_out_if_);
}


void
RobotinoClipsAgentThread::loop()
{
  skiller_if_->read();

  if ((skiller_if_->exclusive_controller() == 0) && skiller_if_->has_writer())
  {
    if (ctrl_recheck_) {
      logger->log_info(name(), "Acquiring exclusive skiller control");
      SkillerInterface::AcquireControlMessage *msg =
        new SkillerInterface::AcquireControlMessage();
      skiller_if_->msgq_enqueue(msg);
      ctrl_recheck_ = false;
    } else {
      ctrl_recheck_ = true;
    }
    return;
  }

  if (! started_) {
    clips->assert_fact("(start)");
    started_ = true;
  }

  worldmodel_changed_ = false;

  wm_in_if_->read();
  if (wm_in_if_->changed()) {
    logger->log_debug(name(), "WM Update");

    for (unsigned int i = 1; i <= 10; ++i) {
      // ignore unknown machines, the information can
      // only be as bad as or even worse than ours
      if (wm_in_if_->machine_types(i) == RobotinoWorldModelInterface::TYPE_UNKNOWN)
        continue;

      const char *loaded_with = NULL;
      switch (wm_in_if_->machine_states(i)) {
      case RobotinoWorldModelInterface::EMPTY:
        loaded_with = "(loaded-with)"; break;
      case RobotinoWorldModelInterface::S0_ONLY:
        loaded_with = "(loaded-with S0)"; break;
      case RobotinoWorldModelInterface::S1_ONLY:
        loaded_with = "(loaded-with S1)"; break;
      case RobotinoWorldModelInterface::S2_ONLY:
        loaded_with = "(loaded-with S2)"; break;
      case RobotinoWorldModelInterface::S1_S2:
        loaded_with = "(loaded-with S1 S2)"; break;
      case RobotinoWorldModelInterface::S0_S1:
        loaded_with = "(loaded-with S0 S1)"; break;
      case RobotinoWorldModelInterface::S0_S2:
        loaded_with = "(loaded-with S0 S2)"; break;
      default:
        loaded_with = ""; break;
      }

      clips->assert_fact_f(
        "(wm-ext-update (machine \"m%u\") (mtype %s) %s)",
        i, wm_in_if_->tostring_machine_type_t(wm_in_if_->machine_types(i)),
        loaded_with);
    }
  }

  // might be used to trigger loop events
  // must be cleaned up each loop from within the CLIPS code
  //clips->assert_fact("(time (now))");

  if (goto_started_) {
    Time now(clock);
    if ((now - goto_start_time_) >= cfg_skill_sim_time_) {
      logger->log_warn(name(), "GOTO is final");
      clips->assert_fact("(goto-final NONE)");
      goto_started_ = false;
    }
  }

  if (get_s0_started_) {
    Time now(clock);
    if ((now - get_s0_start_time_) >= cfg_skill_sim_time_) {
      logger->log_warn(name(), "GET-S0 is final");
      clips->assert_fact("(get-s0-final)");
      get_s0_started_ = false;
    }
  }

  clips->refresh_agenda();
  clips->run();

  if (worldmodel_changed_) {
    wm_out_if_->write();
  }
}


CLIPS::Values
RobotinoClipsAgentThread::clips_now()
{
  CLIPS::Values rv;
  fawkes::Time now(clock);
  rv.push_back(now.get_sec());
  rv.push_back(now.get_usec());
  return rv;
}


std::string
RobotinoClipsAgentThread::clips_get_clips_dir()
{
  return cfg_clips_dir_;
}

void
RobotinoClipsAgentThread::clips_get_s0()
{
  logger->log_info(name(), "Get new S0");

  if (USE_SKILLER) {
    try {
      get_s0_skill_string_ = "get_s0()";
      SkillerInterface::ExecSkillContinuousMessage *msg =
        new SkillerInterface::ExecSkillContinuousMessage(get_s0_skill_string_.c_str());
      
      skiller_if_->msgq_enqueue(msg);
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to execute goto skill");
      logger->log_error(name(), e);
    }
  } else {
    get_s0_start_time_->stamp();
    get_s0_started_ = true;
  }
  
}


void
RobotinoClipsAgentThread::clips_goto_machine(std::string machines,
                                             std::string puck)
{
  logger->log_info(name(), "Goto of the machines (%s) with puck %s",
                   machines.c_str(), puck.c_str());

  goto_machines_ = machines;
  goto_puck_ = puck;

  if (USE_SKILLER) {
    try {
      char *sstr_temp;
      if (asprintf(&sstr_temp, "take_puck_to_best{machines=\"%s\", puck=\"%s\"}",
                   machines.c_str(), puck.c_str()) != -1)
      {
        goto_skill_string_ = sstr_temp;
        free(sstr_temp);
      } else {
        throw Exception("Cannot generate skill string.");
      }
      SkillerInterface::ExecSkillContinuousMessage *msg =
        new SkillerInterface::ExecSkillContinuousMessage(goto_skill_string_.c_str());
      
      skiller_if_->msgq_enqueue(msg);
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to execute goto skill");
      logger->log_error(name(), e);
    }
  } else {
    goto_start_time_->stamp();
    goto_started_ = true;
  }
}

void
RobotinoClipsAgentThread::clips_wm_pub(std::string machine, std::string mtype,
                                       CLIPS::Values loaded_with, int junk)
{
  bool loaded_s0 = false, loaded_s1 = false, loaded_s2 = false;
  unsigned int machine_id = 0;

  if (sscanf(machine.c_str(), "m%u", &machine_id) == 1) {
    // valid machine name
    if (mtype == "M1") {
      wm_out_if_->set_machine_types(machine_id, RobotinoWorldModelInterface::M1);
    } else if (mtype == "M2") {
      wm_out_if_->set_machine_types(machine_id, RobotinoWorldModelInterface::M2);
    } else if (mtype == "M3") {
      wm_out_if_->set_machine_types(machine_id, RobotinoWorldModelInterface::M3);
    } else if (mtype == "M1_2") {
      wm_out_if_->set_machine_types(machine_id, RobotinoWorldModelInterface::M1_2);
    } else if (mtype == "M2_3") {
      wm_out_if_->set_machine_types(machine_id, RobotinoWorldModelInterface::M2_3);
    } else if (mtype == "M1_EXPRESS") {
      wm_out_if_->set_machine_types(machine_id,
                                    RobotinoWorldModelInterface::M1_EXPRESS);
      wm_out_if_->set_express_machine(machine_id);
    } else if (mtype == "IGNORED") {
      wm_out_if_->set_machine_types(machine_id,
                                    RobotinoWorldModelInterface::IGNORED);
    }

    CLIPS::Values::iterator v;
    for (v = loaded_with.begin(); v != loaded_with.end(); ++v) {
      if (v->as_string() == "S0") {
        loaded_s0 = true;
      } else if (v->as_string() == "S1") {
        loaded_s1 = true;
      } else if (v->as_string() == "S2") {
        loaded_s2 = true;
      }
    }

    if (! (loaded_s0 || loaded_s1 || loaded_s2)) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::EMPTY);
    } else if (  loaded_s0 && ! loaded_s1 && ! loaded_s2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::S0_ONLY);
    } else if ( ! loaded_s0 &&   loaded_s1 && ! loaded_s2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::S1_ONLY);
    } else if ( ! loaded_s0 && ! loaded_s1 &&   loaded_s2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::S2_ONLY);
    } else if (   loaded_s0 &&   loaded_s1 && ! loaded_s2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::S0_S1);
    } else if (   loaded_s0 && ! loaded_s1 &&   loaded_s2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::S0_S2);
    } else if ( ! loaded_s0 &&   loaded_s1 &&   loaded_s2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::S1_S2);
    } else if (junk == 1) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::CONSUMED_1);
    } else if (junk >= 2) {
      wm_out_if_->set_machine_states(machine_id,
                                     RobotinoWorldModelInterface::CONSUMED_2);
    }

    worldmodel_changed_ = true;
  }
}
