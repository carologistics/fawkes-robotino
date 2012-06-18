
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
  try {
    cfg_clips_debug_ = config->get_bool("/plugins/robotino-agent/clips-debug");
  } catch (Exception &e) {} // ignore, use default

  cfg_clips_dir_ = std::string(SRCDIR) + "/clips/";

  clips->add_function("get-clips-dir", sigc::slot<std::string>(sigc::mem_fun(*this, &RobotinoClipsAgentThread::clips_get_clips_dir)));
  clips->add_function("now", sigc::slot<CLIPS::Values>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_now)));
  clips->add_function("goto-machine", sigc::slot<void, std::string, std::string>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_goto_machine)));
  clips->add_function("get-s0", sigc::slot<void>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_get_s0)));

  if (!clips->batch_evaluate(cfg_clips_dir_ + "init.clp")) {
    logger->log_error(name(), "Failed to initialize CLIPS environment, "
                      "batch file failed.");
    throw Exception("Failed to initialize CLIPS environment, batch file failed.");
  }

  if (cfg_clips_debug_) {
    clips->assert_fact("(enable-debug)");
    clips->refresh_agenda();
    clips->run();
  }

  skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

  if (! skiller_if_->has_writer()) {
    blackboard->close(skiller_if_);
    throw Exception("Skiller has no writer, aborting");
    
  } else if (skiller_if_->exclusive_controller() != 0) {
    blackboard->close(skiller_if_);
    throw Exception("Skiller already has a different exclusive controller");
  }


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

  // might be used to trigger loop events
  // must be cleaned up each loop from within the CLIPS code
  //clips->assert_fact("(time (now))");

  if (goto_started_) {
    Time now(clock);
    if ((now - goto_start_time_) >= 0.1) {
      logger->log_warn(name(), "GOTO is final");
      clips->assert_fact("(goto-final NONE)");
      goto_started_ = false;
    }
  }

  if (get_s0_started_) {
    Time now(clock);
    if ((now - get_s0_start_time_) >= 0.1) {
      logger->log_warn(name(), "GET-S0 is final");
      clips->assert_fact("(get-s0-final)");
      get_s0_started_ = false;
    }
  }

  clips->refresh_agenda();
  clips->run();
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
