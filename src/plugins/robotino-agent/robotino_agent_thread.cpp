
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

using namespace fawkes;

/** @class RobotinoClipsAgentThread "clips_thread.h"
 * RobotinoClipsAgent environment thread.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoClipsAgentThread::RobotinoClipsAgentThread()
  : Thread("RobotinoClipsAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
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
  clips->add_function("goto-machine", sigc::slot<void, CLIPS::Values>(sigc::mem_fun( *this, &RobotinoClipsAgentThread::clips_goto_machine)));

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
}


void
RobotinoClipsAgentThread::finalize()
{
}


void
RobotinoClipsAgentThread::loop()
{
  if (! started_) {
    clips->assert_fact("(start)");
    started_ = true;
  }

  // might be used to trigger loop events
  // must be cleaned up each loop from within the CLIPS code
  //clips->assert_fact("(time (now))");
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
RobotinoClipsAgentThread::clips_goto_machine(CLIPS::Values nodes)
{
  std::string ms = "";

  CLIPS::Values::iterator v;
  for (v = nodes.begin(); v != nodes.end(); ++v) {
    ms += v->as_string() + " ";
  }

  logger->log_debug(name(), "Decide for closest machine of %s", ms.c_str());
}
