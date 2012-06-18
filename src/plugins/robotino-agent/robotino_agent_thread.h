
/***************************************************************************
 *  clips_thread.h - CLIPS aspect provider thread
 *
 *  Created: Sat Jun 16 14:38:21 2012 (Mexico City)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_CLIPS_THREAD_H_
#define __PLUGINS_CLIPS_CLIPS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips.h>
#include <utils/time/time.h>

#include <clipsmm.h>

namespace fawkes {
  class SkillerInterface;
  class RobotinoWorldModelInterface;
}

class RobotinoClipsAgentThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::CLIPSAspect
{
 public:
  RobotinoClipsAgentThread();
  virtual ~RobotinoClipsAgentThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::string clips_get_clips_dir();
  CLIPS::Values clips_now();
  void clips_goto_machine(std::string machine, std::string puck);
  void clips_get_s0();
  void clips_wm_pub(std::string machine, std::string mtype,
                    CLIPS::Values loaded_with, int junk);

 private:
  std::string cfg_clips_dir_;
  bool        cfg_clips_debug_;
  float       cfg_skill_sim_time_;

  fawkes::SkillerInterface *skiller_if_;
  fawkes::RobotinoWorldModelInterface *wm_in_if_;
  fawkes::RobotinoWorldModelInterface *wm_out_if_;

  bool          ctrl_recheck_;

  bool          started_;
  bool          goto_started_;
  fawkes::Time *goto_start_time_;
  std::string   goto_machines_;
  std::string   goto_puck_;
  std::string   goto_skill_string_;

  bool          get_s0_started_;
  fawkes::Time *get_s0_start_time_;
  std::string   get_s0_skill_string_;

  bool          worldmodel_changed_;

};

#endif
