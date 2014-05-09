
/***************************************************************************
 *  agent-monitor-thread.h - Monitoring the CLIPS agents in LLSF via webview
 *
 *  Created: Fri May 09 16:04:13 2014
 *  Copyright  2014  Frederik Zwilling
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

#ifndef __PLUGINS_AGENT_MONITOR_AGENT_MONITOR_THREAD_H_
#define __PLUGINS_AGENT_MONITOR_AGENT_MONITOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_manager.h>

#include <vector>
#include <string>

class AgentMonitorWebRequestProcessor;

class AgentMonitorThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::WebviewAspect,
  public fawkes::CLIPSManagerAspect
{
 public:
  AgentMonitorThread();
  virtual ~AgentMonitorThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::vector<std::string> cfg_proto_dirs_;

  AgentMonitorWebRequestProcessor *web_proc_;
};

#endif
