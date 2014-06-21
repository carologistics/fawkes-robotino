
/***************************************************************************
 *  agent-monitor-thread.cpp - Monitoring the CLIPS agents in LLSF via webview
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

#include "agent-monitor-thread.h"
#include "agent-monitor-processor.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>

using namespace fawkes;

#define URL_PREFIX "/agent-monitor"

/** @class AgentMonitorThread "agent-monitor-thread.h"
 * Monitoring the CLIPS agents in LLSF via webview
 * @author Frederik Zwilling
 */

/** Constructor. */
AgentMonitorThread::AgentMonitorThread()
  : Thread("AgentMonitorThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
AgentMonitorThread::~AgentMonitorThread()
{
}


void
AgentMonitorThread::init()
{
  web_proc_  = new AgentMonitorWebRequestProcessor(clips_env_mgr, logger, URL_PREFIX, config, blackboard);
  webview_url_manager->register_baseurl(URL_PREFIX, web_proc_);
  webview_nav_manager->add_nav_entry(URL_PREFIX, "Agent-Monitor");
}


void
AgentMonitorThread::finalize()
{
  webview_url_manager->unregister_baseurl(URL_PREFIX);
  webview_nav_manager->remove_nav_entry(URL_PREFIX);
  delete web_proc_;
}


void
AgentMonitorThread::loop()
{
}
