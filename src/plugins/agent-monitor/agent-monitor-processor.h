
/***************************************************************************
 *  agent-monitor-processor.h - Monitoring the CLIPS agents in LLSF via webview
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

#ifndef __PLUGINS_AGENT_MONITOR_AGENT_MONITOR_PROCESSOR_H_
#define __PLUGINS_AGENT_MONITOR_AGENT_MONITOR_PROCESSOR_H_

#include <core/utils/lockptr.h>
#include <webview/request_processor.h>
#include <logging/logger.h>

#include <string>
#include <list>
#include <map>
#include <set>

#include <clipsmm.h>

namespace fawkes {
  class CLIPSEnvManager;
}

namespace CLIPS {
  class Environment;
}

class AgentMonitorWebRequestProcessor : public fawkes::WebRequestProcessor
{
 public:
  AgentMonitorWebRequestProcessor(fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips,
			   fawkes::Logger *logger,
			   const char *baseurl);

  virtual ~AgentMonitorWebRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

  void add_error(const char *str);

 private:
  fawkes::LockPtr<fawkes::CLIPSEnvManager> clips_env_mgr_;
  fawkes::Logger       *logger_;

  const char           *baseurl_;
  size_t                baseurl_len_;

  std::list<std::string> errors_;
  
  CLIPS::Fact::pointer get_first_fact(std::string env_name);
  CLIPS::Fact::pointer get_fact(std::string env_name, std::string tmpl_name,
				std::map <std::string, std::string> constraints = {});
  CLIPS::Fact::pointer get_next_fact(CLIPS::Fact::pointer start, std::string tmpl_name,
				std::map <std::string, std::string> constraints = {});
  std::set<CLIPS::Fact::pointer> get_all_facts(std::string env_name, std::string tmpl_name,
				std::map <std::string, std::string> constraints = {});
};

#endif
