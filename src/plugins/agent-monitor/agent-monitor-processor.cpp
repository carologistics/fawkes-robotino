
/***************************************************************************
 *  agent-monitor-processor.cpp - Monitoring the CLIPS agents in LLSF via webview
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

#include "agent-monitor-processor.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>
#include <webview/redirect_reply.h>
#include <utils/misc/string_conversions.h>
#include <plugins/clips/aspect/clips_env_manager.h>

#include <cstring>

#include <clipsmm.h>
#include <clips/clips.h>

using namespace fawkes;

/** @class AgentMonitorWebRequestProcessor "agent-monitor-processor.h"
 * Web request processor for monitoring the CLIPS agents in LLSF.
 * @author Frederik Zwilling
 */

/** Constructor.
 * @param clips_env_mgr CLIPS environment manager
 * @param logger logger to report problems
 * @param baseurl base URL of the Clips webrequest processor
 */
AgentMonitorWebRequestProcessor::AgentMonitorWebRequestProcessor(fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr,
						   fawkes::Logger *logger, const char *baseurl)
{
  clips_env_mgr_  = clips_env_mgr;
  logger_         = logger;
  logger->log_info("test", "logged");
  baseurl_        = baseurl;
  baseurl_len_    = strlen(baseurl);

}


/** Destructor. */
AgentMonitorWebRequestProcessor::~AgentMonitorWebRequestProcessor()
{
}

WebReply *
AgentMonitorWebRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string env_name = request->url().substr(baseurl_len_);
    std::string::size_type slash_pos = env_name.find("/", 1);
    std::string subpath;
    if (slash_pos != std::string::npos) {
      subpath  = env_name.substr(slash_pos);
      env_name = env_name.substr(1, slash_pos-1);
    } else if (env_name.length() > 0) {
      // remove lead slash
      env_name = env_name.substr(1);
    }

    std::map<std::string, LockPtr<CLIPS::Environment>> envs =
      clips_env_mgr_->environments();

    if (envs.find(env_name) == envs.end()) {
      if (envs.size() == 1) {
	// if there is only one just redirect
	return new WebRedirectReply(std::string(baseurl_) + "/" + envs.begin()->first);
      } else {
	WebPageReply *r = new WebPageReply("CLIPS - Environment not found");
	*r += "<h2>Environment " + env_name + " not found</h2>\n";
	if (! envs.empty()) {
	  *r += "<p>Choose on of the following existing environments:</p>\n";
	  *r += "<ul>\n";
	  for (auto env : envs) {
	    *r += std::string("<li><a href=\"") + baseurl_ + "/" +
	      env.first + "\">" + env.first + "</a></li>\n";
	  }
	  *r += "</ul>\n";
	} else {
	  *r += "<p>No environments have been registered.</p>\n";
	}
	return r;
      }
    }

    LockPtr<CLIPS::Environment> &clips = envs[env_name];

    MutexLocker lock(clips.objmutex_ptr());

    WebPageReply *r = new WebPageReply("Agent-Monitor");
    r->set_html_header("  <link type=\"text/css\" href=\"/static/css/jqtheme/"
		       "jquery-ui.custom.css\" rel=\"stylesheet\" />\n"
		       "  <script type=\"text/javascript\" src=\"/static/js/"
		       "jquery.min.js\"></script>\n"
		       "  <script type=\"text/javascript\" src=\"/static/js/"
		       "jquery-ui.custom.min.js\"></script>\n");

    *r +=
      "<style type=\"text/css\">\n"
      "  tr:hover { background-color: #eeeeee; }\n"
      "  :link:hover, :visited:hover { background-color: #bb0000; color: white; }\n"
      "  .envs { margin: 0px; padding: 0px; display: inline; }\n"
      "  .envs li { display: inline; padding-left: 8px; white-space: no-wrap; }\n"
      "</style>";

    if (envs.size() > 1) {
      *r += "Environments: <ul class=\"envs\">\n";
      for (auto env : envs) {
	*r += std::string("<li><a href=\"") + baseurl_ + "/" +
	  env.first + "\">" + env.first + "</a></li>\n";
      }
      *r += "</ul>\n";
    }

    *r += "<h2>State:</h2>\n";
    CLIPS::Fact::pointer state = get_fact(env_name, "state");
    *r += state->slot_value("")[0].as_string();
    
    *r += "<h2>Worldmodel:</h2>\n";
    *r += "<h3>Machines:</h3>\n";
    std::set<CLIPS::Fact::pointer> machines = get_all_facts(env_name, "machine");
    for(std::set<CLIPS::Fact::pointer>::iterator it = machines.begin(); it != machines.end(); it++)
    {
      *r += (*it)->slot_value("name")[0].as_string() + " ";
    }

    
    return r;
  } else {
    return NULL;
  }
}

CLIPS::Fact::pointer AgentMonitorWebRequestProcessor::get_first_fact(std::string env_name)
{
  std::map<std::string, LockPtr<CLIPS::Environment>> envs =
    clips_env_mgr_->environments();
  LockPtr<CLIPS::Environment> &clips = envs[env_name];
  MutexLocker lock(clips.objmutex_ptr());
  return clips->get_facts();
}

CLIPS::Fact::pointer
AgentMonitorWebRequestProcessor::get_next_fact(CLIPS::Fact::pointer start, std::string tmpl_name, std::map<std::string,std::string> constraints)
{
  CLIPS::Fact::pointer fact = start;
  while (fact)
  {
    CLIPS::Template::pointer tmpl = fact->get_template();
    if(tmpl->name().compare(tmpl_name) == 0)
    {
      if(constraints.size() == 0)
      {
	return fact;
      }
      bool fits_to_constraints = true;
      for(std::map<std::string,std::string>::iterator it = constraints.begin(); it != constraints.end() && fits_to_constraints; it++)
      {
	if(!fact->get_template()->slot_exists(it->first))
	{
	  logger_->log_error("agent-webview", "template %s has no slot %s", tmpl_name.c_str(), it->first.c_str());
	}
	else if(fact->slot_value(it->first)[0].as_string().compare(it->second) != 0)
	{
	  fits_to_constraints = false;
	}
      }
      if(fits_to_constraints)
      {
	return fact;
      }
    }
    
    fact = fact->next();
  }

  logger_->log_info("Agent-Monitor", "couldn't find fact with template name %s and constraints", tmpl_name.c_str());

  return NULL;
}

CLIPS::Fact::pointer
AgentMonitorWebRequestProcessor::get_fact(std::string env_name, std::string tmpl_name, std::map<std::string,std::string> constraints)
{
  CLIPS::Fact::pointer first = get_first_fact(env_name);
  return get_next_fact(first, tmpl_name, constraints);
}

std::set<CLIPS::Fact::pointer>
AgentMonitorWebRequestProcessor::get_all_facts(std::string env_name, std::string tmpl_name, std::map<std::string,std::string> constraints)
{
  std::set<CLIPS::Fact::pointer> res;
  CLIPS::Fact::pointer fact = get_first_fact(env_name);
  while(fact = get_next_fact(fact, tmpl_name, constraints))
  {
    res.insert(fact);
    fact = fact->next();
  }
  return res;
}
