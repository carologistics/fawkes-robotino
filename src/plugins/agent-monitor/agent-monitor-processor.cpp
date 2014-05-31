
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
AgentMonitorWebRequestProcessor::AgentMonitorWebRequestProcessor(fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr, fawkes::Logger *logger, const char *baseurl, fawkes::Configuration *config, fawkes::BlackBoard *blackboard)
{
  clips_env_mgr_  = clips_env_mgr;
  logger_         = logger;
  config_         = config;
  blackboard_     = blackboard;
  baseurl_        = baseurl;
  baseurl_len_    = strlen(baseurl);

  //read config values:
  robotino1_ = config_->get_string("/agent-monitor/robotino1");
  robotino2_ = config_->get_string("/agent-monitor/robotino2");
  robotino3_ = config_->get_string("/agent-monitor/robotino3");
  env_name_ = config_->get_string("/agent-monitor/clips-environment");
  own_name_ = config_->get_string("/clips-agent/llsf2014/robot-name");

  //open interfaces
  light_if_ = blackboard_->open_for_reading<RobotinoLightInterface>
    (config->get_string("/plugins/light_front/light_state_if").c_str());
}


/** Destructor. */
AgentMonitorWebRequestProcessor::~AgentMonitorWebRequestProcessor()
{
  blackboard_->close(light_if_);
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
                       "<meta http-equiv=\"refresh\" content=\"2\" />\n");

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

    r->set_navbar_enabled(false);
    r->set_footer_enabled(false);

    
    if(subpath == "/state")
    {
      *r += "<table>\n";
      *r += "<tr><td>";
      //state and current task
      r->append_body("<h2>%s:</h2>\n", own_name_.c_str());
      CLIPS::Fact::pointer state = get_fact(env_name, "state");
      r->append_body("<b>State:</b> %s<br>\n", get_slot(state, ""));
      CLIPS::Fact::pointer task = get_fact(env_name, "task");
      *r += "<b>Task: </b>";
      if(std::strcmp(get_slot(task, "name"), "load-with-S0") == 0)
	r->append_body("load %s with S0", get_slot(task, "args", 0));
      else if(std::strcmp(get_slot(task, "name"), "load-with-S1") == 0)
	r->append_body("load %s with S1", get_slot(task, "args", 0));
      else if(std::strcmp(get_slot(task, "name"), "pick-and-load") == 0)
	r->append_body("bring produced puck from %s to %s", get_slot(task, "args", 0), get_slot(task, "args", 1));
      else if(std::strcmp(get_slot(task, "name"), "pick-and-deliver") == 0)
	r->append_body("deliver puck from %s", get_slot(task, "args", 0));
      else if(std::strcmp(get_slot(task, "name"), "recycle") == 0)
	r->append_body("recycle puck from %s", get_slot(task, "args", 0));
      else
	*r += "unknown task";
      *r += "<br>\n";
      CLIPS::Fact::pointer puck = get_fact(env_name, "holding");
      r->append_body("<b>Holding:</b> %s<br>\n", get_slot(puck, ""));

      //locks
      *r += "<h3>Locks:</h3>\n";
      r->append_body("<b>Role: </b> %s<br>\n", get_slot(get_fact(env_name, "lock-role"), ""));
      *r += "<b>Owns:</b> ";
      std::set<CLIPS::Fact::pointer> holding_locks = get_all_facts(env_name, "lock", {{"type", "ACCEPT"}, {"agent", own_name_}});
      for(std::set<CLIPS::Fact::pointer>::iterator it = holding_locks.begin(); holding_locks.size() > 0 && it != holding_locks.end(); it++)
      {
	r->append_body("%s ", get_slot(*it, "resource"));
      }

      *r += "<br>\n<b>Waiting for:</b> <font color=\"#FF0000\">";
      std::set<CLIPS::Fact::pointer> requested_locks = get_all_facts(env_name, "lock", {{"type", "GET"}, {"agent", own_name_}});
      for(std::set<CLIPS::Fact::pointer>::iterator it = requested_locks.begin(); requested_locks.size() > 0 && it != requested_locks.end(); it++)
      {
	r->append_body("%s ", get_slot(*it, "resource"));
      }
      *r += "</font><br>\n";
      *r += "</td>\n<td>";
      
      //Current Vision:
      *r += "<h3>Vision:</h3>\n";
      light_if_->read();

      if(std::strcmp(light_if_->tostring_LightState(light_if_->red()), "ON") == 0)
      {
	r->append_body("<img src=\"/static/red-on.png\" /><br>");
      }
      else if(std::strcmp(light_if_->tostring_LightState(light_if_->red()), "OFF") == 0)
	r->append_body("<img src=\"/static/red-off.png\" /><br>");
      else
	r->append_body("<img src=\"/static/red-blink.png\" /><br>");

      if(std::strcmp(light_if_->tostring_LightState(light_if_->yellow()), "ON") == 0)
      {
	r->append_body("<img src=\"/static/yellow-on.png\" /><br>");
      }
      else if(std::strcmp(light_if_->tostring_LightState(light_if_->yellow()), "OFF") == 0)
	r->append_body("<img src=\"/static/yellow-off.png\" /><br>");
      else
	r->append_body("<img src=\"/static/yellow-blink.png\" /><br>");

      if(std::strcmp(light_if_->tostring_LightState(light_if_->green()), "ON") == 0)
      {
	r->append_body("<img src=\"/static/green-on.png\" /><br>");
      }
      else if(std::strcmp(light_if_->tostring_LightState(light_if_->green()), "OFF") == 0)
	r->append_body("<img src=\"/static/green-off.png\" /><br>");
      else
	r->append_body("<img src=\"/static/green-blink.png\" /><br>");
      
      r->append_body("History:  %d<br>\n", light_if_->visibility_history());
      *r += "</td></tr>";
      *r += "</table>\n";

      return r;
    }

    if(subpath == "/worldmodel")
    {
      *r += "<h2>Worldmodel:</h2>\n";
      //table for machines
      *r += "<table>\n";
      *r += "<tr> <th>Machine</th> <th>Type</th> <th>Loaded With</th> <th>Incoming</th> <th>Produced</th> <th>Junk</th>  </tr>\n";
      for(int i = 1; i <= 24; i++)
      {
	r->append_body(machine_table_row(env_name, i).c_str());
      }
      *r += "</table>\n";
      
      return r;
    }

    if(subpath == "/refbox")
    {
      *r += "<h2>Refbox:</h2>\n";
      r->append_body("<b>Time:</b> %s<br>\n", get_slot(get_fact(env_name, "game-time"), "", 0));
      *r += "<b>Points:</b> \n";
      r->append_body("<span style=\"background-color:cyan\">%s</span>\n", get_slot(get_fact(env_name, "points-cyan"), ""));
      r->append_body("<span style=\"background-color:magenta\">%s</span>\n", get_slot(get_fact(env_name, "points-magenta"), ""));
      //table for orders
      *r += "<table>\n";
      *r += "<tr> <th>Order</th> <th>Type</th> <th>Amount</th> <th>Start</th> <th>End</th>  </tr>\n";
      for(int i = 1; i <= 5; i++)
      {
	r->append_body(order_table_row(env_name, i).c_str());
      }
      *r += "</table>\n";
      
      return r;
    }

    if (subpath == "/saveButton") {
      // Button for saving facts
      //do not reload page
      r->set_html_header("  <link type=\"text/css\" href=\"/static/css/jqtheme/"
			 "jquery-ui.custom.css\" rel=\"stylesheet\" />\n"
			 "  <script type=\"text/javascript\" src=\"/static/js/"
			 "jquery.min.js\"></script>\n"
			 "  <script type=\"text/javascript\" src=\"/static/js/"
			 "jquery-ui.custom.min.js\"></script>\n");
      r->append_body("<p><form action=\"%s/%s/save\" method=\"post\">"
		     "<input type=\"hidden\" name=\"index\" value=\"\">"
		     "<input type=\"text\" name=\"comment\" />"
		     "<input type=\"submit\" value=\"save facts with comment\" /></form></p>",
		     baseurl_, env_name.c_str());
      return r;
    }

    if (subpath == "/save") {
      MutexLocker lock(clips.objmutex_ptr());
      std::string save_command = "(save-facts (str-cat \""
	+ request->post_value("comment") + "\" "
	+ get_slot(get_fact(env_name, "game-time"), "", 0)
	+ " \".clp\") visible)";
      clips->evaluate(save_command);
      logger_->log_info("save debug", get_slot(get_fact(env_name, "game-time"), "", 0));
      return new WebRedirectReply(std::string(baseurl_) + "/" + env_name + "/saveButton");
    }

    //no subpath => show composed overview    
    r->set_navbar_enabled(true);
    r->set_footer_enabled(true);
    r->set_html_header("  <link type=\"text/css\" href=\"/static/css/jqtheme/"
		       "jquery-ui.custom.css\" rel=\"stylesheet\" />\n"
		       "  <script type=\"text/javascript\" src=\"/static/js/"
		       "jquery.min.js\"></script>\n"
		       "  <script type=\"text/javascript\" src=\"/static/js/"
		       "jquery-ui.custom.min.js\"></script>\n");

    //*r += "<h2>Composed Overview:</h2>\n";

    //Worldmodel
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/worldmodel\" width=\"35%\" height=\"50%\" frameborder=\"0\"></iframe>\n", robotino1_.c_str());
    //refbox infos
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/refbox\" width=\"35%\" height=\"50%\" frameborder=\"0\"></iframe>\n", robotino1_.c_str());
    *r += "<br>\n";
    
    //Robot states
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/state\" width=\"33%\" height=\"35%\" frameborder=\"0\"></iframe>\n", robotino1_.c_str());
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/state\" width=\"33%\" height=\"35%\" frameborder=\"0\"></iframe>\n", robotino2_.c_str());
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/state\" width=\"33%\" height=\"35%\" frameborder=\"0\"></iframe>\n", robotino3_.c_str());
    *r += "<br>\n";

    //Save facts buttons
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/saveButton\" width=\"33%\" height=\"10%\" frameborder=\"0\"></iframe>\n", robotino1_.c_str());
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/saveButton\" width=\"33%\" height=\"10%\" frameborder=\"0\"></iframe>\n", robotino2_.c_str());
    r->append_body("<iframe src=\"http://%s/agent-monitor/agent/saveButton\" width=\"33%\" height=\"10%\" frameborder=\"0\"></iframe>\n", robotino3_.c_str());
    *r += "<br>\n";

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
	else if(it->second.compare(get_slot(fact, it->first)) != 0)
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

  //logger_->log_info("Agent-Monitor", "couldn't find fact with template name %s and constraints", tmpl_name.c_str());

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


std::string
AgentMonitorWebRequestProcessor::machine_table_row(std::string env_name, int n)
{
  std::string name = "M" + std::to_string(n);
  CLIPS::Fact::pointer m = get_fact(env_name, "machine", {{"name",name.c_str()}});
  std::string res = "";

  //ommit machines of the other team
  std::string team_color = get_slot(get_fact(env_name, "team-color"), "");
  if(team_color.compare(get_slot(m, "team")) != 0)
  {
    return res;
  }
  
  res += "<tr> <td>" + name + "</td> ";
  res += "<td>" + get_slot_string(m, "mtype") + "</td> ";

  CLIPS::Values loaded_with = m->slot_value("loaded-with");
  res += "<td>";
  for(CLIPS::Values::iterator it = loaded_with.begin(); loaded_with.size() > 0 && it != loaded_with.end(); it++)
  {
    res += it->as_string() + " ";
  }
  res += "</td> ";

  CLIPS::Values incoming = m->slot_value("incoming");
  res += "<td>";
  for(CLIPS::Values::iterator it2 = incoming.begin(); incoming.size() > 0 && it2 != incoming.end(); it2++)
  {
    res += it2->as_string() + " ";
  }
  res += "</td> ";

  res += "<td>" + get_slot_string(m, "produced-puck") + "</td> ";
  res += "<td>" + get_slot_string(m, "junk") + "</td> ";
  
  res += "</tr>\n";
  return res;
}

std::string
AgentMonitorWebRequestProcessor::order_table_row(std::string env_name, int n)
{
  std::string id = std::to_string(n);
  CLIPS::Fact::pointer order = get_fact(env_name, "order", {{"id",id.c_str()}});
  std::string res = "";

  //is the order already there?
  if(!order)
  {
    return "<tr> <td>" + id + "</td></tr>";
  }
  
  res += "<tr> <td>" + id + "</td> ";
  res += "<td>" + get_slot_string(order, "product") + "</td> ";
  res += "<td>" + get_slot_string(order, "quantity-requested") + "</td> ";
  res += "<td>" + get_slot_string(order, "begin") + "</td> ";
  res += "<td>" + get_slot_string(order, "end") + "</td> ";
  res += "</tr>\n";
  return res;
}

const char*
AgentMonitorWebRequestProcessor::get_slot(CLIPS::Fact::pointer fact, std::string slot, int entry)
{
  std::string res = "";
  //make sure the fact exists
  if(!fact)
  {
    res = "could not get slot (no fact?)";
    return res.c_str();
  }
  CLIPS::Values values = fact->slot_value(slot);
  //TODO: check if slot exists
  
  switch(values[entry].type())
  {
  case CLIPS::TYPE_INTEGER :
    return std::to_string(values[entry].as_integer()).c_str();
  case CLIPS::TYPE_FLOAT :
    return std::to_string(values[entry].as_float()).c_str();
  default :
    return values[entry].as_string().c_str();
  }
}

std::string
AgentMonitorWebRequestProcessor::get_slot_string(CLIPS::Fact::pointer fact, std::string slot, int entry)
{
  std::string res = get_slot(fact, slot, entry);
  return res;
}
