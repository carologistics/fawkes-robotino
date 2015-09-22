
/***************************************************************************
 *  BridgeProcessor.cpp - Providing the backende calles to be acceable by python bridge
 *
 *  Copyright  2015 Mostafa Gomaa
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

#include "bridge_processor.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
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


BridgeProcessor::BridgeProcessor(fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr, fawkes::Logger *logger,  fawkes::Configuration *config, fawkes::BlackBoard *blackboard)
{
  clips_env_mgr_  = clips_env_mgr;
  logger_         = logger;
  config_         = config;
  blackboard_     = blackboard;
 
  //read config values:
  robotino1_ = config_->get_string("/agent-monitor/robotino1");
  robotino2_ = config_->get_string("/agent-monitor/robotino2");
  robotino3_ = config_->get_string("/agent-monitor/robotino3");
  env_name_ = config_->get_string("/agent-monitor/clips-environment");
  own_name_ = config_->get_string("/clips-agent/llsf2014/robot-name");
  refresh_interval_ = std::to_string(config_->get_int("/agent-monitor/refresh-interval"));

  //open interfaces
  light_if_ = blackboard_->open_for_reading<RobotinoLightInterface>("/machine-signal/best");
  skiller_if_ = blackboard_->open_for_reading<SkillerInterface>("Skiller");
   logger_->log_info("THE BRIDGEEEEEE PROCESSOR------", "Intialzed");
}


/** Destructor. */
BridgeProcessor::~BridgeProcessor()
{
  blackboard_->close(light_if_);
  blackboard_->close(skiller_if_);
}


std::map<std::string, std::string> 
BridgeProcessor::getAgentState(){

  std::string r="";
  std::map<std::string, std::string> agentState;

 //Have to change..This is just a hack
  std::string env_name="agent";
  std::map<std::string, LockPtr<CLIPS::Environment>> envs =
      clips_env_mgr_->environments();


  LockPtr<CLIPS::Environment> &clips = envs[env_name];

  MutexLocker lock(clips.objmutex_ptr());


  agentState["name"] = own_name_.c_str();

  CLIPS::Fact::pointer state = get_fact(env_name, "state");
  agentState["state"] =  get_slot(state, "");

  CLIPS::Fact::pointer task = get_fact(env_name, "task");
  agentState["task"] = get_slot(task, "name");

  CLIPS::Fact::pointer puck = get_fact(env_name, "holding");
  agentState["holding"] = get_slot(puck, "");

  agentState["locks-role"] = get_slot(get_fact(env_name, "lock-role"), "");
      
  std::set<CLIPS::Fact::pointer> holding_locks = get_all_facts(env_name, "lock", {{"type", "ACCEPT"}, {"agent", own_name_}});
  for(std::set<CLIPS::Fact::pointer>::iterator it = holding_locks.begin(); holding_locks.size() > 0 && it != holding_locks.end(); it++)
  {
  r += get_slot(*it, "resource");
  }
  agentState["locks-owns"] = r;


  std::set<CLIPS::Fact::pointer> requested_locks = get_all_facts(env_name, "lock", {{"type", "GET"}, {"agent", own_name_}});
  r="";
  for(std::set<CLIPS::Fact::pointer>::iterator it = requested_locks.begin(); requested_locks.size() > 0 && it != requested_locks.end(); it++)
  {
  r+=  get_slot(*it, "resource");
  }
  agentState["Waiting for"] = r; 



  logger_->log_info("THE BRIDGEEEEEE------:", "State %s" , agentState["state"].c_str());
  logger_->log_info("THE BRIDGEEEEEE------:", "Task %s" , agentState["task"].c_str());
  logger_->log_info("THE BRIDGEEEEEE------:", "holding %s" , agentState["holding"].c_str());
  logger_->log_info("THE BRIDGEEEEEE------:", "Locks-role %s" , agentState["locks-role"].c_str());
  logger_->log_info("THE BRIDGEEEEEE------:", "lockes-owns %s" , agentState["lockes-owns"].c_str());

  return agentState;

}


int
BridgeProcessor::TestInt(){
  return 5;
}




CLIPS::Fact::pointer BridgeProcessor::get_first_fact(std::string env_name)
{
  std::map<std::string, LockPtr<CLIPS::Environment>> envs =
    clips_env_mgr_->environments();
  LockPtr<CLIPS::Environment> &clips = envs[env_name];
  MutexLocker lock(clips.objmutex_ptr());
  return clips->get_facts();
}

CLIPS::Fact::pointer
BridgeProcessor::get_next_fact(CLIPS::Fact::pointer start, std::string tmpl_name, std::map<std::string,std::string> constraints)
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
BridgeProcessor::get_fact(std::string env_name, std::string tmpl_name, std::map<std::string,std::string> constraints)
{
  CLIPS::Fact::pointer first = get_first_fact(env_name);
  return get_next_fact(first, tmpl_name, constraints);
}

std::set<CLIPS::Fact::pointer>
BridgeProcessor::get_all_facts(std::string env_name, std::string tmpl_name, std::map<std::string,std::string> constraints)
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



const char*
BridgeProcessor::get_slot(CLIPS::Fact::pointer fact, std::string slot, int entry)
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
BridgeProcessor::get_slot_string(CLIPS::Fact::pointer fact, std::string slot, int entry)
{
  std::string res = get_slot(fact, slot, entry);
  return res;
}



