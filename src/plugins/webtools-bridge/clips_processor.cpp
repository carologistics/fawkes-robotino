
/***************************************************************************
 * ClipsProcessor.cpp - process Webtools-Bridge requests intended for Clips Facts
 *
 *  Created: Mon April 11 2016
 *  Copyright  2016 Mostafa Gomaa 
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
#include "clips_processor.h"
#include "serializer.h"

#include "rapidjson/writer.h"//TODO:: be moved when serializer is complete
#include "rapidjson/stringbuffer.h"

#include <map>

#include <logging/logger.h>
#include <core/threading/mutex_locker.h>
 #include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#include <plugins/clips/aspect/clips_env_manager.h>

#include <clipsmm.h>
#include <clips/clips.h>//am not sure i even need this ..lets see


using namespace fawkes ;
using namespace rapidjson ;


ClipsSubscription::ClipsSubscription(std::string topic_name 
                          , std::string processor_prefix 
                          , fawkes::Clock *clock
                          , fawkes::LockPtr<CLIPS::Environment> &clips)
: Subscription(topic_name,processor_prefix, clock)
, clips_(clips)
{

}

ClipsSubscription::~ClipsSubscription()
{

}

void
ClipsSubscription::activate_impl()
{

}

void
ClipsSubscription::deactivate_impl()
{

}

void
ClipsSubscription::finalize_impl()
{

}

std::string
ClipsSubscription::serialize(std::string op
                            , std::string topic_name
                            , std::string id)
{

  std::string prefixed_topic_name= processor_prefix_+"/"+topic_name;
  std::string tmpl_name = topic_name;

  MutexLocker lock(clips_.objmutex_ptr()); 
  CLIPS::Fact::pointer fact= clips_->get_facts(); //intialize it with the first fact
  bool fact_found = false;
  while (fact)
  {
    CLIPS::Template::pointer tmpl = fact->get_template();
    if(tmpl->name().compare(tmpl_name) == 0)
    {
      fact_found= true;
      break;
    }
    fact = fact->next();
  }

  if(fact_found)
  {
    //Default 'publish' header
    StringBuffer s;
    Writer<StringBuffer> writer(s);      
    writer.StartObject();

    writer.String("op");
    writer.String(op.c_str(),(SizeType)op.length());

    writer.String("id");
    writer.String(id.c_str(),(SizeType)id.length());
    
    writer.String("topic");
    writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());
    
    writer.String("msg");
    
    Serializer::serialize(fact);

    writer.EndObject();//End of complete Json_msg 
    
    return s.GetString();
    
  }
  return "";
}



//=================================   Processor  ===================================

ClipsProcessor::ClipsProcessor(std::string prefix 
                              , fawkes::Logger *logger
                              , fawkes::Configuration *config
                              , fawkes::Clock *clock
                              , fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr)
:BridgeProcessor(prefix)
, env_name_("agent") //TODO: get from configs
{
  clips_env_mgr_  = clips_env_mgr;
  logger_         = logger;
  config_         = config;
  clock_          = clock;
  
  logger_->log_info("ClipsProcessor::", "Initialized!");

  //TODO set the env_name
}

ClipsProcessor::~ClipsProcessor()
{

}

void
ClipsProcessor::init()
{
  std::map<std::string, LockPtr<CLIPS::Environment>> envs =
  clips_env_mgr_->environments();

  if (envs.find(env_name_) == envs.end()) 
  {
    if (envs.size() == 1)
    { // if there is only one just select it
      env_name_ = envs.begin()->first;
    }
    else
    {
      throw fawkes::UnknownTypeException("ClipsProcessor: Environment '%s' was not found!", env_name_.c_str());
      //say what was wrong no envs Vs Wrong name
    }
  }

  clips_ = envs[env_name_];
}

std::shared_ptr<Subscription>
ClipsProcessor::subscribe   ( std::string prefixed_topic_name 
                            , std::string id    
                            , std::string compression
                            , unsigned int throttle_rate  
                            , unsigned int queue_length   
                            , unsigned int fragment_size  
                            , std::shared_ptr<WebSession> session)
{
  std::string tmpl_name = ""; //get it from the prefixed topic name
  std::size_t pos = prefixed_topic_name.find(prefix_,0);
  if (pos != std::string::npos && pos <= 1)
  {
    tmpl_name= prefixed_topic_name.substr(pos+prefix_.length()+1);//+1 accounts for the leading '/' before the topic name
  }

  MutexLocker lock(clips_.objmutex_ptr()); 
  CLIPS::Fact::pointer fact= clips_->get_facts(); //intialize it with the first fact
  bool fact_found = false;
  while (fact)
  {
    CLIPS::Template::pointer tmpl = fact->get_template();
    if(tmpl->name().compare(tmpl_name) == 0)
    {
      fact_found= true;
      break;
      //for now, its just enough to know that there is a fact with this topic_name to make a subscription
      //Realisticly, u need to know the constraint and fit to it
      //more over if nothing fits now or no fact with tmp_name found..Does not mean that this subscription is not valid. 
      //Could mean that the fact has not been asserted yet and that we need to watch for it
      //So basically i think there is no invalide subscriptions with clips..Just ones that u need to watch for.

    }
    fact = fact->next();
  }

  if(! fact_found)
  logger_->log_info("ClipsProcessor", "couldn't find fact with template name %s for now", tmpl_name.c_str());

  //for now, always make a subscription intstance for the subscription request (even if no fact found)
  std::shared_ptr <ClipsSubscription> new_subscirption;
   
  //a DORMANT subscription instance with it
  try{

    new_subscirption = std::make_shared <ClipsSubscription>(tmpl_name
                                                            , prefix_ 
                                                            , clock_ 
                                                            , clips_);
                                                               
  }catch (fawkes::Exception &e) {
    logger_->log_info("ClipsProcessor:" , "Failed to subscribe to '%s': %s\n", tmpl_name.c_str(), e.what());
    throw e;
  }
  
  return new_subscirption;
}

void
ClipsProcessor::unsubscribe ( std::string id
                    , std::shared_ptr<Subscription> subscription
                    , std::shared_ptr<WebSession> session ) 
{

}
