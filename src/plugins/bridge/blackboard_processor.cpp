
/***************************************************************************
 *  BridgeBlackBoardProcessor.cpp - Providing the backende calles to be acceable by python bridge
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

#include "blackboard_processor.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <utils/misc/string_conversions.h>
#include <utils/time/time.h>
#include <utils/misc/string_split.h>

#include <interface/interface.h>
#include <interface/field_iterator.h>
#include <interface/interface_info.h>

#include <logging/logger.h>

#include <blackboard/blackboard.h>


#include <string>
#include <cstring>
#include <cstdlib>

#include <iostream>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"


using namespace rapidjson;
using namespace std;

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

BridgeBlackBoardProcessor::BridgeBlackBoardProcessor(fawkes::Logger *logger,  fawkes::Configuration *config, fawkes::BlackBoard *blackboard)
{
  logger_         = logger;
  config_         = config;
  blackboard_     = blackboard;
 
  logger_->log_info("BlackBoard processor::", "Intialzed");
}


/** Destructor. */
BridgeBlackBoardProcessor::~BridgeBlackBoardProcessor()
{
  for (ifi_ = interfaces_.begin(); ifi_ != interfaces_.end(); ++ifi_) {
    blackboard_->close(ifi_->second);
  }
  interfaces_.clear();
}

bool
BridgeBlackBoardProcessor::subscribe(std::string full_name){

  //TODO::take care of the differnt ways the topic is spelled

   std::size_t pos =full_name.find("::");
   std::string if_type=full_name;
   std::string if_id=full_name;

    if (pos != std::string::npos)
    {
      if_type.erase(pos,full_name.length());
      if_id.erase(0 ,pos+2);
      logger_->log_info("BridgeProcessor::if_type",if_type.c_str());
      logger_->log_info("BridgeProcessor::if_id",if_id.c_str());
    
    }

  bool found=false;

  InterfaceInfoList *iil=blackboard_->list_all();
  for (InterfaceInfoList::iterator i = iil->begin(); i != iil->end(); ++i){
    if(if_type.compare(i->type()) == 0 && if_id.compare(i->id())){
      found= true;
      break;
    }
  }
  delete iil;

  if(! found)
  {
    logger_->log_info("FawkesBridge::","Interface Does not Exist");
    return false;
  }

  if (interfaces_.find(full_name) == interfaces_.end()) {
    try {
      Interface *iface = blackboard_->open_for_reading(if_type.c_str(), if_id.c_str());
      interfaces_[full_name] = iface;
    } catch (Exception &e) {
      logger_->log_info("FawkesBridge::","Failed to open interface: %s\n", e.what());
      return false;
    }
    logger_->log_info("FawkesBridge::", "Interface %s  Succefully Opened!",full_name.c_str());
  }
  else {
    logger_->log_info("FawkesBridge::", "Interface %s was already opened",full_name.c_str());
  }

  return found;
}

std::string 
BridgeBlackBoardProcessor::read_single_topic(std::string full_name){

  StringBuffer s;
      Writer<StringBuffer> writer(s);
      
      writer.StartObject();
      writer.String("hello");
      writer.String("world");
      writer.String("t");
      writer.Bool(true);
      writer.String("f");
      writer.Bool(false);
      writer.String("n");
      writer.Null();
      writer.String("i");
      writer.Uint(123);
      writer.String("pi");
      writer.Double(3.1416);
      writer.String("a");
      writer.StartArray();
      for (unsigned i = 0; i < 4; i++)
          writer.Uint(i);
      writer.EndArray();
      writer.EndObject();

      std::cout << s.GetString() << std::endl;

      publish();
      return s.GetString();


}


void 
BridgeBlackBoardProcessor::publish(){

  for ( ifi_ = interfaces_.begin(); ifi_ != interfaces_.end(); ++ifi_){
    postInterface(ifi_->second);
  }

}

void 
BridgeBlackBoardProcessor::postInterface(fawkes::Interface* iface){
  iface->read();

   std::string writer;
    if (iface->has_writer()) {
      try {
        writer = iface->writer();
      } catch (Exception &e) {}
    }
    std::string readers;
    try {
      readers = str_join(iface->readers(), ", ");
    } catch (Exception &e) {}

  
      logger_->log_info("Type:",iface->type());
      logger_->log_info("ID:",iface->id());
      logger_->log_info("Writer: blackboard-writer- ",iface->has_writer() ?  writer.c_str() : "none");
      logger_->log_info("Readers:",iface->num_readers() > 0 ? readers.c_str() : "none",iface->num_readers());
      //logger_->log_info("Serial:%u \n",iface->serial());
      // logger_->log_info("Data size:%u \n",iface->datasize());
      // logger_->log_info("Hash:%s \n",iface->hash_printable());
      // logger_->log_info("Data changed: \n %s (last at %s) \n",iface->changed() ? "yes" : "no", iface->timestamp()->str());
      

      //Fields

for (InterfaceFieldIterator fi  = iface->fields(); fi != iface->fields_end(); ++fi){
      logger_->log_info("Name",fi.get_name());
      logger_->log_info("Type",fi.get_name());
      logger_->log_info("Value",fi.get_value_string());
}


}
