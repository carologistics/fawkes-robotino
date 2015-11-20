
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

BridgeBlackBoardProcessor::init(){}


BridgeBlackboardProcessor::process_request(std::string msg){

}


bool
BridgeBlackBoardProcessor::subscribe(std::string if_type, std::string if_id){

  std::string full_name = if_type+"::"+if_id;
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

  return true;
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
