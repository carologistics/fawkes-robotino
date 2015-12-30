
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

//TODO::replace all this CRAP with a proper serializer 
std::string 
BridgeBlackBoardProcessor::publish_topic(std::string full_name,std::string id){


  if (interfaces_.find(full_name) == interfaces_.end()){
    if(!subscribe(full_name))
      return "";
  }

  interfaces_[full_name]->read();      
  
  StringBuffer s;
  Writer<StringBuffer> writer(s);      
  writer.StartObject();

  writer.String("op");
  writer.String("publish");

  if(!id.empty()){
    writer.String("id");
    writer.String(id.c_str(),(SizeType)id.length());
  }

  writer.String("topic");
  writer.String(full_name.c_str(), (SizeType)full_name.length());
  

  //TODO::handle JSON  types Object, null and Blackboard type float 
  
      //Fields
  writer.String("msg");
  //"msg" Json construction: 
  writer.StartObject();
  for (InterfaceFieldIterator fi  = interfaces_[full_name]->fields(); fi != interfaces_[full_name]->fields_end(); ++fi){ 


      std::string fieldName= fi.get_name();

      #ifdef RAPIDJSON_HAS_STDSTRING
        writer.String(fieldName);
      #else
        writer.String(fieldName.c_str(), (SizeType)fieldName.length());
      #endif

      std::string fieldType= fi.get_typename();


      if (fi.get_length() > 1 && fieldType != "string"){

       writer.StartArray();

       if(fieldType == "bool"){
        bool * arr = fi.get_bools();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Bool(arr[i]);
        }

        else if(fieldType == "double"){
        double * arr = fi.get_doubles();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Double(arr[i]);
        }

         else if(fieldType == "uint64"){
        uint64_t * arr = fi.get_uint64s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Uint64(arr[i]);
        }

        else if(fieldType == "uint32"){
        uint32_t * arr = fi.get_uint32s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Uint(arr[i]);
        }

        else if(fieldType == "uint16"){
        uint16_t * arr = fi.get_uint16s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Uint(arr[i]);
        }

         else if(fieldType == "uint8"){
        uint8_t * arr = fi.get_uint8s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Uint(arr[i]);
        }

        else if(fieldType == "uint64"){
        uint64_t * arr = fi.get_uint64s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Uint64(arr[i]);
        }

        else if(fieldType == "int32"){
        int32_t * arr = fi.get_int32s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Int(arr[i]);
        }

        else if(fieldType == "int16"){
        int16_t * arr = fi.get_int16s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Int(arr[i]);
        }

         else if(fieldType == "int8"){
        int8_t * arr = fi.get_int8s();
        for (unsigned i = 0; i < fi.get_length()-1; i++)
          writer.Int(arr[i]);
        }


      writer.EndArray();

      }
      else{
        if (fieldType== "bool")
          writer.Bool(fi.get_bool());

        else if (fieldType== "string")
          writer.String(fi.get_string());

        else if (fieldType== "double")
          writer.Double(fi.get_double());

        else if (fieldType== "uint32")
          writer.Uint(fi.get_uint32());

        else if (fieldType== "int32")
          writer.Int(fi.get_int32());

        else if (fieldType== "int8")
          writer.Int(fi.get_int8());

        else if (fieldType== "uint8")
          writer.Uint(fi.get_uint8());

        else if (fieldType== "int16")
          writer.Uint(fi.get_int16());

        else if (fieldType== "uint16")
          writer.Uint(fi.get_uint16());

        else if (fieldType== "int64")
          writer.Int64(fi.get_int64());

        else if (fieldType== "uint64")
          writer.Uint64(fi.get_uint64());

        else
          writer.String(fi.get_value_string());
      }


     //  else if (fieldType== "byte");
     //  else if (fieldType== "unknown")
     //  else if (fieldType== "_info->enumtype") find out where is this coming from

      //writer.Null();  find what null means in blackboard...if it exists


    }
    writer.EndObject();//the "msg" Json object
    writer.EndObject();//the full JSON object
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
      logger_->log_info("Type",fi.get_typename());
      logger_->log_info("Value",fi.get_value_string());
}


}
