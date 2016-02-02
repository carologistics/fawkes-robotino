
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

#include <core/exceptions/software.h>
#include <core/threading/mutex_locker.h>
#include <utils/misc/string_conversions.h>
#include <utils/time/time.h>
#include <utils/misc/string_split.h>

#include <interface/interface.h>
#include <interface/field_iterator.h>
#include <interface/interface_info.h>

#include <logging/logger.h>

#include <blackboard/blackboard.h>

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <string>
#include <cstring>
#include <cstdlib>

using namespace std;

using namespace fawkes;

using namespace rapidjson;

//=================================   Processor  ===================================

BridgeBlackBoardProcessor::BridgeBlackBoardProcessor(std::string prefix
                                                    , fawkes::Logger *logger
                                                    , fawkes::Configuration *config
                                                    , fawkes::BlackBoard *blackboard
                                                    , fawkes::Clock *clock)
:BridgeProcessor(prefix)
{
  logger_         = logger;
  config_         = config;
  blackboard_     = blackboard;
  clock_          = clock;
  logger_->log_info("BlackBoardProcessor::", "Initialized!");
}


/** Destructor. */
BridgeBlackBoardProcessor::~BridgeBlackBoardProcessor()
{
  // for (ifi_ = interface_.begin(); ifi_ != interface_.end(); ++ifi_) {
  //   blackboard_->close(ifi_->second);
  // }
  // interface_.clear();
}

std::shared_ptr <Subscription> 
BridgeBlackBoardProcessor::subscribe( std::string prefixed_topic_name 
                                      , std::string id    
                                      , std::string compression
                                      , unsigned int throttle_rate  
                                      , unsigned int queue_length   
                                      , unsigned int fragment_size  
                                      , std::shared_ptr<WebSession> session)
{

  //Extract prefix from the name
  std::string topic_name="";
  std::size_t pos = prefixed_topic_name.find(prefix_,0);
  if (pos != std::string::npos && pos <= 1)
  {
    topic_name= prefixed_topic_name.erase(0 , prefix_.length()+pos+1);//+1 accounts for the leading '/' before the topic name
  }

  if(topic_name == ""){
    //this should not happen
    throw fawkes::SyntaxErrorException("BlackBoardProcessor: Topic Name '%s' does not contian prefix of %s processor ", prefixed_topic_name.c_str() , prefix_.c_str());
  }

  //TODO::take care of the differnt ways the topic is spelled

  //Extracet the BlackBoard Interface Type and Id
  pos =topic_name.find("::",0);
  std::string if_id=topic_name;
  std::string if_type=topic_name;
  
  if (pos != std::string::npos)
  {
    if_type.erase(pos,topic_name.length());
    if_id.erase(0 ,pos+2);
    //logger_->log_info("BlackboardProcessor: if_type",if_type.c_str());
    //logger_->log_info("BlackboardProcessor: if_id",if_id.c_str());
  }
  else{
    throw fawkes::SyntaxErrorException("BlackBoardProcessor: Wrong 'Topic Name' format!");
  }

  //Look for the topic in the BlackBoard intefaces
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
    throw fawkes::UnknownTypeException("BlackboardProcessor: Interface '%s' was not found!", topic_name.c_str());
  }
 
  
  std::shared_ptr <BlackBoardSubscription> new_subscirption;

  //Open the interface and create a dromant subscription instance with it
  try{

    new_subscirption = std::make_shared <BlackBoardSubscription>(topic_name 
                                                               , prefix_ 
                                                               , clock_ 
                                                               , blackboard_ 
                                                               , blackboard_->open_for_reading(if_type.c_str(), if_id.c_str()) );
                                                               
  }catch (fawkes::Exception &e) {
    logger_->log_info("BlackBoardProcessor:" , "Failed to open subsribe to '%s': %s\n", topic_name.c_str(), e.what());
    throw e;
  }

  logger_->log_info("BlackboardProcessor:", "Interface '%s' Succefully Opened!", topic_name.c_str());

  new_subscirption->add_Subscription_request(id , compression , throttle_rate  
                                           , queue_length , fragment_size , session);
  
  return new_subscirption ;
}



void
BridgeBlackBoardProcessor::unsubscribe( std::string id
                                      , std::shared_ptr <Subscription>  subscription
                                      , std::shared_ptr <WebSession> session)
{
  std::shared_ptr <BlackBoardSubscription>  bb_subscription
                  = std::static_pointer_cast<BlackBoardSubscription> (subscription);

  bb_subscription->remove_Subscription_request(id, session);
  
}



//====================================  Subsciption ===========================================


BlackBoardSubscription::BlackBoardSubscription(std::string topic_name 
                                              , std::string processor_prefix 
                                              , fawkes::Clock *clock
                                              , fawkes::BlackBoard *blackboard
                                              , fawkes::Interface *interface)
: Subscription(topic_name,processor_prefix, clock)
, BlackBoardInterfaceListener("WebToolsBridgeListener")
, blackboard_(blackboard)
, interface_(interface)
{
  
}

BlackBoardSubscription::~BlackBoardSubscription()
{
  BlackBoardSubscription::finalize();
  delete interface_;
}

fawkes::Interface*
BlackBoardSubscription::get_interface_ptr()
{
  return  interface_;
}


void
BlackBoardSubscription::activate_impl()
{
   if(active_status_ != ACTIVE)
   {
  //   //Extracet the BlackBoard Interface Type and Id
  //   std::size_t pos =topic_name.find("::");
  //   if (pos != std::string::npos)
  //   {
  //     std::string if_type=topic_name;
  //     if_type.erase(pos,Subscription.length());

  //     std::string if_id=topic_name;
  //     if_id.erase(0 ,pos+2);
  //   }

  //   //Open the interface
  //   try{
  //     interface_ = blackboard_->open_for_reading(if_type.c_str(), if_id.c_str());
  //   }catch (Exception &e) {
  //     logger_->log_info("FawkesBridge::","Failed to open interface: %s\n", e.what());
  //     //Throw and may be not catch it and escelate it to the calling CPM.
  //     return;
  //   }

  //Checf interface is valid is and thorw exception if not

    bbil_add_data_interface(interface_);
  }
}

void
BlackBoardSubscription::deactivate_impl()
{
  if(active_status_ != DORMANT){
    bbil_remove_data_interface(interface_);
  }
}

void
BlackBoardSubscription::finalize_impl()
{
  deactivate();
  blackboard_->close(interface_);
}

std::string
BlackBoardSubscription::serialize_impl()
{
  //Create the Json that will hold the data
  StringBuffer s;
  Writer<StringBuffer> writer(s);      
  writer.StartObject();

  //Fill in the data
  for (InterfaceFieldIterator fi  = interface_->fields(); fi != interface_->fields_end(); ++fi)
  { 

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
  
  //Close the json
  writer.EndObject();

  return s.GetString();
}

void 
BlackBoardSubscription::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
  if(active_status_ == DORMANT){
    //this should not happen 
    return;
  }

  interface_->read();

  publish();
}