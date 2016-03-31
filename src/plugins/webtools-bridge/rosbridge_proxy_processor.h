
/***************************************************************************
 *  RosBridgeProxyProcessor.h - Monitoring the CLIPS agents in LLSF via webview
 *
 *  Created: Mon Mar 2016
 *  Copyright  2016  MosafaGomaa
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

#ifndef __PLUGINS_ROSPROXY_PROCESSOR_H_
#define __PLUGINS_ROSPROXY_PROCESSOR_H_


#include "bridge_processor.h"
#include "subscription_capability.h"

//TODO:move includes to cpp and use from namespace
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <logging/logger.h>

namespace fawkes {
  class Clock;
  class Logger;
  class Mutex;
}

//=================================   Subscription  ===================================
//its optional here wither to keep track of the subscribtions or act 
//as a pure proxy only forwading requests.For now, pure proxy

//=================================   Processor  ===================================

class WebSession;
class EventEmitter;

class RosBridgeProxyProcessor
: public BridgeProcessor
, public SubscriptionCapability
, public Callable
, public std::enable_shared_from_this<RosBridgeProxyProcessor>
{
 public:
  RosBridgeProxyProcessor(std::string prefix 
                          , fawkes::Logger *logger
                          , fawkes::Clock  *clock);

  virtual ~RosBridgeProxyProcessor();

 void init();

  std::shared_ptr<Subscription>  subscribe   ( std::string topic_name 
                                              , std::string id    
                                              , std::string compression
                                              , unsigned int throttle_rate  
                                              , unsigned int queue_length   
                                              , unsigned int fragment_size  
                                              , std::shared_ptr<WebSession> session);

  void                           unsubscribe ( std::string id
                                              , std::shared_ptr<Subscription> 
                                              , std::shared_ptr<WebSession> session ) ; 

  //handles session termination
  void  callback  ( EventType event_type , std::shared_ptr <EventEmitter> handler) ; 



private:
  void  create_rb_connection(std::shared_ptr <WebSession> new_session);
  void  close_rb_connection(std::shared_ptr <WebSession> session);

  void rb_send(std::shared_ptr <WebSession> session , std::string message);
  void rb_on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg);

  websocketpp::client<websocketpp::config::asio_client>                              rosbridge_endpoint_;
  std::map< websocketpp::connection_hdl , std::shared_ptr <WebSession> 
                                        , std::owner_less<websocketpp::connection_hdl>>           pears_map_;
  std::map< websocketpp::connection_hdl , std::shared_ptr <WebSession> 
                                        , std::owner_less<websocketpp::connection_hdl>>::iterator     it_pears_;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread>                             proxy_thread_;


  fawkes::Logger                                                                    *logger_; 
  fawkes::Clock                                                                     *clock_;
  fawkes::Mutex                                                                     *mutex_;

};



#endif
