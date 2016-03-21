
/***************************************************************************
 *  RosProxyProcessor.h - Monitoring the CLIPS agents in LLSF via webview
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

// #include <config/config.h>

// #include <string>
// #include <list>
// #include <map>
// #include <set>

// #include <blackboard/interface_listener.h>
// #include <interface/interface.h>
// #include <core/exceptions/system.h>
// #include <core/threading/mutex_locker.h>

#include "bridge_processor.h"
#include "subscription_capability.h"


class WebSession;


//=================================   Subscription  ===================================

class BlackBoardSubscription
: public Subscription 
, public fawkes::BlackBoardInterfaceListener
{
  public:
    BlackBoardSubscription(std::string topic_name 
                          , std::string processor_prefix 
                          , fawkes::Clock *clock
                          , fawkes::BlackBoard *blackboard
                          , fawkes::Interface *interface);

    ~BlackBoardSubscription();

    fawkes::Interface* get_interface_ptr();

    void activate_impl();
    void deactivate_impl();
    void finalize_impl();// finalize oper and listeners interfaces 
    std::string   serialize(std::string op
                          , std::string topic
                          , std::string id);

    void bb_interface_data_changed(fawkes::Interface *interface) throw();

  private:
    fawkes::BlackBoard         *blackboard_;
    fawkes::Interface          *interface_;
};


//=================================   Processor  ===================================

class BridgeBlackBoardProcessor
: public BridgeProcessor,
  public SubscriptionCapability
{
 public:
  BridgeBlackBoardProcessor(std::string prefix 
                          , fawkes::Logger *logger
                          , fawkes::Configuration *config
                          , fawkes::BlackBoard *blackboard 
                          , fawkes::Clock *clock)
;

  virtual ~BridgeBlackBoardProcessor();

  std::shared_ptr<Subscription>  subscribe   ( std::string topic_name 
                                              , std::string id    
                                              , std::string compression
                                              , unsigned int throttle_rate  
                                              , unsigned int queue_length   
                                              , unsigned int fragment_size  
                                              , std::shared_ptr<WebSession> session);

  void  unsubscribe ( std::string id
                    , std::shared_ptr<Subscription> 
                    , std::shared_ptr<WebSession> session ) ; 

private:
  fawkes::Logger         *logger_;
  fawkes::Configuration  *config_;
  fawkes::BlackBoard     *blackboard_;
  fawkes::Clock          *clock_;

  std::map<std::string, fawkes::Interface *>::iterator ifi_;

};



#endif
