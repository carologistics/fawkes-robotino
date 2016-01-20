
/***************************************************************************
 *  BridgeBlackboardProcessor.h - Monitoring the CLIPS agents in LLSF via webview
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

#ifndef __PLUGINS_BRIDGE_BRIDGE_PROCESSOR_H_
#define __PLUGINS_BRIDGE_BRIDGE_PROCESSOR_H_

#include <config/config.h>

#include <string>
#include <list>
#include <map>
#include <set>

#include <blackboard/interface_listener.h>
#include <interface/interface.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex_locker.h>

#include "bridge_processor.h"
#include "subscription_capability.h"

namespace fawkes {
  class Clock;
  class BlackBoard;
  class Interface;
  class Logger;
}

class WebSession;

class BridgeBlackBoardProcessor
: public BridgeProcessor,
  public SubscriptionCapability
{
 public:
  BridgeBlackBoardProcessor( fawkes::Logger *logger
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

  void                            unsubscribe ( std::string id
                                            , std::shared_ptr<Subscription> 
                                            , std::shared_ptr<WebSession> session ) ; 

private:
  fawkes::Logger       *logger_;
  fawkes::Configuration *config_;
  fawkes::BlackBoard *blackboard_;
  fawkes::Clock       *clock_;

  std::map<std::string, fawkes::Interface *>interfaces_;
  std::map<std::string, fawkes::Interface *>::iterator ifi_;

};


class BlackBoardSubscription
: public Subscription,
, public BlackBoardInterfaceListener
{
  public:
    BlackBoardSubscription(std::string topic_name 
                          , std::string processor_prefix 
                          , fawkes::Clock *clock
                          , fawkes::BlackBoard *blackboard);

    ~BlackBoardSubscription();

    Interface* get_interface_ptr();

    void  activate();
    void  deactivate();
    void  finalize();// finalize oper and listeners interfaces 

    void bb_interface_data_changed(Interface *interface) throw();

  private:
    Inteface                   *interface_;
    fawkes::BlackBoard         *blackboard_;
};

#endif
