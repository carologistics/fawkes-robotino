/***************************************************************************
 *  webtools_bridge_plugin.cpp - Websocket access to diffrent components mimicing the rosbridge protocol
 *
 *  Created: Wed Jan 13 16:33:00 2016 
 *  Copyright  2016 Mostafa Gomaa 
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


#include "webtools_bridge_thread.h"
 
#include <tf/types.h>
#include <interfaces/Position3DInterface.h>

#include "Web_server.cpp"
#include "bridge_manager.h"

#include "subscription_capability_manager.h"
#include "advertisment_capability_manager.h"
#include "service_capability_manager.h"

#include "blackboard_processor.h"
#include "rosbridge_proxy_processor.h"
#include "clips_processor.h"


using namespace fawkes;



WebtoolsBridgeThread::WebtoolsBridgeThread()
  : Thread("BridgeThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
  
}


WebtoolsBridgeThread::~WebtoolsBridgeThread()
{
}


void
WebtoolsBridgeThread::init()
{
//  pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");
//  world_ =new World("f");

  logger-> log_info("I CAN SEE THE WORLD","YAAY");
 // time_var_=new Time(clock);
 // time_var_->stamp();


  bridge_manager_=std::make_shared<BridgeManager> ();

  std::shared_ptr<SubscriptionCapabilityManager> subscription_cpm=
                                              std::make_shared<SubscriptionCapabilityManager>();

  std::shared_ptr<AdvertismentCapabilityManager> advertisment_cpm=
                                              std::make_shared<AdvertismentCapabilityManager>();

  std::shared_ptr<ServiceCapabilityManager> service_cpm=
                                              std::make_shared<ServiceCapabilityManager>();

  //register capability managers
  bridge_manager_->register_operation_handler("subscribe" , subscription_cpm );
  bridge_manager_->register_operation_handler("unsubscribe" , subscription_cpm );

  bridge_manager_->register_operation_handler("advertise" , advertisment_cpm );
  bridge_manager_->register_operation_handler("unadvertise" , advertisment_cpm );

  bridge_manager_->register_operation_handler("call_service" , service_cpm );
  
  //register processors
  bridge_manager_->register_processor(std::make_shared<BridgeBlackBoardProcessor> ("blackboard"
                                                                                  , logger
                                                                                  , config
                                                                                  , blackboard
                                                                                  , clock));

  bridge_manager_->register_processor(std::make_shared<RosBridgeProxyProcessor> ("/"
                                                                                  , logger
                                                                                  , clock));

  bridge_manager_->register_processor(std::make_shared<ClipsProcessor> ("clips"
                                                                        , logger
                                                                        , config
                                                                        , clock
                                                                        , clips_env_mgr));


  // bridge_manager_->register_processor(std::make_shared<ClipsProcessor> ("/"
  //                                                                       , logger
  //                                                                       , clock));

  web_server_=websocketpp::lib::make_shared<Web_server>(logger, bridge_manager_);  
  web_server_->run(6060);
}

void
WebtoolsBridgeThread::finalize()
{
 // blackboard->close(pose_if_);
  // delete proc_;
  // delete fawkes_bridge_manager_;
  // delete web_server_;//when do i actually need to teminate it..noty sure yet
}

void
WebtoolsBridgeThread::loop()
{
 //  fawkes_bridge_manager_->loop();
  // if (pose_if_->has_writer()) {
  //   pose_if_->read();
  //   double *r = pose_if_->rotation();
  //   tf::Quaternion pose_q(r[0], r[1], r[2], r[3]);
  //   logger->log_info(name(), "Pose: (%f,%f,%f)", pose_if_->translation(0),
  //                    pose_if_->translation(1), tf::get_yaw(pose_q));
  // } else {
  //   logger->log_warn(name(), "No writer for pose interface");
  // }

  // //  proc_->getAgentState();

  //Palying around with time

  // time_var_->stamp();
  // fawkes::Time now(clock);
  // now.stamp();
 
  //   std::cout<< "time_is" << now.in_msec() <<std::endl;
  //   std::cout<< "time_is" << (*time_var_).in_msec() <<std::endl;
    
  //   std::cout<< "time_is" << now.in_msec() - (*time_var_).in_msec() <<std::endl;
    
  //   std::cout<< "0000000000000000000000000000000000000"  <<std::endl;

}
