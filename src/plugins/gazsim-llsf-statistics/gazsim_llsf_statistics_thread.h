/***************************************************************************
 *  gazsim_llsf_statistics_plugin.h - Plugin generates a statistic about
 *     a game
 *
 *  Created: Mon Sep 23 17:12:33 2013
 *  Copyright  2013 Frederik Zwilling
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

#ifndef __PLUGINS_GAZSIM_LLSF_STATISTICS_THREAD_H_
#define __PLUGINS_GAZSIM_LLSF_STATISTICS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <interfaces/Position3DInterface.h>
#include <protobuf_msgs/GameState.pb.h>
#include <string.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


namespace fawkes {
  class Position3DInterface;
}

namespace mongo{
  class GridFS;
  class BSONObj;
}

typedef const boost::shared_ptr<llsf_msgs::GameState const> ConstGameStatePtr;

class LlsfStatisticsSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect,
  public fawkes::MongoDBAspect
{
 public:
  LlsfStatisticsSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  //suscribers for gazebo nodes (refbox messages forwarded by gazsim-llsfrbcomm)
  gazebo::transport::SubscriberPtr game_state_sub_;

  //Mongo stuff
  mongo::DBClientBase *mongodb_;
  std::map<std::string, mongo::GridFS*> mongogrids_;
  
  //handler functions
  void on_game_state_msg(ConstGameStatePtr &msg);

  //statistics
  std::string configuration_, replay_, namespace_, db_name_, collection_;
  int run_;
  int exp_points_, prod_points_;
  bool written_;

  void write_statistics();
};

#endif
