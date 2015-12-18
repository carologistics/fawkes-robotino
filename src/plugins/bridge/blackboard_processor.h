
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
 
#include "interfaces/ibridge_processor.h"


namespace fawkes {
  class BlackBoard;
  class Interface;
  class Logger;
}

class BridgeBlackBoardProcessor: public IBridgeProcessor
 {
 public:
  BridgeBlackBoardProcessor(fawkes::Logger *logger,fawkes::Configuration *config, fawkes::BlackBoard *blackboard);

  virtual ~BridgeBlackBoardProcessor();

  bool subscribe(std::string full_name);
  std::string read_single_topic(std::string);

  void publish();
  void postInterface(fawkes::Interface* iface);

 private:
  fawkes::Logger       *logger_;
  fawkes::Configuration *config_;
  fawkes::BlackBoard *blackboard_;

  std::map<std::string, fawkes::Interface *>interfaces_;
  std::map<std::string, fawkes::Interface *>::iterator ifi_;
};

#endif
