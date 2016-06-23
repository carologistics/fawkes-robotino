
/***************************************************************************
 *  BridgeProcessor.h - Monitoring the CLIPS agents in LLSF via webview
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

#include <core/utils/lockptr.h>
#include <logging/logger.h>
#include <config/config.h>
#include <blackboard/blackboard.h>


#include <string>
#include <list>
#include <map>
#include <set>

#include <clipsmm.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/SkillerInterface.h>

namespace fawkes {
  class CLIPSEnvManager;
  class RobotinoLightInterface;
}

namespace CLIPS {
  class Environment;
}

class BridgeProcessor{
 public:
  BridgeProcessor(fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips,
				  fawkes::Logger *logger,
				  fawkes::Configuration *config, fawkes::BlackBoard *blackboard);

  virtual ~BridgeProcessor();

  std::map<std::string,std::string> getAgentState();

  static int TestInt();

 private:
  fawkes::LockPtr<fawkes::CLIPSEnvManager> clips_env_mgr_;
  fawkes::Logger       *logger_;
  fawkes::Configuration *config_;
  fawkes::BlackBoard *blackboard_;

  fawkes::RobotinoLightInterface *light_if_;
  fawkes::SkillerInterface *skiller_if_;

  std::list<std::string> errors_;

  
  CLIPS::Fact::pointer get_first_fact(std::string env_name);
  CLIPS::Fact::pointer get_fact(std::string env_name, std::string tmpl_name,
				std::map <std::string, std::string> constraints = {});
  CLIPS::Fact::pointer get_next_fact(CLIPS::Fact::pointer start, std::string tmpl_name,
				std::map <std::string, std::string> constraints = {});
  std::set<CLIPS::Fact::pointer> get_all_facts(std::string env_name, std::string tmpl_name,
				std::map <std::string, std::string> constraints = {});

  const char* get_slot(CLIPS::Fact::pointer fact, std::string slot, int entry = 0);
  std::string get_slot_string(CLIPS::Fact::pointer fact, std::string slot, int entry = 0);

  //config values
  std::string robotino1_, robotino2_, robotino3_;
  std::string env_name_;
  std::string own_name_;
  std::string refresh_interval_;
};

#endif
