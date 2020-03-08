/***************************************************************************
 *  clips_mips_scheduler_feature.h - CLIPS MIP Scheduler feature
 *
 *  Created: Sun 8 Mar 2020 17:44:08 CET 17:44
 *  Copyright  2020  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

#ifndef _PLUGINS_CLIPS_MIP_SCHEDULER_FEATURE_H_
#define _PLUGINS_CLIPS_MIP_SCHEDULER_FEATURE_H_

#include <plugins/clips/aspect/clips_feature.h>

#include <map>
#include <string>

namespace CLIPS {
class Environment;
}

namespace fawkes {
class Logger;
}

class MIPSCLIPSFeature : public fawkes::CLIPSFeature
{
public:
	MIPSCLIPSFeature(fawkes::Logger *logger);
	//virtual ~MIPSCLIPSFeature();
	virtual void clips_context_init(const std::string &                  env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

private:
	void build_model(std::string env_name);

private:
	fawkes::Logger *                                           logger_;
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;
};

#endif /* !PLUGINS_CLIPS_MIP_SCHEDULER_FEATURE_H__ */
