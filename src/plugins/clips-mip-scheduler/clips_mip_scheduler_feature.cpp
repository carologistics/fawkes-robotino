/***************************************************************************
 *  clips_mips_scheduler_feature.cpp - CLIPS MIP Scheduler feature
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

#include "clips_mip_scheduler_feature.h"

#include <core/threading/mutex_locker.h>
#include <logging/logger.h>

#include <clipsmm.h>
#include <fstream>

using namespace std;
//using namespace mips_scheduler;

/** @class MIPSCLIPSFeature "clips_mip_scheduler_feature.h"
 * Provide a MIP scheduler to a CLIPS environment.
 * @author Mostafa Gomaa
 */

/** Initialize the CLIPS feature.
 * @param logger The logger to use for logging in the feature
 */
MIPSCLIPSFeature::MIPSCLIPSFeature(fawkes::Logger *logger)
: CLIPSFeature("mip-scheduler"), logger_(logger)
{
}

/** Initialize the context and add a 'build-mip-model' CLIPS function.
 * @param env_name The name of the environment.
 * @param clips The CLIPS environment to add the parser functionality to.
 */
void
MIPSCLIPSFeature::clips_context_init(const string &                       env_name,
                                     fawkes::LockPtr<CLIPS::Environment> &clips)
{
	envs_[env_name] = clips;
	clips->add_function("build-mip-model",
	                    sigc::slot<void>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &MIPSCLIPSFeature::build_model),
	                                    env_name)));
}

/** Clean up a context.
 * @param env_name The name of the environment to clean.
 */
void
MIPSCLIPSFeature::clips_context_destroyed(const string &env_name)
{
	envs_.erase(env_name);
}

/** CLIPS function to build the MIPS model used for scheduling.
 * This generates the datasets and paramters of events from clips facts and
 * build the MIPS model
 * @param env_name The name of the calling environment
 */
void
MIPSCLIPSFeature::build_model(std::string env_name)
{
	fawkes::MutexLocker lock(envs_[env_name].objmutex_ptr());
	//CLIPS::Environment &env = **(envs_[env_name]);
}
