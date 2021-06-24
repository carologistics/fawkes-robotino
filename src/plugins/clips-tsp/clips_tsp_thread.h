
/***************************************************************************
 *  clips_tsp_thread.h - TSP feature for CLIPS
 *
 *  Created: Wed Oct 09 19:26:41 2013
 *  Copyright  2021 Gjorgji Nikolovski [gjorgji.nikolovski@alumni.fh-aachen.de]
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

#ifndef _PLUGINS_CLIPS_TSP_CLIPS_THREAD_H_
#define _PLUGINS_CLIPS_TSP_CLIPS_THREAD_H_

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <map>
#include <string>
#include <vector>

namespace fawkes {
}

class ClipsTSPThread : public fawkes::Thread,
                       public fawkes::LoggingAspect,
                       public fawkes::ConfigurableAspect,
                       public fawkes::CLIPSFeature,
                       public fawkes::CLIPSFeatureAspect
{
public:
	ClipsTSPThread();
	virtual ~ClipsTSPThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for CLIPSFeature
	virtual void clips_context_init(const std::string &                  env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::string solve_tsp(std::string env_name, std::string params);

	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;
};

#endif
