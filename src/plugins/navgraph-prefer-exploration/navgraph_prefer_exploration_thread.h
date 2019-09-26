/***************************************************************************
 *  navgraph_prefer_exploration_thread.h - prefer exploration sub-graph
 *
 *  Created: Thu Apr 09 16:52:21 2015
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_PREFER_EXPLORATION_NAVGRAPH_PREFER_EXPLORATION_THREAD_H_
#define __PLUGINS_NAVGRAPH_PREFER_EXPLORATION_NAVGRAPH_PREFER_EXPLORATION_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>

#include <map>
#include <string>

namespace fawkes {
class NavGraphStaticListEdgeCostConstraint;
}

class NavGraphPreferExplorationThread : public fawkes::Thread,
                                        public fawkes::ClockAspect,
                                        public fawkes::LoggingAspect,
                                        public fawkes::ConfigurableAspect,
                                        public fawkes::BlackBoardAspect,
                                        public fawkes::NavGraphAspect,
                                        public fawkes::NavGraph::ChangeListener
{
public:
	NavGraphPreferExplorationThread();
	virtual ~NavGraphPreferExplorationThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	virtual void graph_changed() throw();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	float                                         cfg_cost_factor_;
	fawkes::NavGraphStaticListEdgeCostConstraint *edge_cost_constraint_;
};

#endif
