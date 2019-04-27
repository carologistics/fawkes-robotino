/***************************************************************************
 *  navgraph_clusters_thread.h - block paths based on laser clusters
 *
 *  Created: Sun Jul 13 15:30:03 2014
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

#ifndef __PLUGINS_NAVGRAPH_CLUSTERS_NAVGRAPH_CLUSTERS_THREAD_H_
#define __PLUGINS_NAVGRAPH_CLUSTERS_NAVGRAPH_CLUSTERS_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <navgraph/navgraph_node.h>

#include <list>
#include <string>
#include <tuple>

namespace fawkes {
class Position3DInterface;
class Time;
class NavGraphEdgeConstraint;
class NavGraphEdgeCostConstraint;
class NavGraphGeneratorInterface;
} // namespace fawkes

class NavGraphGenerator2014Thread : public fawkes::Thread,
                                    public fawkes::ClockAspect,
                                    public fawkes::LoggingAspect,
                                    public fawkes::ConfigurableAspect,
                                    public fawkes::BlackBoardAspect
{
public:
	NavGraphGenerator2014Thread();
	virtual ~NavGraphGenerator2014Thread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void        add_node_edge(const fawkes::NavGraphNode &n);
	void        add_node_node(const fawkes::NavGraphNode &n);
	std::string gen_id();

private:
	unsigned int                        last_id_;
	fawkes::NavGraphGeneratorInterface *navgen_if_;
};

#endif
