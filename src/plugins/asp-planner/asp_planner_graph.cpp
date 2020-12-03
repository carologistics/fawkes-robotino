/***************************************************************************
 *  asp_planner_graph.cpp - ASP-based planner plugin interaction with the
 *navgraph
 *
 *  Created on Fri Dec 16 18:03:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
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

#include "asp_planner_thread.h"

#include <core/threading/mutex_locker.h>

using fawkes::MutexLocker;

/*
 * @brief Helper function to calculate the center point of each zone.
 * @return An array with the center points. Zone i is in the element [i]. [0] is
kept "empty".
 * @todo Why can't we modify the array as constexpr?
 *
static auto
calculateZoneCoords(void) noexcept
{
        std::array<float[2], 25> ret{};

        for ( int i = 0; i < 24; ++i )
        {
                const auto row = i % 4;
                auto column    = i / 4;

                //Real columns:         6  5  4  1  2  3
                //To the base of 0:     5  4  3  0  1  2
                //For the calculation: -3 -2 -1  0  1  2
                if ( column >= 3 )
                {
                        column = 2 - column;
                } //if ( column >= 3 )

                ret[i + 1][0] = column * 2. + 1.;
                ret[i + 1][1] = row * 1.5 + .75;
        } //for ( int i = 0; i < 24; ++i )

        return ret;
}*/

/**
 * @brief Will be called if the navgraph is changed. Will add the property
 * "ASP-Location" to all nodes which are used by the ASP encoding.
 */
void
AspPlannerThread::graph_changed(void) noexcept
{
	MutexLocker navgraphLocker(navgraph.objmutex_ptr());
	MutexLocker distanceLocker(&NavgraphDistanceMutex);
	logger->log_error(LoggingComponent, "Navgraph update!");
	for (auto node : navgraph->nodes()) {
		if (!node.has_property(NodePropertyASP) && NavgraphNodesForASP.count(node.name())) {
			node.set_property(NodePropertyASP, true);
			navgraph->update_node(node);
			logger->log_warn(LoggingComponent,
			                 "Setting UpdateNavgraphDistance from %s to true. %s",
			                 UpdateNavgraphDistances ? "true" : "false",
			                 __func__);
			UpdateNavgraphDistances = true;
			NodesToFind.erase(node.name());
			logger->log_info(LoggingComponent, "graph_changed: %s", node.name().c_str());
		} // if ( !node.has_property(NodePropertyASP) &&
		  // NavgraphNodesForASP.count(node.name()) )
	}   // for ( auto node : navgraph->nodes() )
	return;
}

/**
 * @brief Fill the NavgraphNodesForASP with the nodes we export to ASP.
 * @param[in] lockWorldMutex If the world mutex should be locked.
 */
void
AspPlannerThread::fillNavgraphNodesForASP(const bool lockWorldMutex)
{
	MutexLocker locker(&NavgraphDistanceMutex);
	NavgraphNodesForASP.clear();
	// Maschinen * Seiten + Zonen
	NavgraphNodesForASP.reserve(6 * 2 + 12 + 1);

	// The machines.
	std::string name;
	name.reserve(7);

	Clingo::Symbol arguments[3];
	assert(TeamColor);
	arguments[0] = Clingo::String(TeamColor);
	decltype(NavgraphNodesForASP.begin()) lastInsertion;
	for (const auto &machine : {"BS", "CS1", "CS2", "RS1", "RS2", "DS"}) {
		name = TeamColor;
		name += "-";
		name += machine;
		name += "-S";

		arguments[1] = Clingo::String(machine);

		for (const auto &side : {"I", "O"}) {
			name.back()   = side[0];
			arguments[2]  = Clingo::String(side);
			lastInsertion = NavgraphNodesForASP.insert({name, Clingo::Function("m", {arguments, 3})});
		} // for ( const auto& side : {"I", "O"} )
	}   // for ( const auto& machine : {"BS", "CS1", "CS2", "RS1", "RS2", "DS"} )

	// Remove the output side of the DS.
	assert(lastInsertion->first == std::string(TeamColor) + "-DS-O");
	NavgraphNodesForASP.erase(lastInsertion);
	NavgraphNodesForASP.insert({std::string(TeamColor) + "-ins-out", Clingo::String("ins-out")});

	//! @todo Do we need nodes for the zones? (I think yes.)
	const auto dummyNode = std::string(TeamColor) + "-ins-in";
	//	static const auto zones(calculateZoneCoords());
	//	MutexLocker locker(navgraph.objmutex_ptr());
	MutexLocker worldLocker(&WorldMutex, lockWorldMutex);
	for (auto zone : ZonesToExplore) {
		NavgraphNodesForASP.insert({dummyNode, Clingo::Function("z", {Clingo::Number(zone)})});
		//		const auto node = navgraph->closest_node(zones[zone][0],
		// zones[zone][1], false, NodePropertyASP);
	} // for ( auto zone : ZonesToExplore )

	logger->log_warn(LoggingComponent,
	                 "Setting UpdateNavgraphDistance from %s to true. %s",
	                 UpdateNavgraphDistances ? "true" : "false",
	                 __func__);
	UpdateNavgraphDistances = true;
	return;
}

/**
 * @brief Updates the navgraph distances.
 * @note Assumes, that NavgraphDistanceMutex is locked.
 */
void
AspPlannerThread::updateNavgraphDistances(void)
{
	static bool done = false;

	logger->log_warn(LoggingComponent,
	                 "Setting UpdateNavgraphDistance from %s to false. %s",
	                 UpdateNavgraphDistances ? "true" : "false",
	                 __func__);
	UpdateNavgraphDistances = false;

	if (done) {
		logger->log_error(LoggingComponent,
		                  "updateNavgraphDistances called, but "
		                  "we already released the externals!");
		return;
	} // if ( done )

	NavgraphDistances.clear();
	NavgraphDistances.reserve(NavgraphNodesForASP.size() * NavgraphNodesForASP.size());

	MutexLocker navgraphLocker(navgraph.objmutex_ptr());

	auto distanceToDuration = [this](const float distance) noexcept {
		constexpr float constantCosts   = 2.16988;
		constexpr float costPerDistance = 1.55322;
		return std::min(static_cast<int>(constantCosts + distance * costPerDistance), MaxDriveDuration);
	};

	const auto end = NavgraphNodesForASP.end();
	for (auto from = NavgraphNodesForASP.begin(); from != end; ++from) {
		const auto &fromNode = navgraph->node(from->first);
		for (auto to = from; ++to != end;) {
			const auto &toNode = navgraph->node(to->first);

			/* If we use invalid nodes or don't find a path take a big number for the
       * duration. This is done because initally the nav graph is not connected
       * and we wouldn't set durations. By that the ASP solver wouldn't assign
       * tasks, because it could not calculate the estimated time for the tasks.
       * The default value should be an upper bound on the driving duration a
       * robot would take without mobile obstacles, i.e. drive from one end of
       * the field to the other side, possibly around machines, but there
       * is no replanning because of other robots. */
			auto findDuration = [this, &toNode, &fromNode, distanceToDuration](void) {
				if (fromNode.is_valid() && toNode.is_valid()) {
					const auto path = navgraph->search_path(fromNode, toNode);
					if (!path.empty()) {
						return distanceToDuration(path.cost());
					} // if ( !path.empty() )
				}   // if ( fromNode.is_valid() && toNode.is_valid() )
				return MaxDriveDuration;
			};

			const auto duration = realGameTimeToAspGameTime(findDuration());

			Clingo::Symbol arguments[3] = {from->second, to->second, Clingo::Number(duration)};

			NavgraphDistances.emplace_back(Clingo::Function("setDriveDuration", {arguments, 3}));
		} // for ( auto to = from; ++to != end; )
	}   // for ( auto from = NavgraphDistances.begin(); from != end; ++from )

	done = NodesToFind.empty();
	return;
}

/**
 * @brief Returns the nearest asp location to a given coordinate.
 * @param[in] x The x coordination.
 * @param[in] y The y coordination.
 * @return The atom of the location.
 */
Clingo::Symbol
AspPlannerThread::nearestLocation(const float x, const float y)
{
	MutexLocker navgraphLocker(navgraph.objmutex_ptr());
	MutexLocker durationLocker(&NavgraphDistanceMutex);
	auto        node = navgraph->closest_node(x, y, false, NodePropertyASP);
	if (!node.is_valid()) {
		throw fawkes::Exception("No ASP-NavGraph-Node for (%f, %f) found!", x, y);
	} // if ( !node.is_valid() )

	return NavgraphNodesForASP.find(node.name())->second;
}
