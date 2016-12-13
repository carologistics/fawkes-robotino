/***************************************************************************
 *  asp_planer_clingo.cpp - ASP-based planer plugin clingo interface
 *
 *  Created on Tue Aug 23 13:57:02 2016
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

#include "asp_planer_thread.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>
#include <experimental/string_view>

//! @todo Replace include and using, once C++17 is implemented properly.
using std::experimental::string_view;

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <plugins/asp/aspect/clingo_access.h>

using fawkes::MutexLocker;

/**
 * @enum InterruptSolving
 * @brief States the current interrupt request.
 * @note Sort by priority. We use operator> when setting the value.
 *
 * @var InterruptSolving::Not
 * @brief Do not interrupt.
 *
 * @var InterruptSolving::JustStarted
 * @brief Only interrupt if the solving was just started.
 *
 * @var InterruptSolving::Normal
 * @brief Do interrupt, if the plan isn't to old.
 *
 * @var InterruptSolving::Critical
 * @brief Interrupt in any case.
 */

/**
 * @struct GroundRequest
 * @brief A simple container for a ground request.
 *
 * @property GroundRequest::Name
 * @brief The name of the program to ground.
 *
 * @property GroundRequest::Params
 * @brief The parameters for the request.
 *
 * @property GroundRequest::AddTick
 * @brief If the current tick should be appended to the parameters.
 */

/**
 * @property AspPlanerThread::LookAhaed
 * @brief How many seconds the planer should look ahaed.
 *
 * @property AspPlanerThread::MaxOrders
 * @brief The maximum amount of orders we expect.
 *
 * @property AspPlanerThread::MaxQuantity
 * @brief The maximum quantity for an order we expect.
 *
 * @property AspPlanerThread::MoreModels
 * @brief If we want to have more than one model (if available) from the solver.
 *
 * @property AspPlanerThread::LastTick
 * @brief The last tick for the asp program.
 *
 * @property AspPlanerThread::Horizon
 * @brief The horizon, up to which point (measured in gametime in seconds) the planer should plan.
 *
 * @property AspPlanerThread::Past
 * @brief Up to which point the past has been fixed.
 *
 * @property AspPlanerThread::LastModel
 * @brief When the last model was found, i.e. how old is our plan.
 *
 * @property AspPlanerThread::SolvingStarted
 * @brief When the solving was started.
 *
 * @property AspPlanerThread::MachinesFound
 * @brief How many machines we have found.
 *
 * @property AspPlanerThread::StillNeedExploring
 * @brief If we still need exploring.
 *
 * @property AspPlanerThread::CompleteRestart
 * @brief If we should restart the asp solver from the very beginning.
 *
 * @property AspPlanerThread::TimeResolution
 * @brief How many real time seconds are one time unit for ASP.
 *
 * @property AspPlanerThread::MaxDriveDuration
 * @brief The maximum we expect a robot needs for driving to some point.
 *
 * @property AspPlanerThread::Robots
 * @brief The robots that are available.
 *
 *
 * @property AspPlanerThread::UpdateNavgraphDistances
 * @brief If the distances between the known locations should be recalculated. (Because the navgraph was changed.)
 *
 * @property AspPlanerThread::NavgraphDistances
 * @brief The currently assigned externals for the distances.
 *
 * @property AspPlanerThread::NavgraphNodesForASP
 * @brief Mapping of all navgraph nodes we export to asp to their corrosponding symbols.
 *
 * @property AspPlanerThread::NodePropertyASP
 * @brief The property which is set to nodes which are exported to ASP.
 *
 *
 * @property AspPlanerThread::RequestMutex
 * @brief Protects AspPlanerThread::Requests.
 *
 * @property AspPlanerThread::Interrupt
 * @brief If the solving should be interrupted for the new requests.
 *
 * @property AspPlanerThread::SentCancel
 * @brief If we already sent the cancel command to the ASP Solver.
 *
 * @property AspPlanerThread::Requests
 * @brief Stores everything we have to add to the solver for the next iteration.
 */

/**
 * @brief Fill the NavgraphNodesForASP with the nodes we export to ASP.
 */
void
AspPlanerThread::fillNavgraphNodesForASP(void)
{
	NavgraphNodesForASP.clear();
	//Maschinen * Seiten + Zonen
	NavgraphNodesForASP.reserve(6 * 2 + 24);

	//The machines.
	std::string name;
	name.reserve(7);
	name = TeamColor;
	name += "-";

	Clingo::Symbol arguments[3];
	arguments[0] = Clingo::String(TeamColor);
	for ( const auto& machine : {"BS", "CS1", "CS2", "RS1", "RS2", "DS"} )
	{
		name += machine;
		name += "-S";

		arguments[1] = Clingo::String(machine);

		for ( const auto& side : {"I", "O"} )
		{
			name.back() = side[0];
			arguments[2] = Clingo::String(side);
			NavgraphNodesForASP.insert({name, Clingo::Function("m", {arguments, 3})});
		} //for ( const auto& side : {"I", "O"} )
	} //for ( const auto& machine : {"BS", "CS1", "CS2", "RS1", "RS2", "DS"} )

	if ( StillNeedExploring )
	{
		//! @todo Do we need nodes for the zones? (I think yes.)
		const auto dummyNode = std::string(TeamColor) + "-ins-out";
//		static const auto zones(calculateZoneCoords());
//		MutexLocker locker(navgraph.objmutex_ptr());
		for ( auto zone = 1; zone <= 24; ++zone )
		{
			NavgraphNodesForASP.insert({dummyNode, Clingo::Function("z", {Clingo::Number(zone)})});
//			const auto node = navgraph->closest_node(zones[zone][0], zones[zone][1], false, NodePropertyASP);
		} //for ( auto zone = 1; zone <= 24; ++zone )
	} //if ( StillNeedExploring )
	return;
}

/**
 * @brief Will be called if the navgraph is changed. Will add the property "ASP-Location" to all nodes which are used by
 *        the ASP encoding.
 */
void
AspPlanerThread::graph_changed(void) noexcept
{
	MutexLocker locker(navgraph.objmutex_ptr());
	logger->log_error(LoggingComponent, "Navgraph update!");
	for ( auto node : navgraph->nodes() )
	{
		logger->log_warn(LoggingComponent, "Node: %s %s", node.name().c_str(), node.unconnected() ? "false" : "true");
		if ( !node.has_property(NodePropertyASP) && NavgraphNodesForASP.count(node.name()) )
		{
			node.set_property(NodePropertyASP, true);
			navgraph->update_node(node);
		} //if ( !node.has_property(NodePropertyASP) && NavgraphNodesForASP.count(node.name()) )
	} //for ( auto node : navgraph->nodes() )

	UpdateNavgraphDistances = true;
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in the constructor.
 */
void
AspPlanerThread::constructClingo(void)
{
	//Using the aspect.
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in init().
 */
void
AspPlanerThread::initClingo(void)
{
	MutexLocker navgraphLocker(navgraph.objmutex_ptr());
	navgraph->add_change_listener(this);
	navgraphLocker.unlock();

	MutexLocker locker(ClingoAcc.objmutex_ptr());
	ClingoAcc->registerModelCallback(std::make_shared<std::function<bool(void)>>([this](void) { return newModel(); }));
	ClingoAcc->registerFinishCallback(std::make_shared<std::function<void(Clingo::SolveResult)>>(
		[this](const Clingo::SolveResult result)
		{
			solvingFinished(result);
			return;
		}));
	ClingoAcc->setGroundCallback([this](auto... args) { this->groundFunctions(args...); return; });

	constexpr auto infix = "planer/";
	const auto prefixLen = std::strlen(ConfigPrefix), infixLen = std::strlen(infix);
	char buffer[prefixLen + std::max<size_t>(infixLen + 20, 40)];
	const auto suffix = buffer + prefixLen + infixLen;
	std::strcpy(buffer, ConfigPrefix);
	std::strcpy(buffer + prefixLen, infix);

	std::strcpy(suffix, "debug-level");
	ClingoAcc->DebugLevel = static_cast<fawkes::ClingoAccess::DebugLevel_t>(config->get_int(buffer));

	std::strcpy(suffix, "max-orders");
	MaxOrders = config->get_uint(buffer);
	std::strcpy(suffix, "max-quantity");
	MaxQuantity = config->get_uint(buffer);
	std::strcpy(suffix, "more-models");
	MoreModels  = config->get_bool(buffer);
	std::strcpy(suffix, "look-ahaed");
	LookAhaed = config->get_uint(buffer);
	std::strcpy(suffix, "time-resolution");
	TimeResolution = config->get_uint(buffer);
	std::strcpy(suffix, "robots");
	Robots = config->get_strings(buffer);

	std::strcpy(buffer + prefixLen, "time-estimations/max-drive-duration");
	MaxDriveDuration = config->get_uint(buffer);

	loadFilesAndGroundBase(locker);

	Orders.reserve(MaxOrders);
	RingColors.reserve(4);
	return;
}

/**
 * @brief Resets clingo and refills it with the needed information.
 * @param[in] aspLocker The locked locker for clingo.
 * @param[in] reqLocker The locked locker for the requests.
 */
void
AspPlanerThread::resetClingo(MutexLocker& aspLocker, MutexLocker& reqLocker)
{
	ClingoAcc->reset();
	NavgraphDistances.clear();
	loadFilesAndGroundBase(aspLocker);
	fillNavgraphNodesForASP();
	Horizon = Past = 0;
	LastTick = 0;
	LastModel = SolvingStarted = LastPlan = fawkes::Time();
	UpdateNavgraphDistances = true;
	Requests.clear();

	MutexLocker robLocker(&RobotsMutex);
	reqLocker.unlock();
	setTeam();
	auto groundPastActions = [this](auto map)
		{
			auto iter = map.begin();
			const auto end = map.end();
			while ( iter != end )
			{
				if ( !StillNeedExploring && std::strcmp(iter->second.Params[1].name(), "explore") == 0 )
				{
					//Remove all references to the exploration phase, we don't need this information!
					iter = map.erase(iter);
				} //if ( !StillNeedExploring && std::strcmp(iter->second.Params[1].name(), "explore") == 0 )
				else
				{
					GroundRequest copy(iter->second);
					queueGround(std::move(copy));
				} //else -> if ( !StillNeedExploring && std::strcmp(iter->second.Params[1].name(), "explore")=0)
			} //while ( iter != end )
			return;
		};

	if ( !StillNeedExploring )
	{
		updateNavgraphDistances();
	} //if ( !StillNeedExploring )

	for ( const auto& pair : RobotInformations )
	{
		newTeamMate(pair.first, pair.second);
	} //for ( const auto& pair : RobotInformations )

	for ( const auto& color : RingColors )
	{
		setRingColor(color);
	} //for ( const auto& color : RingColors )

	for ( const auto& order : Orders )
	{
		addOrder(order);
	} //for ( const auto& order : Orders )

	groundPastActions(RobotTaskBegin);
	groundPastActions(RobotTaskUpdate);
	groundPastActions(TaskSuccess);
	reqLocker.relock();
	return;
}


/**
 * @brief Helper function to calculate the center point of each zone.
 * @return An array with the center points. Zone i is in the element [i]. [0] is kept "empty".
 * @todo Why can't we modify the array as constexpr?
 */
/*static auto
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
 * @brief Does the loop for clingo.
 */
void
AspPlanerThread::loopClingo(void)
{
	MutexLocker aspLocker(ClingoAcc.objmutex_ptr());
	MutexLocker reqLocker(&RequestMutex);
	if ( ClingoAcc->solving() )
	{
		if ( interruptSolving() && !SentCancel )
		{
			assert(!Requests.empty());
			logger->log_warn(LoggingComponent, "Interrupt solving process for new information: %d", Requests.size());
			ClingoAcc->cancelSolving();
			SentCancel = true;
		} //if ( interruptSolving() && !SentCancel )
		return;
	} //if ( ClingoAcc->solving() )

	if ( CompleteRestart ) {
		resetClingo(aspLocker, reqLocker);
	} //if ( CompleteRestart )
	else if ( UpdateNavgraphDistances )
	{
		updateNavgraphDistances();
	} //else if ( UpdateNavgraphDistances )
	else if ( Requests.empty() )
	{
		//Nothing todo.
		//! @todo Increment the lookahaed?!? If yes, reset it on the complete restart!
		return;
	} //if ( Requests.empty() )

	SentCancel = false;
	//Copy requests, so the lock can be released and new events be recorded.
	decltype(Requests) requests;
	std::swap(requests, Requests);
	Interrupt = InterruptSolving::Not;
	reqLocker.unlock();

	std::vector<Clingo::Part> parts;
	auto groundRequests = [&parts,&requests,this](void)
		{
			parts.reserve(requests.size());
			for ( GroundRequest& request : requests )
			{
				parts.emplace_back(request.Name, request.Params);
			} //for ( GroundRequest& request : requests )

			//We are not allowed to clear requests before grounding! The params are just stored as "pointers".
			ClingoAcc->ground(parts);
			parts.clear();
			return;
		};
	groundRequests();

	//Unset the old horizon.
	Clingo::Symbol horizonValueSymbol = Clingo::Number(Horizon);
	Clingo::SymbolSpan horizonSpan(&horizonValueSymbol, 1);
	Clingo::Symbol horizonSymbol = Clingo::Function("horizon", horizonSpan);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::False);

	//Handle the past.
	requests.clear();
	setPast(requests);
	groundRequests();

	//Set the new horizon.
	const auto oldHorizon(std::move(Horizon));
	//Use a fixed look ahaed of one minute for the exploration phase.
	const auto usedLookAhaed = StillNeedExploring ? 90u : LookAhaed;
	//Cap the horizon at game end.
	Horizon = realGameTimeToAspGameTime(std::min(GameTime + usedLookAhaed, (15 + 4) * 60u));
	horizonValueSymbol = Clingo::Number(Horizon);
	horizonSymbol = Clingo::Function("horizon", horizonSpan);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::True);

	parts.reserve(Horizon - oldHorizon);
	Clingo::SymbolVector symbols;
	symbols.reserve(Horizon - oldHorizon);
	for ( auto i = oldHorizon + 1; i <= Horizon; ++i )
	{
		symbols.emplace_back(Clingo::Number(i));
		parts.emplace_back("transition", Clingo::SymbolSpan(&symbols.back(), 1));
	} //for ( auto i = oldHorizon + 1; i <= Horizon; ++i )

	ClingoAcc->ground(parts);
	reqLocker.relock();
	if ( interruptSolving() )
	{
		logger->log_info(LoggingComponent, "Solving would be interrupted immediately, just don't start it.");
		return;
	} //if ( Interrupt != InterruptSolving::Not )
	reqLocker.unlock();
	ClingoAcc->startSolving();
	SolvingStarted = clock->now();
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in finalize().
 */
void
AspPlanerThread::finalizeClingo(void)
{
	MutexLocker locker(ClingoAcc.objmutex_ptr());
	ClingoAcc->cancelSolving();
	return;
}

/**
 * @brief Says if the solving process should be interrupted.
 * @return If the solving should be interrupted.
 * @note RequestLock must be locked before calling this method and unlocked when appropriate!
 */
bool
AspPlanerThread::interruptSolving(void) const noexcept
{
	const auto now(clock->now());
	const auto diff(now - LastModel);
	switch ( Interrupt )
	{
		case InterruptSolving::Not         : break;
		case InterruptSolving::JustStarted : /** @todo Do it! */break;
		case InterruptSolving::Normal      :
		{
			//! @todo Read threashold from config.
			static const fawkes::Time threshold(10, 0);
			if ( diff <= threshold )
			{
				return true;
			} //if ( diff <= threshold )
			break;
		} //case InterruptSolving::Normal
		case InterruptSolving::Critical    : return true;
	} //switch ( Interrupt )
	return false;
}

/**
 * @brief Loads the ASP files into the solver and grounds the base program.
 * @param[in, out] locker The locker used to lock ClingoAcc. Has to be locked at the beginning.
 */
void
AspPlanerThread::loadFilesAndGroundBase(MutexLocker& locker)
{
	constexpr auto infix = "planer/";
	const auto prefixLen = std::strlen(ConfigPrefix), infixLen = std::strlen(infix);
	char buffer[prefixLen + infixLen + 20];
	const auto suffix = buffer + prefixLen + infixLen;
	std::strcpy(buffer, ConfigPrefix);
	std::strcpy(buffer + prefixLen, infix);

	std::strcpy(suffix, "program-path");
	const auto path  = config->get_string(buffer);
	std::strcpy(suffix, "program-files");
	const auto files = config->get_strings(buffer);

	logger->log_info(LoggingComponent, "Loading program files from %s. Debug state: %d", path.c_str(),
		ClingoAcc->DebugLevel.load());
	for ( const auto& file : files )
	{
		ClingoAcc->loadFile(path + file);
	} //for ( const auto& file : files )

	if ( StillNeedExploring )
	{
		std::strcpy(suffix, "exploration-files");
		const auto files = config->get_strings(buffer);
		for ( const auto& file : files )
		{
			ClingoAcc->loadFile(path + file);
		} //for ( const auto& file : files )
	} //if ( StillNeedExploring )

	const auto symbol(Clingo::Number(0));
	ClingoAcc->ground({Clingo::Part("base", Clingo::SymbolSpan())});
	ClingoAcc->ground({Clingo::Part("transition", Clingo::SymbolSpan(&symbol, 1))});

	//We have to unlock, to prevent a deadlock in newModel.
	locker.unlock();
	ClingoAcc->startSolvingBlocking();
	locker.relock();
	return;
}

/**
 * @brief Updates the navgraph distances.
 * @note Assumes, that ClingoAcc is locked.
 */
void
AspPlanerThread::updateNavgraphDistances(void)
{
	UpdateNavgraphDistances = false;
	for ( const auto& distance : NavgraphDistances )
	{
		ClingoAcc->assign_external(distance, Clingo::TruthValue::Free);
	} //for ( const auto& distance : NavgraphDistances )

	NavgraphDistances.clear();
	if ( StillNeedExploring )
	{
		NavgraphDistances.reserve(NavgraphNodesForASP.size() * NavgraphNodesForASP.size());
	} //if ( StillNeedExploring )
	else
	{
		//Release the memory!
		NavgraphDistances.shrink_to_fit();
	} //else -> if ( StillNeedExploring )

	MutexLocker locker(navgraph.objmutex_ptr());

	auto distanceToDuration = [this](const float distance) noexcept
		{
			//! @todo Werte holen!
			constexpr unsigned int constantCosts = 4;
			constexpr unsigned int costPerDistance = 2;
			return std::min(constantCosts + static_cast<unsigned int>(distance * costPerDistance),
				MaxDriveDuration);
		};

	const auto end = NavgraphNodesForASP.end();
	for ( auto from = NavgraphNodesForASP.begin(); from != end; ++from )
	{
		const auto& fromNode = navgraph->node(from->first);
		auto start = from;
		if ( !StillNeedExploring )
		{
			/* Increment the start when we are not in exploration. At least when there are no special nodes for the
			 * zones and the starting area we need also the distances between the node X and X.
			 *
			 * The example why we need this:
			 * Exploration phase started, the robot is located in zone(Z) (at least for Clingo), it gets the task to
			 * explore Z, but because there is no driveDuration(z(Z), z(Z), _) the doing will not be set and by that
			 * no end. Resulting in every robot has to do explore(Z) in step 0 without duration and begin another zone
			 * in step 1.
			 *
			 * In the production phase something like that will most likely not happen and we can reduce the facts for
			 * the solver and by that increase the performance.
			 */
			++start;
		} //if ( !StillNeedExploring )
		for ( auto to = start; to != end; ++to )
		{
			const auto& toNode = navgraph->node(to->first);

			/* If we use invalid nodes or don't find a path take a bug number for the duration.
			 * This is done because initally the nav graph is not connected and we wouldn't set durations. By
			 * that the ASP solver wouldn't assign tasks, because it could not calculate the estimated time
			 * for the tasks.
			 * The default value should be an upper bound on the driving duration a robot would take without
			 * mobile obstacles, i.e. drive from one end of the field to the other side, possibly around
			 * machines, but there is no replanning because of other robots.
			 */
			auto findDuration = [this,&toNode,&fromNode,distanceToDuration](void) {
					if ( fromNode.is_valid() && toNode.is_valid() )
					{
						const auto path = navgraph->search_path(fromNode, toNode);
						if ( !path.empty() )
						{
							return distanceToDuration(path.cost());
						} //if ( !path.empty() )
					} //if ( fromNode.is_valid() && toNode.is_valid() )
					return MaxDriveDuration;
				};

			const auto duration = realGameTimeToAspGameTime(findDuration());

			Clingo::Symbol arguments[3] = {from->second, to->second, Clingo::Number(duration)};

			std::function<void(void)> setDuration;
			if ( StillNeedExploring )
			{
				setDuration = [this,&arguments](void)
				{
					NavgraphDistances.emplace_back(Clingo::Function("driveDuration", {arguments, 3}));
					ClingoAcc->assign_external(NavgraphDistances.back(), Clingo::TruthValue::True);
					return;
				};
			} //if ( StillNeedExploring )
			else
			{
				setDuration = [this,&arguments](void)
				{
					Clingo::SymbolVector vec(sizeof(arguments) / sizeof(arguments[0]));
					std::copy(std::begin(arguments), std::end(arguments), vec.begin());
					queueGround(GroundRequest{"setDriveDuration", vec});
					return;
				};
			} //else -> if ( StillNeedExploring )

			setDuration();

			std::swap(arguments[0], arguments[1]);
			setDuration();
		} //for ( auto to = start; to != end; ++to )
	} //for ( auto from = NavgraphDistances.begin(); from != end; ++from )
	return;
}

/**
 * @brief Sets the past for the program, i.e. releases externals we know their status and sets if a robot doesn't begin
 *        or end a task.
 * @param[out] requests Stores ground requests in this vector.
 * @note Assumes ClingoAcc as locked.
 */
void
AspPlanerThread::setPast(std::vector<GroundRequest>& requests)
{
	//How many seconds of the past we still keep open to receive messages from the robots.
	constexpr decltype(GameTime) variablePast = 8;

	if ( GameTime <= variablePast )
	{
		//There is nothing we could ever fix.
		return;
	} //if ( GameTime <= variablePast )

	const auto fixUpTo = realGameTimeToAspGameTime(GameTime - variablePast);
	if ( fixUpTo <= Past )
	{
		//Nothing new to fix.
		return;
	} //if ( fixUpTo <= Past )

	static const auto robotVector([this](void)
		{
			std::vector<Clingo::Symbol> ret;
			ret.reserve(Robots.size());
			for ( const auto& robot : Robots )
			{
				ret.emplace_back(Clingo::String(robot.c_str()));
			} //for ( const auto& robot : Robots )
			return ret;
		}());
	requests.reserve(Robots.size() * (fixUpTo - Past));

	MutexLocker locker(&RobotsMutex);
	for ( auto t = Past; t <= fixUpTo; ++t )
	{
		const auto number = Clingo::Number(t);
		requests.emplace_back(GroundRequest{"past", Clingo::SymbolVector{number}, false});

		for ( const auto& robot : robotVector )
		{
			if ( !RobotTaskBegin.count({robot, t}) && !RobotTaskUpdate.count({robot, t}) )
			{
				requests.emplace_back(GroundRequest{"past", Clingo::SymbolVector{robot, number}, false});
			} //if ( !RobotTaskBegin.count({robot, t}) && !RobotTaskUpdate.count({robot, t}) )
		} //for ( const auto& robot : robotVector )
	} //for ( auto t = Past; t <= fixUpTo; ++t )
	locker.unlock();

	Past = fixUpTo;
	return;
}

/**
 * @brief Releases all externals associated by this information.
 * @param[in] info The information.
 * @param[in] lock If the ASP solver object should be locked. Set to false, if you have locked it before calling this
 *                 method.
 */
void
AspPlanerThread::releaseExternals(RobotInformation& info, const bool lock)
{
	MutexLocker locker(ClingoAcc.objmutex_ptr(), lock);
	return;
}

/**
 * @brief Transforms the real time to ASP time steps.
 * @param[in] realGameTime The real game time in seconds.
 * @return The ASP time units.
 */
unsigned int
AspPlanerThread::realGameTimeToAspGameTime(const unsigned int realGameTime) const noexcept
{
	return realGameTime / TimeResolution + ((realGameTime % TimeResolution) * 2 >= TimeResolution ? 1 : 0);
}

/**
 * @brief Transforms the ASP time steps to real time.
 * @param[in] aspGameTime The ASP time units.
 * @return The real game time in seconds.
 */
unsigned int
AspPlanerThread::aspGameTimeToRealGameTime(const unsigned int aspGameTime) const noexcept
{
	return aspGameTime * TimeResolution;
}

/**
 * @brief Queues a request for grounding.
 * @param[in, out] request The request, will me moved.
 * @param[in] interrupt Which interrupt level this request has.
 */
void
AspPlanerThread::queueGround(GroundRequest&& request, const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	if ( request.AddTick )
	{
		request.Params.push_back(Clingo::Number(LastTick++));
	} //if ( request.AddTick )
	Requests.emplace_back(std::move(request));

	if ( static_cast<unsigned short>(interrupt) > static_cast<unsigned short>(Interrupt) )
	{
		Interrupt = interrupt;
	} //if ( static_cast<unsigned short>(interrupt) > static_cast<unsigned short>(Interrupt) )
	return;
}

/**
 * @brief Is called, when the solver has found a new model.
 * @return If the solver should search for additional models.
 */
bool
AspPlanerThread::newModel(void)
{
	MutexLocker aspLocker(ClingoAcc.objmutex_ptr());
	MutexLocker locker(&SymbolMutex);
	Symbols = ClingoAcc->modelSymbols();
	NewSymbols = true;
	LastModel = clock->now();
	return MoreModels;
}

/**
 * @brief Is called, when the solver is finished. Either because the search is exhausted, the program is infeasable or
 *        we told him to be finished.
 * @param[in] result Contains the information what condition has been met.
 */
void
AspPlanerThread::solvingFinished(const Clingo::SolveResult& result)
{
	if ( result.is_unsatisfiable() )
	{
		Unsat = true;
	} //if ( result.is_unsatisfiable() )
	return;
}

/**
 * @brief Used to implement external functions in ASP, e.g. drive duration based on the nav graph.
 * @param[in] loc The location from where the function is called.
 * @param[in] name The function name.
 * @param[in] arguments The ASP arguments for the function.
 * @param[in] retFunction The function used to return the calculated value.
 */
void
AspPlanerThread::groundFunctions(const Clingo::Location& loc, const char *name, const Clingo::SymbolSpan& arguments,
	Clingo::SymbolSpanCallback& retFunction)
{
	if ( ClingoAcc->DebugLevel >= fawkes::ClingoAccess::All )
	{
		std::stringstream functionCall;
		functionCall<<name<<'(';
		auto sep = "";
		for ( const auto& argument : arguments )
		{
			functionCall<<sep<<argument;
			sep = ", ";
		} //for ( const auto& argument : arguments )
		functionCall<<')';

		const auto functionCallStr(functionCall.str());
		logger->log_warn(LoggingComponent, "Called %s.", functionCallStr.c_str());
	} //if ( ClingoAcc->DebugLevel >= fawkes::ClingoAccess::All )

	string_view view(name);

	switch ( arguments.size() )
	{
		case 0 :
		{
			if ( view == "explorationTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/explore-zone");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "explorationTaskDuration" )
			else if ( view == "getTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/fetch-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "getTaskDuration" )
			else if ( view == "prepareCSTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/fetch-from-shelf");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "prepareCSTaskDuration" )
			else if ( view == "mountCapTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/deliver-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "mountCapTaskDuration" )
			else if ( view == "mountRingTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/deliver-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "mountRingTaskDuration" )
			else if ( view == "feedRSTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/deliver-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "feedRSTaskDuration" )
			else if ( view == "deliverTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/deliver-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "deliverTaskDuration" )
			else if ( view == "getProductTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/fetch-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "getProductTaskDuration" )
			else if ( view == "maxFeedRS" )
			{
				static const unsigned int max = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "planer/max-feed-rs");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(max)});
				return;
			} //if ( view == "maxFeedRS" )
			else if ( view == "maxGetBase" )
			{
				static const unsigned int max = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "planer/max-get-base");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(max)});
				return;
			} //if ( view == "maxGetBase" )
			else if ( view == "maxGetProduct" )
			{
				static const unsigned int max = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "planer/max-get-product");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(max)});
				return;
			} //if ( view == "maxGetProduct" )
			else if ( view == "maxPrepCS" )
			{
				static const unsigned int max = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "planer/max-prep-cs");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(max)});
				return;
			} //if ( view == "maxPrepCS" )
			else if ( view == "maxDriveDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxDriveDuration))});
				return;
			} //else if ( view == "maxDriveDuration" )
			else if ( view == "robots" )
			{
				static const auto robots = [this](void) {
						Clingo::SymbolVector ret;
						ret.reserve(Robots.size());

						for ( const auto& robot : Robots )
						{
							ret.emplace_back(Clingo::String(robot.c_str()));
						} //for ( const auto& robot : Robots )
						return ret;
					}();
				retFunction({robots.data(), robots.size()});
				return;
			} //else if ( view == "robots" )
			else if ( view == "maxTicks" )
			{
				static const unsigned int ticks = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 20];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "planer/max-ticks");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(ticks)});
				return;
			} //else if ( view == "maxTicks" )
			else if ( view == "maxTaskDuration" )
			{
				static const unsigned int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/max-task-duration");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //else if ( view == "maxTaskDuration" )
			else if ( view == "maxOrders" )
			{
				retFunction({Clingo::Number(MaxOrders)});
				return;
			} //else if ( view == "maxOrders" )
			else if ( view == "maxQuantity" )
			{
				retFunction({Clingo::Number(MaxQuantity)});
				return;
			} //else if ( view == "maxQuantity" )
			else if ( view == "minDeliveryTime" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(ExplorationTime))});
				return;
			} //else if ( view == "minDeliveryTime" )
			else if ( view == "maxDeliveryTime" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime((4 + 15) * 60u))});
				return;
			} //else if ( view == "maxDeliveryTime" )
			break;
		} //case 0
		case 1 :
		{
			if ( view == "capColor" )
			{
				static const std::string colorOne = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						//We assume the distribution is the same, for CYAN and MAGENTA.
						std::strcpy(buffer + std::strlen(ConfigPrefix), "cap-station/assigned-color/C-CS1");
						return config->get_string(buffer);
					}();
				//The second CS has to have the other color, don't bother the config.
				static const std::string colorTwo = colorOne == "BLACK" ? "GREY" : "BLACK";
				static const std::string* colors[] = {&colorOne, &colorTwo};
				const auto index = arguments[0].number();
				assert(index >= 1 && index <= 2);
				retFunction({Clingo::String(*colors[index - 1])});
				return;
			} //if ( view == "capColor" )
			break;
		} //case 1
	} //switch ( arguments.size() )

	std::stringstream functionCall;
	functionCall<<name<<'(';
	auto sep = "";
	for ( const auto& argument : arguments )
	{
		functionCall<<sep<<argument;
		sep = ", ";
	} //for ( const auto& argument : arguments )
	functionCall<<')';

	const auto functionCallStr(functionCall.str());
	throw fawkes::Exception("Called function %s from %s:%d-%d, but there exists no definition of %s/%d!",
		functionCallStr.c_str(), loc.begin_file(), loc.begin_line(), loc.begin_column(), name, arguments.size());
	return;
}

/**
 * @brief Sets the team for the solver.
 */
void
AspPlanerThread::setTeam(void)
{
	Clingo::SymbolVector param(1, Clingo::String(TeamColor));
	queueGround({"ourTeam", param, false}, InterruptSolving::Critical);

	fillNavgraphNodesForASP();
	graph_changed();
	UpdateNavgraphDistances = true;
	return;
}

void
AspPlanerThread::unsetTeam(void)
{
	//TODO: I think it is clear what to do.
	throw fawkes::Exception("Should unset the team, but this is not implemented!");
	return;
}

/**
 * @brief Adds a new robot to the ASP program.
 * @param[in] mate The name of the new robot.
 * @param[in] info The information to the robot.
 */
void
AspPlanerThread::newTeamMate(const std::string& mate, const RobotInformation& info)
{
	MutexLocker navgraphLocker(navgraph.objmutex_ptr());
	auto node = navgraph->closest_node(info.X, info.Y, false, NodePropertyASP);
	if ( !node.is_valid() )
	{
		throw fawkes::Exception("No ASP-NavGraph-Node for robot %s found, can not meaningful be added to the program!",
			mate.c_str());
	} //if ( !node.is_valid() )

	Clingo::SymbolVector params;
	params.emplace_back(Clingo::String(mate.c_str()));
	params.emplace_back(Clingo::Number(realGameTimeToAspGameTime(info.GameTime)));
	queueGround({"addRobot", params, true}, InterruptSolving::Critical);

	params.emplace(params.begin() + 1, NavgraphNodesForASP.find(node.name())->second);
	queueGround({"setRobotLocation", params, true});
	return;
}

/**
 * @brief Removes a robot of the ASP program.
 * @param[in] mate The name of the robot.
 */
void
AspPlanerThread::deadTeamMate(const std::string& mate)
{
	Clingo::SymbolVector params;
	params.emplace_back(Clingo::String(mate.c_str()));
	params.emplace_back(Clingo::Number(realGameTimeToAspGameTime(GameTime)));
	queueGround({"removeRobot", params, true}, InterruptSolving::Critical);
	return;
}

/**
 * @brief Adds a zone to explore.
 * @param[in] zone The zone number.
 */
void AspPlanerThread::addZoneToExplore(const long zone)
{
	assert(zone >= 1 && zone <= 24);
	Clingo::SymbolVector params;
	params.reserve(2);
	params.emplace_back(Clingo::Number(zone));
	params.emplace_back(Clingo::Number(realGameTimeToAspGameTime(GameTime)));
	queueGround({"zoneToExplore", params, false}, InterruptSolving::JustStarted);
	return;
}

/**
 * @brief Adds the ring info to asp.
 * @param[in] info The info.
 */
void
AspPlanerThread::setRingColor(const RingColorInformation& info)
{
	Clingo::SymbolVector params = {Clingo::String(info.Color), Clingo::Number(info.Cost), Clingo::String(info.Machine)};
	queueGround({"setRingInfo", params, false}, InterruptSolving::JustStarted);
	return;
}

/**
 * @brief Adds an order to asp.
 * @param[in] order The order.
 */
void
AspPlanerThread::addOrder(const OrderInformation& order)
{
	Clingo::SymbolVector params = {Clingo::Number(order.Number), Clingo::Number(order.Quantity),
		Clingo::String(order.Base), Clingo::String(order.Cap), Clingo::String(order.Rings[0]),
		Clingo::String(order.Rings[1]), Clingo::String(order.Rings[2]),
		Clingo::Number(realGameTimeToAspGameTime(order.DeliveryBegin)),
		Clingo::Number(realGameTimeToAspGameTime(order.DeliveryEnd)),
		Clingo::Number(realGameTimeToAspGameTime(order.GameTime))};
	queueGround({"newOrder", params, false}, InterruptSolving::Critical);
	return;
}

/**
 * @brief Called if a machine is found.
 */
void
AspPlanerThread::foundAMachine(void)
{
	if ( ++MachinesFound == 6 )
	{
		ClingoAcc.lock();
		StillNeedExploring = false;
		CompleteRestart = true;
		ClingoAcc.unlock();
	} //if ( ++MachinesFound == 6 )
	return;
}

/**
 * @brief Helper function to decompose a string and transform it to a Clingo::Function.
 * @param[in] string The task string.
 */
static Clingo::Symbol
taskStringToFunction(const std::string& string)
{
	const auto paramsBegin = string.find('(');
	const auto task(string.substr(0, paramsBegin));
	string_view params(string);
	params.remove_prefix(paramsBegin + 1);
	params.remove_suffix(1);

	Clingo::SymbolVector arguments;
	arguments.reserve(std::count(params.begin(), params.end(), ','));

	auto pos = params.find(','), lastPos = string_view::npos;
	static_assert(string_view::npos + 1 == 0);

	do
	{ //while ( lastPos != string_view::npos )
		const auto param = params.substr(lastPos + 1, pos - (lastPos + 1)).to_string();
		if ( param.find('(') == std::string::npos )
		{
			if ( std::isdigit(param[0]) )
			{
				char *unused;
				arguments.emplace_back(Clingo::Number(std::strtol(param.c_str(), &unused, 10)));
			} //if ( std::isdigit(param[0]) )
			else
			{
				arguments.emplace_back(Clingo::Id(param.c_str()));
			} //else -> if ( std::isdigit(param[0]) )
		} //if ( param.find('(') == std::string::npos )
		else
		{
			//The param is a function itself!
			throw "Not implemented yet";
		} //else -> if ( param.find('(') == std::string::npos )
		pos = params.find(',', lastPos = pos);
	} while ( lastPos != string_view::npos );

	return Clingo::Function(task.c_str(), arguments);
}

/**
 * @brief A robot has begun with a task, add it to the program.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] time At which point in time the task was begun.
 */
void
AspPlanerThread::robotBegunWithTask(const std::string& robot, const std::string& task, unsigned int time)
{
	time = realGameTimeToAspGameTime(time);
	GroundRequest request{"begun", {Clingo::String(robot), taskStringToFunction(task),
		Clingo::Number(time)}, false};
	MutexLocker locker(&RobotsMutex);
	RobotTaskBegin.insert({{Clingo::String(robot), time}, request});
	queueGround(std::move(request), InterruptSolving::Critical);
	return;
}

/**
 * @brief A robot updates the time estimation for a task, add it to the program.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] time At which point in time the update was emitted.
 * @param[in] end The new estimated end time.
 */
void
AspPlanerThread::robotUpdatesTaskTimeEstimation(const std::string& robot, const std::string& task,
		unsigned int time, unsigned int end)
{
	const auto duration = std::max(1u, realGameTimeToAspGameTime(end - time));
	time = realGameTimeToAspGameTime(time);
	end = realGameTimeToAspGameTime(end);
	GroundRequest request{"update", {Clingo::String(robot), taskStringToFunction(task),
		Clingo::Number(time), Clingo::Number(duration)}, false};
	MutexLocker locker(&RobotsMutex);
	RobotTaskUpdate.insert({{Clingo::String(robot), time}, request});
	queueGround(std::move(request), InterruptSolving::Critical);
	return;
}

/**
 * @brief A robot has finished a task, add it to the program.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] time At which point in time the task was finished.
 */
void
AspPlanerThread::robotFinishedTask(const std::string& robot, const std::string& task, unsigned int time)
{
	time = realGameTimeToAspGameTime(time);
	GroundRequest request{"update", {Clingo::String(robot), taskStringToFunction(task),
		Clingo::Number(time), Clingo::Number(0)}, false};
	MutexLocker locker(&RobotsMutex);
	RobotTaskUpdate.insert({{Clingo::String(robot), time}, request});
	queueGround(std::move(request), InterruptSolving::Critical);
	return;
}

/**
 * @brief A task was not successfully executed.
 * @param[in] task The task.
 * @param[in] time At which point in time the task was finished.
 */
void
AspPlanerThread::taskWasFailure(const std::string& task, unsigned int time)
{
	time = realGameTimeToAspGameTime(time);
	GroundRequest request{"failure", {taskStringToFunction(task), Clingo::Number(time)}, false};
	MutexLocker locker(&RobotsMutex);
	TaskSuccess.insert({time, request});
	queueGround(std::move(request), InterruptSolving::Critical);
	return;
}
