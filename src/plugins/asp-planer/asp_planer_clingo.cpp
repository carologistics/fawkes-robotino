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
#include "asp_planer_externals.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <sstream>
#include <experimental/string_view>
#include <thread>

//! @todo Replace include and using, once C++17 is implemented properly.
using std::experimental::string_view;

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <plugins/asp/aspect/clingo_access.h>

using fawkes::MutexLocker;

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
	const auto prefixLen = std::strlen(ConfigPrefix);
	constexpr auto infixLen = std::strlen(infix);
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

	ClingoAcc->ground({{"base", {}}});

//	CapLocations.reserve(2);
//	RingLocations.reserve(2);
//	BaseLocations.reserve(2);
//	GetLocations.reserve(4);
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in finalize().
 */
void
AspPlanerThread::finalizeClingo(void)
{
	MutexLocker locker(ClingoAcc.objmutex_ptr());
	if ( ClingoAcc->solving() )
	{
		ClingoAcc->cancelSolving();
		do //while ( ClingoAcc->solving() )
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(20ms);
		} while ( ClingoAcc->solving() );
	} //if ( ClingoAcc->solving() )
	return;
}

/**
 * @brief Handles the loop concerning the solving process.
 */
void
AspPlanerThread::loopClingo(void)
{
	MutexLocker aspLocker(ClingoAcc.objmutex_ptr());
	MutexLocker reqLocker(&RequestMutex);

	if ( TeamColor == nullptr )
	{
		return;
	} //if ( TeamColor == nullptr )

	const auto requests = GroundRequests.size() + ReleaseRequests.size() + AssignRequests.size();
	if ( ClingoAcc->solving() )
	{
		if ( requests > 0 && shouldInterrupt() && !SentCancel )
		{
			logger->log_warn(LoggingComponent, "Cancel solving, new requests: %d", requests);
			ClingoAcc->cancelSolving();
			SentCancel = true;
		} //if ( requests > 0 && shouldInterrupt() && !SentCancel )
		return;
	} //if ( ClingoAcc->solving() )

	MutexLocker navgraphLocker(&NavgraphDistanceMutex);
	if ( UpdateNavgraphDistances )
	{
		for ( const auto& external : NavgraphDistances )
		{
			ClingoAcc->free_exteral(external);
		} //for ( const auto& external : NavgraphDistances )
		updateNavgraphDistances();
		for ( const auto& external : NavgraphDistances )
		{
			ClingoAcc->assign_external(external, true);
		} //for ( const auto& external : NavgraphDistances )
	} //if ( UpdateNavgraphDistances )
	else if ( requests == 0 && Interrupt == InterruptSolving::Not )
	{
		//Nothing to do.
		return;
	} //else if ( requests == 0 && Interrupt == InterruptSolving::Not )
	navgraphLocker.unlock();
	reqLocker.unlock();

	SentCancel = false;
	Interrupt = InterruptSolving::Not;

	//Set "initial" state.
	MutexLocker worldLocker(&WorldMutex);
	auto checkAndRelease = [this](const Clingo::Symbol& external)
		{
			if ( isValidExternal(external) )
			{
				ClingoAcc->release_external(external);
			} //if ( isValidExternal(external) )
			return;
		};
	auto checkAndSet = [this](Clingo::Symbol& external)
		{
			if ( isValidExternal(external) )
			{
				ClingoAcc->assign_external(external, true);
				return true;
			} //if ( isValidExternal(external) )
			return false;
		};

	for ( auto& pair : Robots )
	{
		auto& name(pair.first);
		auto& robot(pair.second);
		ClingoAcc->assign_external(robot.AliveExternal, robot.Alive);

		checkAndRelease(robot.LocationExternal);
		checkAndRelease(robot.HoldingExternal);
		checkAndRelease(robot.DoingExternal);

		if ( !robot.Alive )
		{
			continue;
		} //if ( !robot.Alive )

		checkAndSet(robot.HoldingExternal = generateHoldingExternal(name, robot.Holding));
		if ( !checkAndSet(robot.DoingExternal = generateDoingExternal(name, robot.Doing)) )
		{
			checkAndSet(robot.LocationExternal = generateRobotLocationExternal(name, nearestLocation(robot.X, robot.Y)));
		} //if ( !checkAndSet(robot.DoingExternal = generateDoingExternal(name, robot.Doing)) )
	} //for ( auto& pair : Robots )
	worldLocker.unlock();

	reqLocker.relock();
	ClingoAcc->ground(GroundRequests);
	GroundRequests.clear();

	for ( const auto& atom : ReleaseRequests )
	{
		ClingoAcc->release_external(atom);
	} //for ( const auto& atom : ReleaseRequests )
	ReleaseRequests.clear();

	for ( const auto& atom : AssignRequests )
	{
		ClingoAcc->assign_external(atom, true);
	} //for ( const auto& atom : AssignRequests )
	AssignRequests.clear();
	reqLocker.unlock();

	MutexLocker planLocker(&PlanMutex);
	MutexLocker solvingLokcer(&SolvingMutex);
	worldLocker.relock();
	StartSolvingGameTime = GameTime;
	SolvingStarted = clock->now();
	ClingoAcc->startSolving();
	return;
}

/**
 * @brief Queues a ground request.
 * @param[in] part The program part to ground.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlanerThread::queueGround(Clingo::Part&& atom, const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	GroundRequests.push_back(atom);
	setInterrupt(interrupt, false);
	return;
}

/**
 * @brief Queues a release request.
 * @param[in] atom The atom to release.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlanerThread::queueRelease(Clingo::Symbol&& atom, const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	ReleaseRequests.push_back(atom);
	setInterrupt(interrupt, false);
	return;
}

/**
 * @brief Queues a assign request.
 * @param[in] atom The atom to release.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlanerThread::queueAssign(Clingo::Symbol&& atom, const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	AssignRequests.push_back(atom);
	setInterrupt(interrupt, false);
	return;
}

/**
 * @brief Sets the interrupt value.
 * @param[in] interrupt Which level of interrupt is requested.
 * @param[in] lock If we should lock, if set to false we expect it to be locked by the caller.
 */
void
AspPlanerThread::setInterrupt(const InterruptSolving interrupt, const bool lock)
{
	MutexLocker locker(&RequestMutex, lock);
	if ( static_cast<unsigned short>(interrupt) > static_cast<unsigned short>(Interrupt) )
	{
		Interrupt = interrupt;
	} //if ( static_cast<unsigned short>(interrupt) > static_cast<unsigned short>(Interrupt) )
	return;
}

/**
 * @brief Says if the solving process should be interrupted.
 * @return If the solving should be interrupted.
 * @note Requestmutex has to be locked.
 */
bool
AspPlanerThread::shouldInterrupt(void) const
{
	const auto now(clock->now());
	switch ( Interrupt )
	{
		case InterruptSolving::Not         : break;
		case InterruptSolving::JustStarted :
		{
			static const fawkes::Time threshold(
				config->get_uint(std::string(ConfigPrefix) + "interrupt-thresholds/just-started"), 0);
			MutexLocker locker(&SolvingMutex);
			const auto diff(now - SolvingStarted);
			return diff <= threshold;
		} //case InterruptSolving::JustStarted
		case InterruptSolving::Normal      :
		{
			static const fawkes::Time threshold(
				config->get_uint(std::string(ConfigPrefix) + "interrupt-thresholds/normal"), 0);
			MutexLocker locker(&PlanMutex);
			const auto diff(now - LastPlan);
			return diff <= threshold;
		} //case InterruptSolving::Normal
		case InterruptSolving::Critical    : return true;
	} //switch ( Interrupt )
	return false;
}

/**
 * @brief Is called, when the solver has found a new model.
 * @return If the solver should search for additional models.
 */
bool
AspPlanerThread::newModel(void)
{
	MutexLocker aspLocker(ClingoAcc.objmutex_ptr());
	MutexLocker locker(&SolvingMutex);
	Symbols = ClingoAcc->modelSymbols();
	NewSymbols = true;
	LastModel = clock->now();
	return true;
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
				retFunction({Clingo::Number(realGameTimeToAspGameTime(FetchProductTaskDuration))});
				return;
			} //if ( view == "getTaskDuration" )
			else if ( view == "prepareCSTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(PrepareCSTaskDuration))});
				return;
			} //if ( view == "prepareCSTaskDuration" )
			else if ( view == "mountCapTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
				return;
			} //if ( view == "mountCapTaskDuration" )
			else if ( view == "mountRingTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
				return;
			} //if ( view == "mountRingTaskDuration" )
			else if ( view == "feedRSTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
				return;
			} //if ( view == "feedRSTaskDuration" )
			else if ( view == "deliverTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
				return;
			} //if ( view == "deliverTaskDuration" )
			else if ( view == "getProductTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(FetchProductTaskDuration))});
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

						for ( const auto& robot : PossibleRobots )
						{
							ret.emplace_back(Clingo::String(robot.c_str()));
						} //for ( const auto& robot : PossibleRobots )
						return ret;
					}();
				retFunction({robots.data(), robots.size()});
				return;
			} //else if ( view == "robots" )
			else if ( view == "horizon" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(LookAhaed))});
				return;
			} //else if ( view == "maxTicks" )
			else if ( view == "maxTaskDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxTaskDuration))});
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
				retFunction({Clingo::Number(realGameTimeToAspGameTime(ProductionEnd))});
				return;
			} //else if ( view == "maxDeliveryTime" )
			break;
		} //case 0
		case 1 :
		{
			if ( view == "capColor" )
			{
				const auto index = arguments[0].number();
				assert(index >= 1 && index <= 2);
				retFunction({Clingo::String(CapColors[index - 1].Color)});
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
	MutexLocker aspLocker(ClingoAcc.objmutex_ptr());
	ClingoAcc->ground({{"ourTeam", {Clingo::String(TeamColor)}}});

	fillNavgraphNodesForASP();
	graph_changed();
	UpdateNavgraphDistances = true;
	return;
}

/**
 * @brief Unsets the team color, we have to restart all over again.
 */
void
AspPlanerThread::unsetTeam(void)
{
	//TODO: I think it is clear what to do.
	throw fawkes::Exception("Should unset the team, but this is not implemented!");
	return;
}

/**
 * @brief Adds an order to asp.
 * @param[in] order The order.
 */
void
AspPlanerThread::addOrderToASP(const OrderInformation& order)
{
	Clingo::SymbolVector params = {Clingo::Number(order.Number), Clingo::Number(order.Quantity),
		Clingo::String(order.Base), Clingo::String(order.Cap), Clingo::String(order.Rings[0]),
		Clingo::String(order.Rings[1]), Clingo::String(order.Rings[2]),
		Clingo::Number(realGameTimeToAspGameTime(order.DeliveryBegin)),
		Clingo::Number(realGameTimeToAspGameTime(order.DeliveryEnd))};
	queueGround({"newOrder", params}, InterruptSolving::Critical);

	for ( const auto& color : BaseColors )
	{
		queueRelease(Clingo::Function("base", {Clingo::Number(order.Number), Clingo::String(color)}));
	} //for ( const auto& color : BaseColors )

	for ( const auto& color : CapColors )
	{
		queueRelease(Clingo::Function("cap", {Clingo::Number(order.Number), Clingo::String(color.Color)}));
	} //for ( const auto& color : CapColors )

	for ( const auto& color : RingColors )
	{
		for ( auto ring = 1; ring <= 3; ++ring )
		{
			queueRelease(Clingo::Function("ring",
				{Clingo::Number(order.Number), Clingo::Number(ring), Clingo::String(color.Color)}));
		} //for ( auto ring = 1; ring <= 3; ++ring )
	} //for ( const auto& color : RingColors )

	for ( decltype(MaxQuantity) qty = 1; qty <= MaxQuantity; ++qty )
	{
	} //for ( decltype(MaxQuantity) qty = 1; qty <= MaxQuantity; ++qty )
	return;
}

/**
 * @brief Adds the ring info to asp.
 * @param[in] info The info.
 */
void
AspPlanerThread::addRingColorToASP(const RingColorInformation& info)
{
	const auto color(Clingo::String(info.Color));
	queueGround({"setRingInfo", {color, Clingo::Number(info.Cost), Clingo::String(info.Machine)}});

	for ( const auto& machine : {"RS1", "RS2"} )
	{
		queueRelease(Clingo::Function("ringStationAssignment", {Clingo::String(machine), color}));
	} //for ( const auto& machine : {"RS1", "RS2"} )

	for ( auto cost = 0; cost <= 2; ++cost )
	{
		queueRelease(Clingo::Function("ringColorCost", {color, Clingo::Number(cost)}));
	} //for ( auto cost = 0; cost <= 2; ++cost )
	return;
}

/**
 * @brief Adds a zone to explore.
 * @param[in] zone The zone number.
 */
void
AspPlanerThread::addZoneToExplore(const unsigned int zone)
{
	assert(zone >= 1 && zone <= 24);
	queueAssign(generateExploreLocationExternal(zone));
	queueAssign(generateExploreTaskExternal(zone));
	return;
}

/**
 * @brief Releases the externals for a zone exploration.
 * @param[in] zone The zone number.
 */
void
AspPlanerThread::releaseZone(const unsigned int zone)
{
	assert(zone >= 1 && zone <= 24);
	queueRelease(generateExploreLocationExternal(zone));
	queueRelease(generateExploreTaskExternal(zone));
	return;
}

/**
 * @brief Called if a machine is found.
 *
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
 *
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
 *
void
AspPlanerThread::robotBegunWithTask(const std::string& robot, const std::string& task, unsigned int time)
{
	time = realGameTimeToAspGameTime(time);
	GroundRequest request{"begun", {Clingo::String(robot), taskStringToFunction(task),
		Clingo::Number(time)}};
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
 *
void
AspPlanerThread::robotUpdatesTaskTimeEstimation(const std::string& robot, const std::string& task,
		unsigned int time, unsigned int end)
{
	const auto duration = std::max(1u, realGameTimeToAspGameTime(end - time));
	time = realGameTimeToAspGameTime(time);
	end = realGameTimeToAspGameTime(end);
	GroundRequest request{"update", {Clingo::String(robot), taskStringToFunction(task),
		Clingo::Number(time), Clingo::Number(duration)}};
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
 *
void
AspPlanerThread::robotFinishedTask(const std::string& robot, const std::string& task, unsigned int time)
{
	time = realGameTimeToAspGameTime(time);
	GroundRequest request{"update", {Clingo::String(robot), taskStringToFunction(task),
		Clingo::Number(time), Clingo::Number(0)}, std::string()};
	MutexLocker locker(&RobotsMutex);
	RobotTaskUpdate.insert({{Clingo::String(robot), time}, request});
	queueGround(std::move(request), InterruptSolving::Critical);
	return;
}

/**
 * @brief A task was not successfully executed.
 * @param[in] task The task.
 * @param[in] time At which point in time the task was finished.
 *
void
AspPlanerThread::taskWasFailure(const std::string& task, unsigned int time)
{
	time = realGameTimeToAspGameTime(time);
	GroundRequest request{"failure", {taskStringToFunction(task), Clingo::Number(time)}};
	MutexLocker locker(&RobotsMutex);
	TaskSuccess.insert({time, request});
	queueGround(std::move(request), InterruptSolving::Critical);
	return;
}
*/
