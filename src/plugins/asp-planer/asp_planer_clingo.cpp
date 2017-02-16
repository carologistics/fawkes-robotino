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
#include <thread>

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
	//Locked: ClingoAcc, RequestMutex

	if ( !ProgramGrounded )
	{
		return;
	} //if ( !ProgramGrounded )

	const auto requests = GroundRequests.size() + ReleaseRequests.size() + AssignRequests.size();
	if ( ClingoAcc->solving() )
	{
		if ( shouldInterrupt() && !SentCancel )
		{
			logger->log_warn(LoggingComponent, "Cancel solving, new requests: %d", requests);
			ClingoAcc->cancelSolving();
			SentCancel = true;
		} //if ( shouldInterrupt() && !SentCancel )
		return;
	} //if ( ClingoAcc->solving() )

	MutexLocker navgraphLocker(&NavgraphDistanceMutex);
	//Locked: ClingoAcc, RequestMutex, NavgraphDistanceMutex
	if ( UpdateNavgraphDistances )
	{
		const bool lastUpdate = NodesToFind.empty();
		if ( lastUpdate )
		{
			//Unlock for the release requests in releaseZone().
			reqLocker.unlock();
			MutexLocker worldLocker(&WorldMutex);
			//Locked: ClingoAcc, NavgraphDistanceMutex, WorldMutex
			if ( ReceivedZonesToExplore )
			{
				for ( const auto& zone : ZonesToExplore )
				{
					releaseZone(zone, false);
				} //for ( const auto& zone : ZonesToExplore )
				ZonesToExplore.clear();
			} //if ( ReceivedZonesToExplore )
			else
			{
				for ( auto zone = 1; zone <= 24; ++zone )
				{
					releaseZone(zone, false);
				} //for ( auto zone = 1; zone <= 24; ++zone )
			} //else -> if ( ReceivedZonesToExplore )
			navgraphLocker.unlock();
			//Locked: ClingoAcc, WorldMutex
			fillNavgraphNodesForASP(false);
			navgraphLocker.relock();
			reqLocker.relock();
			//Locked: ClingoAcc, WorldMutex, NavgraphDistanceMutex, RequestMutex
		} //if ( lastUpdate )
		//Locked: ClingoAcc, NavgraphDistanceMutex, RequestMutex

		for ( const auto& external : NavgraphDistances )
		{
			ClingoAcc->assign_external(external, false);
		} //for ( const auto& external : NavgraphDistances )

		updateNavgraphDistances();

		if ( lastUpdate )
		{
			logger->log_info(LoggingComponent, "All machines found, fix distances and release externals.");
			reqLocker.unlock();
			//Locked: ClingoAcc, NavgraphDistanceMutex
			for ( const auto& external : NavgraphDistances )
			{
				queueGround({"setDriveDuration",
					Clingo::SymbolVector(external.arguments().begin(), external.arguments().end())});
				Clingo::Symbol args[3];
				std::copy(external.arguments().begin(), external.arguments().end(), std::begin(args));

				//In difference to assigning and grounding we need the full symmetry in releasing.
				for ( bool swap = true; true; )
				{
					for ( auto d = 1; d <= realGameTimeToAspGameTime(MaxDriveDuration); ++d )
					{
						args[2] = Clingo::Number(d);
						queueRelease(Clingo::Function(external.name(), {args, 3}));
					} //for ( auto d = 1; d <= realGameTimeToAspGameTime(MaxDriveDuration); ++d )

					if ( swap )
					{
						std::swap(args[0], args[1]);
						swap = false;
					} //if ( swap )
					else
					{
						break;
					} //else -> if ( swap )
				} //for ( bool swap = true; true; )
			} //for ( const auto& external : NavgraphDistances )
			//This has to be done, because the behavior of double unlocking is undefined.
			reqLocker.relock();
			//Locked: ClingoAcc, NavgraphDistanceMutex, RequestMutex
		} //if ( lastUpdate )
		else
		{
			for ( const auto& external : NavgraphDistances )
			{
				ClingoAcc->assign_external(external, true);
			} //for ( const auto& external : NavgraphDistances )
		} //else -> if ( lastUpdate )
	} //if ( UpdateNavgraphDistances )
	else if ( requests == 0 && Interrupt == InterruptSolving::Not )
	{
		MutexLocker solvingLocker(&SolvingMutex);
		//Locked: ClingoAcc, RequestMutex, NavgraphDistanceMutex, SolvingMutex
		if ( Clock::now() - SolvingStarted < std::chrono::seconds(5) )
		{
			//Nothing to do and last solving happened within the last five seconds.
			return;
		} //if ( Clock::now() - SolvingStarted < std::chrono::seconds(5) )
	} //else if ( requests == 0 && Interrupt == InterruptSolving::Not )
	navgraphLocker.unlock();
	reqLocker.unlock();
	//Locked: ClingoAcc

	SentCancel = false;
	Interrupt = InterruptSolving::Not;

	static std::vector<Clingo::Symbol> externals;

	for ( const auto& external : externals )
	{
		ClingoAcc->assign_external(external, false);
	} //for ( const auto& external : externals )

	externals.clear();

	//Set "initial" state.
	MutexLocker worldLocker(&WorldMutex);
	//Locked: ClingoAcc, WorldMutex
	auto addExternal = [this](Clingo::Symbol&& external)
		{
			ClingoAcc->assign_external(external, true);
			externals.push_back(std::move(external));
			return;
		};

	auto locations(LocationInUse);
	locations.reserve(Robots.size());

	for ( const auto& pair : Robots )
	{
		const auto& name(pair.first);
		const auto& robot(pair.second);
		ClingoAcc->assign_external(robot.AliveExternal, robot.Alive);

		if ( !robot.Alive )
		{
			continue;
		} //if ( !robot.Alive )

		if ( robot.Holding.isValid() )
		{
			addExternal(generateHoldingExternal(name, robot.Holding));
		} //if ( robot.Holding.isValid() )

		if ( robot.Doing.isValid() )
		{
			addExternal(generateDoingExternal(name, robot.Doing,
				realGameTimeToAspGameTime(std::max(1, robot.Doing.EstimatedEnd - GameTime))));
		} //if ( robot.Doing.isValid() )
		else
		{
			auto location = nearestLocation(robot.X, robot.Y);
			const auto locationRobot = locations.find(location);
			if ( locationRobot != locations.end() && locationRobot->second != name )
			{
				/* Two (or more) robots "on" one locations, should only happen at game start. The robots are mapped to
				 * a side of the near base station instead of ins-out. Change their location to ins-out, this is the
				 * only location where multiple robots are allowed. */
				location = Clingo::String("ins-out");
			} //if ( locationRobot != locations.end() && locationRobot->second != name )
			else
			{
				locations.insert({location, name});
			} //else -> if ( locationRobot != locations.end() && locationRobot->second != name )
			addExternal(generateRobotLocationExternal(name, location));
		} //else -> if ( robot.Doing.isValid() )
	} //for ( const auto& pair : Robots )

	for ( const auto& pair : Machines )
	{
		const auto& name(pair.first);
		const auto& machine(pair.second);

		auto working = machine.WorkingUntil;
		if ( machine.BrokenUntil )
		{
			addExternal(generateMachineBrokenExternal(name,
				std::max(1, realGameTimeToAspGameTime(machine.BrokenUntil - GameTime))));
			if ( working )
			{
				working = machine.BrokenUntil + working - GameTime;
			} //if ( working )
		} //if ( machine.BrokenUntil )

		if ( working )
		{
			assert(machine.Storing.isValid());
			addExternal(generateMachineWorkingExternal(name,
				std::max(1, realGameTimeToAspGameTime(working - GameTime)), machine.Storing));
		} //if ( working )
		else if ( machine.Storing.isValid() )
		{
			addExternal(generateMachineStoringExternal(name, machine.Storing));
		} //else if ( machine.Storing.isValid() )
	} //for ( const auto& pair : Machines )

	for ( auto index = 0; index < static_cast<int>(Products.size()); ++index )
	{
		const auto& product(Products[index]);
		const ProductIdentifier id{index};
		addExternal(generateProductExternal(id));
		addExternal(generateProductBaseExternal(id, product.Base));

		for ( auto ring = 1; ring <= 3; ++ring )
		{
			if ( product.Rings[ring].empty() )
			{
				break;
			} //if ( product.Rings[ring].empty() )
			addExternal(generateProductRingExternal(id, ring, product.Rings[ring]));
		} //for ( auto ring = 1; ring <= 3; ++ring )

		if ( !product.Cap.empty() )
		{
			addExternal(generateProductCapExternal(id, product.Cap));
		} //if ( !product.Cap.empty() )
	} //for ( auto index = 0; index < static_cast<int>(Products.size()); ++index )
	worldLocker.unlock();
	//Locked: ClingoAcc

	reqLocker.relock();
	//Locked: ClingoAcc, RequestMutex
	//Copy the requests to release the lock especially before grounding!
	auto groundRequests(std::move(GroundRequests));
	auto releaseRequests(std::move(ReleaseRequests));
	auto assignRequests(std::move(AssignRequests));
	reqLocker.unlock();
	//Locked: ClingoAcc

	if ( !groundRequests.empty() )
	{
		std::vector<Clingo::Part> parts;
		parts.reserve(groundRequests.size());
		for ( const auto& request : groundRequests )
		{
			parts.emplace_back(request.first, request.second);
		} //for ( const auto& request : groundRequests )
		ClingoAcc->ground(parts);
	} //if ( !groundRequests.empty() )

	for ( const auto& atom : releaseRequests )
	{
		ClingoAcc->release_external(atom);
	} //for ( const auto& atom : releaseRequests )

	for ( const auto& atom : assignRequests )
	{
		ClingoAcc->assign_external(atom, true);
	} //for ( const auto& atom : assignRequests )

	MutexLocker solvingLokcer(&SolvingMutex);
	worldLocker.relock();
	//Locked: ClingoAcc, SolvingMutex, WorldMutex

	auto currentTimeExternal = [](const int time)
		{
			return Clingo::Function("currentTime", {Clingo::Number(time)});
		};

	if ( GameTime >= ExplorationTime && !ProductionStarted )
	{
		ProductionStarted = true;
		ClingoAcc->ground({{"startProduction", {}}});
		ClingoAcc->release_external(Clingo::Id("productionStarted"));
	} //if ( GameTime >= ExplorationTime && !ProductionStarted )

	const auto aspGameTime = realGameTimeToAspGameTime(GameTime);
	for ( auto gt = realGameTimeToAspGameTime(StartSolvingGameTime); gt < aspGameTime; ++gt )
	{
		ClingoAcc->release_external(currentTimeExternal(gt));
	} //for ( auto gt = realGameTimeToAspGameTime(StartSolvingGameTime); gt < aspGameTime; ++gt )
	StartSolvingGameTime = GameTime;
	worldLocker.unlock();
	//Locked: ClingoAcc, SolvingMutex
	ClingoAcc->assign_external(currentTimeExternal(aspGameTime), true);
	SolvingStarted = Clock::now();
	ClingoAcc->startSolving();
	return;
}

/**
 * @brief Queues a ground request.
 * @param[in] request The request to build a Clingo::Part to ground.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlanerThread::queueGround(GroundRequest&& request, const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	GroundRequests.push_back(std::move(request));
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
	ReleaseRequests.push_back(std::move(atom));
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
	AssignRequests.push_back(std::move(atom));
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
	if ( static_cast<short>(interrupt) > static_cast<short>(Interrupt) )
	{
		Interrupt = interrupt;
	} //if ( static_cast<short>(interrupt) > static_cast<short>(Interrupt) )
	return;
}

/**
 * @brief Says if the solving process should be interrupted.
 * @return If the solving should be interrupted.
 * @note RequestMutex has to be locked.
 */
bool
AspPlanerThread::shouldInterrupt(void)
{
	const auto now(Clock::now());
	switch ( Interrupt )
	{
		case InterruptSolving::Not :
		{
			static auto lastCheck(now);
			static const std::chrono::seconds threshold(
				config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/robot-task-check"));

			if ( lastCheck - now >= threshold )
			{
				lastCheck = now;
				MutexLocker worldLocker(&WorldMutex);
				MutexLocker planLocker(&PlanMutex);

				for ( const auto& pair : Robots )
				{
					if ( pair.second.Alive )
					{
						const auto& robotPlan(Plan[pair.first]);
						if ( !robotPlan.CurrentTask.empty() )
						{
							static const std::chrono::seconds threshold(
								config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/robot-task-behind"));
							if ( GameTime > robotPlan.Tasks[robotPlan.FirstNotDone].End + threshold.count() )
							{
								logger->log_warn(LoggingComponent, "Robot %s is more than %d seconds behind schedule "
									"(and has not send an update), increase interrupt level.", pair.first.c_str(),
									threshold.count());
								Interrupt = InterruptSolving::High;
							} //if ( GameTime > robotPlan.Tasks[robotPlan.FirstNotDone].End + threshold.count() )
						} //if ( !robotPlan.CurrentTask.empty() )
					} //if ( pair.second.Alive )
				} //for ( const auto& pair : Robots )
			} //if ( lastCheck - now >= threshold )

			//Check down here, so we have released the locks!
			if ( Interrupt != InterruptSolving::Not )
			{
				return shouldInterrupt();
			} //if ( Interrupt != InterruptSolving::Not )
			break;
		} //case InterruptSolving::Not
		case InterruptSolving::JustStarted :
		{
			static const std::chrono::seconds threshold(
				config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/just-started"));
			MutexLocker locker(&SolvingMutex);
			const auto diff(now - SolvingStarted);
			return diff <= threshold;
		} //case InterruptSolving::JustStarted
		case InterruptSolving::Normal :
		{
			static const std::chrono::seconds threshold(
				config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/normal"));
			MutexLocker locker(&PlanMutex);
			const auto diff(now - LastPlan);
			return diff <= threshold;
		} //case InterruptSolving::Normal
		case InterruptSolving::High :
		{
			static const std::chrono::seconds threshold(
				config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/high"));
			MutexLocker locker(&PlanMutex);
			const auto diff(now - LastPlan);
			return diff <= threshold;
		} //case InterruptSolving::High
		case InterruptSolving::Critical : return true;
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
	LastModel = Clock::now();
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
		++Unsat;
		logger->log_error(LoggingComponent, "The input is infeasiable! #%d", Unsat);
	} //if ( result.is_unsatisfiable() )
	else
	{
		Unsat = 0;
	} //else -> if ( result.is_unsatisfiable() )
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
				static const int dur = [this](void)
					{
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/explore-zone");
						return config->get_int(buffer);
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
			else if ( view == "maxDriveDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxDriveDuration))});
				return;
			} //else if ( view == "maxDriveDuration" )
			else if ( view == "maxProducts" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxProducts))});
				return;
			} //else if ( view == "maxProducts" )
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
			else if ( view == "maxWorkingDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxWorkingDuration))});
				return;
			} //else if ( view == "maxWorkingDuration" )
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
			else if ( view == "clingoToASP" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(arguments[0].number()))});
				return;
			} //else if ( view == "clingoToASP" )
			else if ( view == "machineWorkingDuration" )
			{
				const std::string machineName(arguments[0].string(), 2);
				retFunction({Clingo::Number(realGameTimeToAspGameTime(WorkingDurations[machineName]))});
				return;
			} //else if ( view == "machineWorkingDuration" )
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
	ProgramGrounded = true;

	fillNavgraphNodesForASP(true);
	NodesToFind.clear();
	NodesToFind.reserve(6);
	for ( const auto& machine : {"BS", "CS1", "CS2", "DS", "RS1", "RS2"} )
	{
		NodesToFind.insert(std::string(TeamColor) + "-" + machine + "-I");
	} //for ( const auto& machine : {"BS", "CS1", "CS2", "DS", "RS1", "RS2"} )
	DeliveryLocation = generateLocationExternal(TeamColor, "DS", "I");
	CapLocations[0]  = generateLocationExternal(TeamColor, "CS1", "I");
	CapLocations[1]  = generateLocationExternal(TeamColor, "CS2", "I");
	RingLocations[0] = generateLocationExternal(TeamColor, "RS1", "I");
	RingLocations[1] = generateLocationExternal(TeamColor, "RS2", "I");
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
 * @note Assumes the world mutex is locked!
 */
void
AspPlanerThread::addOrderToASP(const OrderInformation& order)
{
	Clingo::SymbolVector params = {Clingo::Number(order.Number), Clingo::Number(order.Quantity),
		Clingo::String(order.Base), Clingo::String(order.Cap), Clingo::String(order.Rings[1]),
		Clingo::String(order.Rings[2]), Clingo::String(order.Rings[3]),
		Clingo::Number(realGameTimeToAspGameTime(order.DeliveryBegin)),
		Clingo::Number(realGameTimeToAspGameTime(order.DeliveryEnd))};
	queueGround({"newOrder", std::move(params)}, InterruptSolving::Critical);

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

	for ( auto qty = order.Quantity + 1; qty <= MaxQuantity; ++qty )
	{
		for ( auto ring = 1; ring <= 3; ++ring )
		{
			for ( const auto& location : RingLocations )
			{
				queueRelease(generateMountRingExternal(location, order.Number, qty, ring));
			} //for ( const auto& location : RingLocations )
		} //for ( auto ring = 1; ring <= 3; ++ring )
		for ( const auto& location : CapLocations )
		{
			queueRelease(generateMountCapExternal(location, order.Number, qty));
		} //for ( const auto& location : CapLocations )
		queueRelease(generateDeliverExternal(DeliveryLocation, order.Number, qty));
		queueRelease(generateLateDeliverExternal(DeliveryLocation, order.Number, qty));
	} //for ( auto qty = order.Quantity + 1; qty <= MaxQuantity; ++qty )

	for ( auto qty = 1; qty <= order.Quantity; ++qty )
	{
		const auto capIndex = std::find_if(CapColors.begin(), CapColors.end(),
			[&order](const CapColorInformation& info) noexcept
			{
				return info.Color == order.Cap;
			})->Machine[2] - '1';

		OrderTasks tasks{Clingo::Symbol(), Clingo::Symbol(), Clingo::Symbol(), Clingo::Symbol(),
			generateMountCapExternal(CapLocations[capIndex], order.Number, qty),
			generateDeliverExternal(DeliveryLocation, order.Number, qty),
			generateLateDeliverExternal(DeliveryLocation, order.Number, qty)};

		//Make an explicit copy because queueAssign takes an rvalue.
		queueAssign(Clingo::Symbol(tasks.CapTask));
		queueAssign(Clingo::Symbol(tasks.DeliverTasks[0]));
		queueAssign(Clingo::Symbol(tasks.DeliverTasks[1]));
		queueRelease(generateMountCapExternal(CapLocations[1 - capIndex], order.Number, qty));

		auto ring = 1;
		for ( ; ring <= 3; ++ring )
		{
			const auto& color(order.Rings[ring]);
			if ( color == "none" )
			{
				break;
			} //if ( color == "none" )

			const auto ringIndex = std::find_if(RingColors.begin(), RingColors.end(),
				[&order,&color](const RingColorInformation& info) noexcept
				{
					return info.Color == color;
				})->Machine[2] - '1';

			tasks.RingTasks[ring] = generateMountRingExternal(RingLocations[ringIndex], order.Number, qty, ring);
			queueAssign(Clingo::Symbol(tasks.RingTasks[ring]));
			queueRelease(generateMountRingExternal(RingLocations[1 - ringIndex], order.Number, qty, ring));
		} //for ( auto ring = 1; ring <= 3; ++ring )

		for ( ; ring <= 3; ++ring )
		{
			for ( const auto& location : RingLocations )
			{
				queueRelease(generateMountRingExternal(location, order.Number, qty, ring));
			} //for ( const auto& location : RingLocations )
		} //for ( ; ring <= 3; ++ring )

		OrderTaskMap.insert({{order.Number, qty}, std::move(tasks)});
	} //for ( auto qty = 1; qty <= order.Quantity; ++qty )
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
AspPlanerThread::addZoneToExplore(const int zone)
{
	assert(zone >= 1 && zone <= 24);
	queueAssign(generateExploreLocationExternal(zone));
	queueAssign(generateExploreTaskExternal(zone));
	return;
}

/**
 * @brief Releases the externals for a zone exploration.
 * @param[in] zone The zone number.
 * @param[in] removeAndFillNodes If @link ZonesToExplore the vector @endlink should be cleaned and the
 *                               fillNavgraphNodesForASP() should be called.
 */
void
AspPlanerThread::releaseZone(const int zone, const bool removeAndFillNodes)
{
	assert(zone >= 1 && zone <= 24);
	if ( removeAndFillNodes )
	{
		MutexLocker locker(&WorldMutex);
		ZonesToExplore.erase(std::find(ZonesToExplore.begin(), ZonesToExplore.end(), zone));
		fillNavgraphNodesForASP(false);
	} //if ( removeAndFillNodes )
	queueRelease(generateExploreLocationExternal(zone));
	queueRelease(generateExploreTaskExternal(zone));
	return;
}

static inline Clingo::SymbolVector
splitParameters(string_view string)
{
	std::vector<Clingo::Symbol> ret;
	ret.reserve(std::count(string.begin(), string.end(), ','));

	auto comma = string.find(',');
	auto paranthesis = string.find('(');
	while ( comma != string_view::npos || paranthesis != string_view::npos )
	{
		if ( comma != string_view::npos && comma < paranthesis )
		{
			auto param = string.substr(0, comma);

			if ( std::isdigit(param[0]) )
			{
				char *unused;
				ret.push_back(Clingo::Number(std::strtol(param.data(), &unused, 10)));
			} //if ( std::isdigit(param[0]) )
			else
			{
				assert(param[0] == '"');
				ret.push_back(Clingo::String(std::string(param.substr(1, param.size() - 2))));
			} //else -> if ( std::isdigit(param[0]) )
			string.remove_prefix(comma + 1);
		} //if ( comma != string_view::npos && comma < paranthesis )
		else if ( paranthesis != string_view::npos )
		{
			auto index = paranthesis + 1;
			for ( auto count = 1; count != 0; ++index )
			{
				if ( string[index] == ')' )
				{
					--count;
				} //if ( string[index] == ')' )
				else if ( string[index] == '(' )
				{
					++count;
				} //else if ( string[index] == '(' )
			} //for ( auto count = 1u, index = paranthesis + 1; count != 0; ++index )

			const auto substrStart(paranthesis + 1), substrEnd(index - 1 - substrStart);
			ret.push_back(Clingo::Function(std::string(string.substr(0, paranthesis)).c_str(),
				splitParameters(string.substr(substrStart, substrEnd))));

			/* +1 for the following comma, but when there is no comma, i.e. this function was the last parameter bound
			 * the removed size by the strings size. Removing more is undefined. */
			string.remove_prefix(std::min(index + 1, string.size()));
		} //else if ( paranthesis != string_view::npos )
		comma = string.find(',');
		paranthesis = string.find('(');
	} //while ( comma != string_view::npos || paranthesis != string_view::npos )

	if ( string.size() != 0 )
	{
		if ( std::isdigit(string[0]) )
		{
			char *unused;
			ret.push_back(Clingo::Number(std::strtol(string.data(), &unused, 10)));
		} //if ( std::isdigit(string[0]) )
		else
		{
			assert(string[0] == '"');
			ret.push_back(Clingo::String(std::string(string.substr(1, string.size() - 2))));
		} //else -> if ( std::isdigit(string[0]) )
	} //if ( string.size() != 0 )

	return ret;
}

/**
 * @brief Creates a task description.
 * @param[in] task The task as string.
 * @param[in] end The estimated end for the task.
 * @return The new description.
 */
static inline TaskDescription
createTaskDescription(const std::string& task, const int end)
{
	auto type = TaskDescription::None;

	string_view taskView(task);
	const auto paramsBegin = taskView.find('(');
	string_view taskName(taskView.substr(0, paramsBegin));
	string_view params(taskView.substr(paramsBegin + 1));
	//Remove the ) from the parameters.
	params.remove_suffix(1);

	if ( taskName == "deliver" || taskName == "lateDeliver" )
	{
		type = TaskDescription::Deliver;
	} //if ( taskName == "deliver" || taskName == "lateDeliver" )
	else if ( taskName == "feedRS" )
	{
		type = TaskDescription::FeedRS;
	} //else if ( taskName == "feedRS" )
	else if ( taskName == "getBase" )
	{
		type = TaskDescription::GetBase;
	} //else if ( taskName == "getBase" )
	else if ( taskName == "getProduct" )
	{
		type = TaskDescription::GetProduct;
	} //else if ( taskName == "getProduct" )
	else if ( taskName == "goto" )
	{
		type = TaskDescription::Goto;
	} //else if ( taskName == "goto" )
	else if ( taskName == "mountCap" )
	{
		type = TaskDescription::MountCap;
	} //else if ( taskName == "mountCap" )
	else if ( taskName == "mountRing" )
	{
		type = TaskDescription::MountRing;
	} //else if ( taskName == "mountRing" )
	else if ( taskName == "prepareCS" )
	{
		type = TaskDescription::PrepareCS;
	} //else if ( taskName == "prepareCS" )
	else
	{
		throw fawkes::Exception("Unknown task name: %s!", taskName.to_string().c_str());
	} //else -> all tasks

	return TaskDescription{type, Clingo::Function(taskName.to_string().c_str(), splitParameters(params)), end};
}

/**
 * @brief Sets the interrupt flag, depending on the offset a task feedback has.
 * @param[in] offset The offset.
 */
void
AspPlanerThread::checkForInterruptBasedOnTimeOffset(int offset)
{
	logger->log_info(LoggingComponent, "Plan-Feedback offset is %d.", offset);
	offset = std::abs(offset);
	if ( offset >= 3*TimeResolution )
	{
		setInterrupt(InterruptSolving::Critical);
	} //if ( offset >= 3*TimeResolution )
	else if ( offset >= 2*TimeResolution )
	{
		setInterrupt(InterruptSolving::High);
	} //else if ( offset >= 2*TimeResolution )
	else if ( offset >= (15 * TimeResolution) / 10 )
	{
		setInterrupt(InterruptSolving::Normal);
	} //else if ( offset >= (15 * TimeResolution) / 10 )
	else if ( offset >= TimeResolution )
	{
		setInterrupt(InterruptSolving::JustStarted);
	} //else if ( offset >= TimeResolution )
	return;
}

/**
 * @brief A robot has begun with a task, modify the worldstate.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] begin At which point in time the task was begun.
 * @param[in] end At which point in time the task will end.
 */
void
AspPlanerThread::robotBegunWithTask(const std::string& robot, const std::string& task, const int begin, const int end)
{
	MutexLocker worldLocker(&WorldMutex);
	MutexLocker planLocker(&PlanMutex);

	auto& robotPlan(Plan[robot]);
	auto& robotInfo(Robots[robot]);

	assert(robotPlan.CurrentTask.empty());
	assert(!robotInfo.Doing.isValid());

	if ( robotPlan.Tasks[robotPlan.FirstNotDone].Task != task )
	{
		logger->log_info(LoggingComponent, "Plan invalid, the robot started another task.");
		robotPlan.Tasks[robotPlan.FirstNotDone].Task = task;
		robotPlan.Tasks[robotPlan.FirstNotDone].Begin = begin;
		robotPlan.Tasks[robotPlan.FirstNotDone].End = end;
		setInterrupt(InterruptSolving::Critical);
	} //if ( robotPlan.Tasks[robotPlan.FirstNotDone].Task != task )

	robotPlan.CurrentTask = task;
	robotPlan.Tasks[robotPlan.FirstNotDone].Begun = true;
	const auto offset = begin - robotPlan.Tasks[robotPlan.FirstNotDone].Begin;
	static_assert(std::is_signed<decltype(offset)>::value, "Offset has to have a sign!");
	planLocker.unlock();
	worldLocker.unlock();
	checkForInterruptBasedOnTimeOffset(offset);
	worldLocker.relock();
	planLocker.relock();

	robotInfo.Doing = createTaskDescription(task, robotPlan.Tasks[robotPlan.FirstNotDone].End);

	const auto location = robotInfo.Doing.TaskSymbol.arguments().front();
	const auto iter = LocationInUse.find(location);

	if ( iter != LocationInUse.end() && iter->second != robot )
	{
		logger->log_warn(LoggingComponent,
			"The robots %s and %s are trying to use %s! Tell %s to stop immediately and delete its current plan!",
			iter->second.c_str(), robot.c_str(), location.to_string().c_str(), robot.c_str());
		robotPlan.CurrentTask.clear();
		robotPlan.Tasks[robotPlan.FirstNotDone].Begun = false;
		robotInfo.Doing = {};
		tellRobotToStop(robot);
		setInterrupt(InterruptSolving::Critical);
		for ( auto index = robotPlan.FirstNotDone; index < robotPlan.Tasks.size(); ++index ) {
			removeFromPlanDB(robot, index);
		} //for ( auto index = robotPlan.FirstNotDone; index < robotPlan.Tasks.size(); ++index )
		robotPlan.Tasks.erase(robotPlan.Tasks.begin() + robotPlan.FirstNotDone, robotPlan.Tasks.end());
	} //if ( iter != LocationInUse.end() && iter->second != robot )
	else
	{
		LocationInUse.insert({location, robot});
	} //else -> if ( iter != LocationInUse.end() && iter->second != robot )
	return;
}

/**
 * @brief A robot updates the time estimation for a task, add it to the program.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] end The new estimated end time.
 */
void
AspPlanerThread::robotUpdatesTaskTimeEstimation(const std::string& robot, const std::string& task, const int end)
{
	MutexLocker worldLocker(&WorldMutex);
	MutexLocker planLocker(&PlanMutex);

	auto& robotPlan(Plan[robot]);
	auto& robotInfo(Robots[robot]);

	assert(robotPlan.CurrentTask == task);
	assert(robotPlan.Tasks[robotPlan.FirstNotDone].Task == task);
	assert(robotInfo.Doing.isValid());

	const auto offset = end - robotPlan.Tasks[robotPlan.FirstNotDone].End;
	static_assert(std::is_signed<decltype(offset)>::value, "Offset has to have a sign!");
	/* Do NOT update the end value, because we calculate the offset based on that. If we receive multiple small offsets
	 * that do not trigger replaning this does not work as intended.
	 * robotPlan.Tasks[robotPlan.FirstNotDone].End = end; */
	robotInfo.Doing.EstimatedEnd = end;
	planLocker.unlock();
	worldLocker.unlock();
	checkForInterruptBasedOnTimeOffset(offset);
	return;
}

/**
 * @brief A robot has finished a task, add it to the program.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] end At which point in time the task was finished.
 * @param[in] success If the task was executed succesful.
 */
void
AspPlanerThread::robotFinishedTask(const std::string& robot, const std::string& task, const int end, const bool success)
{
	MutexLocker worldLocker(&WorldMutex);
	MutexLocker planLocker(&PlanMutex);

	auto& robotPlan(Plan[robot]);
	auto& robotInfo(Robots[robot]);

	assert(robotPlan.CurrentTask == task);
	assert(robotPlan.Tasks[robotPlan.FirstNotDone].Task == task);
	assert(robotInfo.Doing.isValid());

	const auto offset = end - robotPlan.Tasks[robotPlan.FirstNotDone].End;
	static_assert(std::is_signed<decltype(offset)>::value, "Offset has to have a sign!");
	robotPlan.Tasks[robotPlan.FirstNotDone].End = end;
	planLocker.unlock();
	worldLocker.unlock();
	checkForInterruptBasedOnTimeOffset(offset);
	worldLocker.relock();
	planLocker.relock();

	robotPlan.Tasks[robotPlan.FirstNotDone++].Done = true;
	robotPlan.CurrentTask.clear();

	const auto location = robotInfo.Doing.TaskSymbol.arguments().front();
	assert(LocationInUse[location] == robot);
	LocationInUse.erase(location);

	if ( !success )
	{
		robotInfo.Doing = {};
		return;
	} //if ( !success )

	auto generateProduct = [this](std::string&& baseColor) -> ProductIdentifier
		{
			if ( static_cast<int>(Products.size()) >= MaxProducts )
			{
				 logger->log_error(LoggingComponent, "Have to generate a product, this would be #%d alive, but only %d "
					"are configured. This product will not be part of the ASP program until products with a lower id "
					"will be destroyed!", Products.size() + 1, MaxProducts);
			} //if ( static_cast<int>(Products.size()) >= MaxProducts )

			Products.push_back({baseColor});
			return {static_cast<decltype(ProductIdentifier::ID)>(Products.size())};
		};
	auto destroyProduct = [this](const ProductIdentifier& id)
		{
			Products.erase(Products.begin() + id.ID);
			for ( auto& pair : Robots )
			{
				if ( pair.second.Holding.ID > id.ID )
				{
					--pair.second.Holding.ID;
				} //if ( pair.second.Holding.ID > id.ID )
			} //for ( auto& pair : Robots )
			for ( auto& pair : Machines )
			{
				if ( pair.second.Storing.ID > id.ID )
				{
					--pair.second.Storing.ID;
				} //if ( pair.second.Storing.ID > id.ID )
			} //for ( auto& pair : Machines )
			return;
		};

	auto machinePickup = [this](const std::string& machine, const ProductIdentifier& id)
		{
			auto& machineInfo(Machines[machine]);
			assert(!machineInfo.Storing.isValid());
			machineInfo.Storing = id;
			machineInfo.WorkingUntil = GameTime + WorkingDurations[machine];
			return;
		};

	auto machineDrops = [this](const std::string& machine)
		{
			auto& machineInfo(Machines[machine]);
			assert(machineInfo.Storing.isValid());
			auto id = machineInfo.Storing;
			machineInfo.Storing = {};
			return id;
		};

	auto robotPickups = [&robotInfo](const ProductIdentifier& id)
		{
			assert(!robotInfo.Holding.isValid());
			robotInfo.Holding = id;
			return;
		};
	auto robotDrops = [&robotInfo](void)
		{
			assert(robotInfo.Holding.isValid());
			auto id = robotInfo.Holding;
			robotInfo.Holding = {};
			return id;
		};

	const decltype(auto) taskArguments(robotInfo.Doing.TaskSymbol.arguments());
	const decltype(auto) machine(taskArguments[0].arguments()[1].string());

	auto getOrder = [&taskArguments](void)
		{
			return std::make_pair<int, int>(taskArguments[1].number(), taskArguments[2].number());
		};

	switch ( robotInfo.Doing.Type )
	{
		case TaskDescription::None : break; //Does not happen. (See assert above.)
		case TaskDescription::Deliver :
		{
			auto order(getOrder());
			queueRelease(std::move(OrderTaskMap[order].DeliverTasks[0]));
			queueRelease(std::move(OrderTaskMap[order].DeliverTasks[1]));
			destroyProduct(robotDrops());
			break;
		} //case TaskDescription::Deliver
		case TaskDescription::FeedRS :
		{
			const auto& product(Products[robotInfo.Holding.ID]);
			if ( !product.Rings[1].empty() || !product.Cap.empty() )
			{
				logger->log_warn(LoggingComponent, "%s used a non trivial product to feed a ring station!",
					robot.c_str());
			} //if ( !product.Rings[1].empty() || !product.Cap.empty() )
			destroyProduct(robotDrops());
			auto& info(Machines[machine]);
			assert(info.FillState <= 3);
			++info.FillState;
			break;
		} //case TaskDescription::FeedRS
		case TaskDescription::GetBase : robotPickups(generateProduct(taskArguments[1].string())); break;
		case TaskDescription::GetProduct : robotPickups(machineDrops(machine)); break;
		case TaskDescription::Goto : break; //The robots location will be fetched from the navgraph.
		case TaskDescription::MountCap :
		{
			auto& machineInfo(Machines[machine]);
			assert(machineInfo.Prepared);
			auto order(getOrder());
			auto product = robotDrops();
			assert(Products[product.ID].Cap.empty());
			Products[product.ID].Cap = Orders[order.first].Cap;
			queueRelease(std::move(OrderTaskMap[order].CapTask));
			machinePickup(machine, product);
			machineInfo.Prepared = false;
			break;
		} //case TaskDescription::MountCap
		case TaskDescription::MountRing :
		{
			auto order(getOrder());
			const auto ringNumber = taskArguments[3].number();
			auto product = robotDrops();
			assert(Products[product.ID].Rings[ringNumber].empty());
			const auto ringColor = Orders[order.first].Rings[ringNumber];
			auto& machineInfo(Machines[machine]);
			const auto ringInfo = std::find_if(RingColors.begin(), RingColors.end(),
				[&ringColor](const RingColorInformation& info) { return info.Color == ringColor; });
			assert(machineInfo.FillState >= ringInfo->Cost);
			Products[product.ID].Rings[ringNumber] = Orders[order.first].Rings[ringNumber];
			queueRelease(std::move(OrderTaskMap[order].RingTasks[ringNumber]));
			machinePickup(machine, product);
			machineInfo.FillState -= ringInfo->Cost;
			break;
		} //case TaskDescription::MountRing
		case TaskDescription::PrepareCS :
		{
			auto& machineInfo(Machines[machine]);
			assert(!machineInfo.Prepared);
			machinePickup(machine, generateProduct("TRANSPARENT"));
			machineInfo.Prepared = true;
			break;
		} //case TaskDescription::PrepareCS
	} //switch ( robotInfo.Doing.Type )
	robotInfo.Doing = {};
	return;
}
