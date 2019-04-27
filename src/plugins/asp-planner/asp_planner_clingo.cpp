/***************************************************************************
 *  asp_planner_clingo.cpp - ASP-based planner plugin clingo interface
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

#include "asp_planner_externals.h"
#include "asp_planner_thread.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <plugins/asp/aspect/clingo_access.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>

using fawkes::MutexLocker;

/**
 * @brief Takes care of everything regarding clingo interface in init().
 */
void
AspPlannerThread::initClingo(void)
{
	MutexLocker navgraphLocker(navgraph.objmutex_ptr());
	navgraph->add_change_listener(this);
	navgraphLocker.unlock();

	MutexLocker locker(clingo.objmutex_ptr());
	clingo->register_model_callback(
	  std::make_shared<std::function<bool(void)>>([this](void) { return newModel(); }));
	clingo->register_finish_callback(std::make_shared<std::function<void(Clingo::SolveResult)>>(
	  [this](const Clingo::SolveResult result) {
		  solvingFinished(result);
		  return;
	  }));
	clingo->set_ground_callback([this](auto... args) {
		this->groundFunctions(args...);
		return;
	});

	constexpr auto infix     = "planner/";
	const auto     prefixLen = std::strlen(ConfigPrefix);
	constexpr auto infixLen  = std::strlen(infix);
	char           buffer[prefixLen + infixLen + 20];
	const auto     suffix = buffer + prefixLen + infixLen;
	std::strcpy(buffer, ConfigPrefix);
	std::strcpy(buffer + prefixLen, infix);

	std::strcpy(suffix, "program-path");
	const auto path = config->get_string(buffer);
	std::strcpy(suffix, "program-files");
	const auto files = config->get_strings(buffer);

	std::strcpy(suffix, "threads");
	const auto threads = config->get_int(buffer);
	std::strcpy(suffix, "use-splitting");
	const auto splitting = config->get_bool(buffer);
	clingo->set_num_threads(threads, splitting);

	logger->log_info(LoggingComponent,
	                 "Loading program files from %s. Debug state: %d",
	                 path.c_str(),
	                 clingo->debug_level());
	for (const auto &file : files) {
		clingo->load_file(path + file);
	} // for ( const auto& file : files )

	clingo->ground({{"base", {}}});

	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in finalize().
 */
void
AspPlannerThread::finalizeClingo(void)
{
	MutexLocker locker(clingo.objmutex_ptr());
	if (clingo->solving()) {
		clingo->cancel_solving();
		do // while ( clingo->solving() )
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(20ms);
		} while (clingo->solving());
	} // if ( clingo->solving() )
	return;
}

/**
 * @brief Handles the loop concerning the solving process.
 */
void
AspPlannerThread::loopClingo(void)
{
	MutexLocker aspLocker(clingo.objmutex_ptr());
	// Locked: clingo

	if (!ProgramGrounded) {
		return;
	} // if ( !ProgramGrounded )

	MutexLocker reqLocker(&RequestMutex);
	const auto  requests = GroundRequests.size() + ReleaseRequests.size() + AssignRequests.size();
	reqLocker.unlock();

	if (clingo->solving()) {
		if (shouldInterrupt() && !SentCancel) {
			logger->log_warn(LoggingComponent,
			                 "Cancel solving, new requests: %zu, interrupt level: %s, reason: %s",
			                 requests,
			                 interruptString(Interrupt.load()),
			                 InterruptReason.load());
			clingo->cancel_solving();
			SentCancel = true;
		} // if ( shouldInterrupt() && !SentCancel )
		return;
	} // if ( clingo->solving() )

	MutexLocker navgraphLocker(&NavgraphDistanceMutex);
	// Locked: clingo, NavgraphDistanceMutex
	if (UpdateNavgraphDistances) {
		const bool lastUpdate = NodesToFind.empty();
		if (lastUpdate) {
			MutexLocker worldLocker(&WorldMutex);
			// Locked: clingo, NavgraphDistanceMutex, WorldMutex
			if (ReceivedZonesToExplore) {
				for (const auto &zone : ZonesToExplore) {
					releaseZone(zone, false);
				} // for ( const auto& zone : ZonesToExplore )
				ZonesToExplore.clear();
			} // if ( ReceivedZonesToExplore )
			else {
				for (auto zone = 1; zone <= 24; ++zone) {
					releaseZone(zone, false);
				} // for ( auto zone = 1; zone <= 24; ++zone )
			}   // else -> if ( ReceivedZonesToExplore )
			navgraphLocker.unlock();
			// Locked: clingo, WorldMutex
			fillNavgraphNodesForASP(false);
			navgraphLocker.relock();
			// Locked: clingo, WorldMutex, NavgraphDistanceMutex
		} // if ( lastUpdate )
		// Locked: clingo, NavgraphDistanceMutex

		for (const auto &external : NavgraphDistances) {
			clingo->assign_external(external, false);
		} // for ( const auto& external : NavgraphDistances )

		updateNavgraphDistances();

		if (lastUpdate) {
			logger->log_info(LoggingComponent,
			                 "All machines found, fix distances and release externals.");
			for (const auto &external : NavgraphDistances) {
				queueGround({"setDriveDuration",
				             Clingo::SymbolVector(external.arguments().begin(),
				                                  external.arguments().end())},
				            "");
				Clingo::Symbol args[3];
				std::copy(external.arguments().begin(), external.arguments().end(), std::begin(args));

				// In difference to assigning and grounding we need the full symmetry in
				// releasing.
				for (bool swap = true; true;) {
					for (auto d = 1; d <= realGameTimeToAspGameTime(MaxDriveDuration); ++d) {
						args[2] = Clingo::Number(d);
						queueRelease(Clingo::Function(external.name(), {args, 3}), "");
					} // for ( auto d = 1; d <=
					// realGameTimeToAspGameTime(MaxDriveDuration); ++d )

					if (swap) {
						std::swap(args[0], args[1]);
						swap = false;
					} // if ( swap )
					else {
						break;
					} // else -> if ( swap )
				}   // for ( bool swap = true; true; )
			}     // for ( const auto& external : NavgraphDistances )
		}       // if ( lastUpdate )
		else {
			for (const auto &external : NavgraphDistances) {
				clingo->assign_external(external, true);
			} // for ( const auto& external : NavgraphDistances )
		}   // else -> if ( lastUpdate )
	}     // if ( UpdateNavgraphDistances )
	else if (requests == 0 && Interrupt == InterruptSolving::Not) {
		MutexLocker solvingLocker(&SolvingMutex);
		// Locked: clingo, NavgraphDistanceMutex, SolvingMutex
		if (Clock::now() - SolvingStarted < std::chrono::seconds(15)) {
			// Nothing to do and last solving happened within the last fifteen
			// seconds.
			return;
		} // if ( Clock::now() - SolvingStarted < std::chrono::seconds(15) )
		MutexLocker worldLocker(&WorldMutex);
		// Locked: clingo, NavgraphDistanceMutex, SolvingMutex, WorldMutex
		if (StartSolvingGameTime == GameTime) {
			// Nothing to do and the time hasn't advanced. (Game is paused or still in
			// setup.)
			return;
		} // if ( StartSolvingGameTime == GameTime )
	}   // else if ( requests == 0 && Interrupt == InterruptSolving::Not )
	navgraphLocker.unlock();
	// Locked: clingo

	SentCancel      = false;
	Interrupt       = InterruptSolving::Not;
	InterruptReason = "";

	static std::vector<Clingo::Symbol> externals;

	for (const auto &external : externals) {
		clingo->assign_external(external, false);
	} // for ( const auto& external : externals )

	externals.clear();

	// Set "initial" state.
	MutexLocker worldLocker(&WorldMutex);

	if (GameTime == -1) {
		// The game has ended, we do not need any more planning.
		return;
	} // if ( GameTime == -1 )

	// Locked: clingo, WorldMutex
	auto addExternal = [this](Clingo::Symbol &&external) {
		clingo->assign_external(external, true);
		externals.push_back(std::move(external));
		return;
	};

	auto locations(LocationInUse);
	locations.reserve(Robots.size());

	for (const auto &pair : Robots) {
		const auto &name(pair.first);
		const auto &robot(pair.second);
		clingo->assign_external(robot.AliveExternal, robot.Alive);

		if (!robot.Alive) {
			continue;
		} // if ( !robot.Alive )

		if (robot.Holding.isValid()) {
			addExternal(generateHoldingExternal(name, robot.Holding));
		} // if ( robot.Holding.isValid() )

		if (robot.Doing.isValid()) {
			addExternal(generateDoingExternal(name,
			                                  robot.Doing,
			                                  realGameTimeToAspGameTime(
			                                    std::max(1, robot.Doing.EstimatedEnd - GameTime))));
		} // if ( robot.Doing.isValid() )
		else {
			auto       location      = nearestLocation(robot.X, robot.Y);
			const auto locationRobot = locations.find(location);
			if (locationRobot != locations.end() && locationRobot->second != name) {
				/* Two (or more) robots "on" one locations, should only happen at game
         * start. The robots are mapped to a side of the near base station
         * instead of ins-out. Change their location to ins-out, this is the
         * only location where multiple robots are allowed. */
				location = Clingo::String("ins-out");
			} // if ( locationRobot != locations.end() && locationRobot->second !=
			// name )
			else {
				locations.insert({location, name});
			} // else -> if ( locationRobot != locations.end() &&
			// locationRobot->second != name )
			addExternal(generateRobotLocationExternal(name, location));
		} // else -> if ( robot.Doing.isValid() )
	}   // for ( const auto& pair : Robots )

	for (const auto &pair : Machines) {
		const auto &name(pair.first);
		const auto &machine(pair.second);

		auto working = machine.WorkingUntil;
		if (machine.BrokenUntil) {
			addExternal(generateMachineBrokenExternal(
			  name, std::max(1, realGameTimeToAspGameTime(machine.BrokenUntil - GameTime))));
			if (working) {
				working = machine.BrokenUntil + working - GameTime;
			} // if ( working )
		}   // if ( machine.BrokenUntil )

		if (working) {
			assert(machine.Storing.isValid());
			addExternal(generateMachineWorkingExternal(
			  name, std::max(1, realGameTimeToAspGameTime(working - GameTime)), machine.Storing));
		} // if ( working )
		else if (machine.Storing.isValid()) {
			addExternal(generateMachineStoringExternal(name, machine.Storing));
		} // else if ( machine.Storing.isValid() )
	}   // for ( const auto& pair : Machines )

	for (const auto &machine : {"CS1", "CS2"}) {
		const auto &info(Machines[machine]);
		if (info.Prepared) {
			addExternal(generatePreparedExternal(machine));
		}
	} // for ( const auto& machine : {"CS1", "CS2"} )

	for (const auto &machine : {"RS1", "RS2"}) {
		const auto &info(Machines[machine]);
		addExternal(generateFillStateExternal(machine, info.FillState));
	} // for ( const auto& machine : {"RS1", "RS2"} )

	for (auto index = 0; index < static_cast<int>(Products.size()); ++index) {
		const auto &            product(Products[index]);
		const ProductIdentifier id{index};
		addExternal(generateProductExternal(id));
		addExternal(generateProductBaseExternal(id, product.Base));

		for (auto ring = 1; ring <= 3; ++ring) {
			if (product.Rings[ring].empty()) {
				break;
			} // if ( product.Rings[ring].empty() )
			addExternal(generateProductRingExternal(id, ring, product.Rings[ring]));
		} // for ( auto ring = 1; ring <= 3; ++ring )

		if (!product.Cap.empty()) {
			addExternal(generateProductCapExternal(id, product.Cap));
		} // if ( !product.Cap.empty() )
	}   // for ( auto index = 0; index < static_cast<int>(Products.size()); ++index
	// )
	worldLocker.unlock();
	// Locked: clingo

	reqLocker.relock();
	// Locked: clingo, RequestMutex
	// Copy the requests to release the lock especially before grounding!
	auto groundRequests(std::move(GroundRequests));
	auto releaseRequests(std::move(ReleaseRequests));
	auto assignRequests(std::move(AssignRequests));
	reqLocker.unlock();
	// Locked: clingo

	if (!groundRequests.empty()) {
		std::vector<Clingo::Part> parts;
		parts.reserve(groundRequests.size());
		for (const auto &request : groundRequests) {
			parts.emplace_back(request.first, request.second);
		} // for ( const auto& request : groundRequests )
		clingo->ground(parts);
	} // if ( !groundRequests.empty() )

	for (const auto &atom : releaseRequests) {
		clingo->release_external(atom);
	} // for ( const auto& atom : releaseRequests )

	for (const auto &atom : assignRequests) {
		clingo->assign_external(atom, true);
	} // for ( const auto& atom : assignRequests )

	MutexLocker solvingLokcer(&SolvingMutex);
	worldLocker.relock();
	// Locked: clingo, SolvingMutex, WorldMutex

	auto currentTimeExternal = [](const int time) {
		return Clingo::Function("currentTime", {Clingo::Number(time)});
	};

	if (GameTime >= ExplorationTime && !ProductionStarted) {
		ProductionStarted = true;
		clingo->ground({{"startProduction", {}}});
		clingo->release_external(Clingo::Id("productionStarted"));
	} // if ( GameTime >= ExplorationTime && !ProductionStarted )

	const auto aspGameTime = realGameTimeToAspGameTime(GameTime);
	for (auto gt = realGameTimeToAspGameTime(StartSolvingGameTime); gt < aspGameTime; ++gt) {
		clingo->release_external(currentTimeExternal(gt));
	} // for ( auto gt = realGameTimeToAspGameTime(StartSolvingGameTime); gt <
	// aspGameTime; ++gt )
	StartSolvingGameTime = GameTime;
	worldLocker.unlock();
	// Locked: clingo, SolvingMutex
	clingo->assign_external(currentTimeExternal(aspGameTime), true);
	MutexLocker planLocker(&PlanMutex);
	// Locked: clingo, SolvingMutex, PlanMutex
	for (auto &robotPlan : Plan) {
		robotPlan.second.FirstNotDoneOnSolveStart = robotPlan.second.FirstNotDone;
	} // for ( auto& robotPlan : Plan )
	planLocker.unlock();
	// Locked: clingo, SolvingMutex
	SolvingStarted = Clock::now();
	clingo->start_solving();
	return;
}

/**
 * @brief Queues a ground request.
 * @param[in] request The request to build a Clingo::Part to ground.
 * @param[in] reason The reason for the request.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlannerThread::queueGround(GroundRequest &&       request,
                              const char *           reason,
                              const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	GroundRequests.push_back(std::move(request));
	setInterrupt(interrupt, reason);
	return;
}

/**
 * @brief Queues a release request.
 * @param[in] atom The atom to release.
 * @param[in] reason The reason for the request.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlannerThread::queueRelease(Clingo::Symbol &&      atom,
                               const char *           reason,
                               const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	ReleaseRequests.push_back(std::move(atom));
	setInterrupt(interrupt, reason);
	return;
}

/**
 * @brief Queues a assign request.
 * @param[in] atom The atom to release.
 * @param[in] reason The reason for the request.
 * @param[in] interrupt Which level of interrupt is requested.
 */
void
AspPlannerThread::queueAssign(Clingo::Symbol &&      atom,
                              const char *           reason,
                              const InterruptSolving interrupt)
{
	MutexLocker locker(&RequestMutex);
	AssignRequests.push_back(std::move(atom));
	setInterrupt(interrupt, reason);
	return;
}

/**
 * @brief Sets the interrupt value.
 * @param[in] interrupt Which level of interrupt is requested.
 * @param[in] reason The reason of the interrupt.
 */
void
AspPlannerThread::setInterrupt(const InterruptSolving interrupt, const char *reason)
{
	if (static_cast<short>(interrupt) > static_cast<short>(Interrupt.load())) {
		Interrupt       = interrupt;
		InterruptReason = reason;
	} // if ( static_cast<short>(interrupt) > static_cast<short>(Interrupt.load())
	// )
	return;
}

/**
 * @brief Says if the solving process should be interrupted.
 * @return If the solving should be interrupted.
 */
bool
AspPlannerThread::shouldInterrupt(void)
{
	const auto now(Clock::now());
	switch (Interrupt) {
	case InterruptSolving::Not: {
		static auto                       lastCheck(now);
		static const std::chrono::seconds threshold(
		  config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/robot-task-check"));

		if (now - lastCheck >= threshold) {
			lastCheck = now;
			MutexLocker worldLocker(&WorldMutex);

			for (const auto &pair : Robots) {
				if (pair.second.Alive) {
					const auto &doing(pair.second.Doing);
					if (doing.isValid()) {
						static const std::chrono::seconds threshold(config->get_int(
						  std::string(ConfigPrefix) + "interrupt-thresholds/robot-task-behind"));
						if (GameTime > doing.EstimatedEnd + threshold.count()) {
							logger->log_warn(LoggingComponent,
							                 "Robot %s is more than %ld seconds behind schedule "
							                 "(and has not send an update), increase interrupt level.",
							                 pair.first.c_str(),
							                 threshold.count());
							setInterrupt(InterruptSolving::High, "Delayed Task");
						} // if ( GameTime > doing.EstimatedEnd + threshold.count() )
					}   // if ( doing.isValid() )
				}     // if ( pair.second.Alive )
			}       // for ( const auto& pair : Robots )
		}         // if ( now - lastCheck >= threshold )

		// Check down here, so we have released the locks!
		if (Interrupt != InterruptSolving::Not) {
			return shouldInterrupt();
		} // if ( Interrupt != InterruptSolving::Not )
		break;
	} // case InterruptSolving::Not
	case InterruptSolving::JustStarted: {
		static const std::chrono::seconds threshold(
		  config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/just-started"));
		MutexLocker locker(&SolvingMutex);
		const auto  diff(now - SolvingStarted);
		return diff <= threshold;
	} // case InterruptSolving::JustStarted
	case InterruptSolving::Normal: {
		static const std::chrono::seconds threshold(
		  config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/normal"));
		MutexLocker locker(&PlanMutex);
		const auto  diff(now - LastPlan);
		return diff <= threshold;
	} // case InterruptSolving::Normal
	case InterruptSolving::High: {
		static const std::chrono::seconds threshold(
		  config->get_int(std::string(ConfigPrefix) + "interrupt-thresholds/high"));
		MutexLocker locker(&PlanMutex);
		const auto  diff(now - LastPlan);
		return diff <= threshold;
	} // case InterruptSolving::High
	case InterruptSolving::Critical: return true;
	} // switch ( Interrupt )
	return false;
}

/**
 * @brief Is called, when the solver has found a new model.
 * @return If the solver should search for additional models.
 */
bool
AspPlannerThread::newModel(void)
{
	MutexLocker aspLocker(clingo.objmutex_ptr());
	MutexLocker locker(&SolvingMutex);
	Symbols    = clingo->model_symbols();
	NewSymbols = true;
	LastModel  = Clock::now();
	return true;
}

/**
 * @brief Is called, when the solver is finished. Either because the search is
 * exhausted, the program is infeasible or we told him to be finished.
 * @param[in] result Contains the information what condition has been met.
 */
void
AspPlannerThread::solvingFinished(const Clingo::SolveResult &result)
{
	if (result.is_unsatisfiable()) {
		++Unsat;
		logger->log_error(LoggingComponent, "The input is infeasiable! #%d", Unsat);
	} // if ( result.is_unsatisfiable() )
	else {
		Unsat = 0;
	} // else -> if ( result.is_unsatisfiable() )
	return;
}

/**
 * @brief Used to implement external functions in ASP, e.g. drive duration based
 * on the nav graph.
 * @param[in] loc The location from where the function is called.
 * @param[in] name The function name.
 * @param[in] arguments The ASP arguments for the function.
 * @param[in] retFunction The function used to return the calculated value.
 */
void
AspPlannerThread::groundFunctions(const Clingo::Location &    loc,
                                  const char *                name,
                                  const Clingo::SymbolSpan &  arguments,
                                  Clingo::SymbolSpanCallback &retFunction)
{
	if (clingo->debug_level() >= fawkes::ClingoAccess::ASP_DBG_ALL) {
		std::stringstream functionCall;
		functionCall << name << '(';
		auto sep = "";
		for (const auto &argument : arguments) {
			functionCall << sep << argument;
			sep = ", ";
		} // for ( const auto& argument : arguments )
		functionCall << ')';

		const auto functionCallStr(functionCall.str());
		logger->log_warn(LoggingComponent, "Called %s.", functionCallStr.c_str());
	}

	string_view view(name);

	switch (arguments.size()) {
	case 0: {
		if (view == "explorationTaskDuration") {
			static const int dur = [this](void) {
				char buffer[std::strlen(ConfigPrefix) + 40];
				std::strcpy(buffer, ConfigPrefix);
				std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/explore-zone");
				return config->get_int(buffer);
			}();
			retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
			return;
		} // if ( view == "explorationTaskDuration" )
		else if (view == "getTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(FetchProductTaskDuration))});
			return;
		} // if ( view == "getTaskDuration" )
		else if (view == "prepareCSTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(PrepareCSTaskDuration))});
			return;
		} // if ( view == "prepareCSTaskDuration" )
		else if (view == "mountCapTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
			return;
		} // if ( view == "mountCapTaskDuration" )
		else if (view == "mountRingTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
			return;
		} // if ( view == "mountRingTaskDuration" )
		else if (view == "feedRSTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
			return;
		} // if ( view == "feedRSTaskDuration" )
		else if (view == "deliverTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(DeliverProductTaskDuration))});
			return;
		} // if ( view == "deliverTaskDuration" )
		else if (view == "getProductTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(FetchProductTaskDuration))});
			return;
		} // if ( view == "getProductTaskDuration" )
		else if (view == "maxDriveDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxDriveDuration))});
			return;
		} // else if ( view == "maxDriveDuration" )
		else if (view == "maxProducts") {
			retFunction({Clingo::Number(MaxProducts)});
			return;
		} // else if ( view == "maxProducts" )
		else if (view == "robots") {
			static const auto robots = [this](void) {
				Clingo::SymbolVector ret;
				ret.reserve(Robots.size());

				for (const auto &robot : PossibleRobots) {
					ret.emplace_back(Clingo::String(robot.c_str()));
				} // for ( const auto& robot : PossibleRobots )
				return ret;
			}();
			retFunction({robots.data(), robots.size()});
			return;
		} // else if ( view == "robots" )
		else if (view == "horizon") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(LookAhaed))});
			return;
		} // else if ( view == "maxTicks" )
		else if (view == "maxTaskDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxTaskDuration))});
			return;
		} // else if ( view == "maxTaskDuration" )
		else if (view == "maxOrders") {
			retFunction({Clingo::Number(MaxOrders)});
			return;
		} // else if ( view == "maxOrders" )
		else if (view == "maxQuantity") {
			retFunction({Clingo::Number(MaxQuantity)});
			return;
		} // else if ( view == "maxQuantity" )
		else if (view == "minDeliveryTime") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(ExplorationTime))});
			return;
		} // else if ( view == "minDeliveryTime" )
		else if (view == "maxDeliveryTime") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(ProductionEnd))});
			return;
		} // else if ( view == "maxDeliveryTime" )
		else if (view == "maxWorkingDuration") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxWorkingDuration))});
			return;
		} // else if ( view == "maxWorkingDuration" )
		break;
	} // case 0
	case 1: {
		if (view == "capColor") {
			const auto index = arguments[0].number();
			assert(index >= 1 && index <= 2);
			retFunction({Clingo::String(CapColors[index - 1].Color)});
			return;
		} // if ( view == "capColor" )
		else if (view == "clingoToASP") {
			retFunction({Clingo::Number(realGameTimeToAspGameTime(arguments[0].number()))});
			return;
		} // else if ( view == "clingoToASP" )
		else if (view == "machineWorkingDuration") {
			retFunction(
			  {Clingo::Number(realGameTimeToAspGameTime(WorkingDurations[arguments[0].string()]))});
			return;
		} // else if ( view == "machineWorkingDuration" )
		break;
	} // case 1
	} // switch ( arguments.size() )

	std::stringstream functionCall;
	functionCall << name << '(';
	auto sep = "";
	for (const auto &argument : arguments) {
		functionCall << sep << argument;
		sep = ", ";
	} // for ( const auto& argument : arguments )
	functionCall << ')';

	const auto functionCallStr(functionCall.str());
	throw fawkes::Exception("Called function %s from %s:%d-%d, but there exists "
	                        "no definition of %s/%d!",
	                        functionCallStr.c_str(),
	                        loc.begin_file(),
	                        loc.begin_line(),
	                        loc.begin_column(),
	                        name,
	                        arguments.size());
	return;
}

/**
 * @brief Sets the team for the solver.
 */
void
AspPlannerThread::setTeam(void)
{
	MutexLocker aspLocker(clingo.objmutex_ptr());
	clingo->ground({{"ourTeam", {Clingo::String(TeamColor)}}});
	ProgramGrounded = true;

	fillNavgraphNodesForASP(true);
	NodesToFind.clear();
	NodesToFind.reserve(6);
	for (const auto &machine : {"BS", "CS1", "CS2", "DS", "RS1", "RS2"}) {
		NodesToFind.insert(std::string(TeamColor) + "-" + machine + "-I");
	} // for ( const auto& machine : {"BS", "CS1", "CS2", "DS", "RS1", "RS2"} )
	DeliveryLocation = generateLocationExternal(TeamColor, "DS", "I");
	CapLocations[0]  = generateLocationExternal(TeamColor, "CS1", "I");
	CapLocations[1]  = generateLocationExternal(TeamColor, "CS2", "I");
	RingLocations[0] = generateLocationExternal(TeamColor, "RS1", "I");
	RingLocations[1] = generateLocationExternal(TeamColor, "RS2", "I");
	graph_changed();
	logger->log_warn(LoggingComponent,
	                 "Setting UpdateNavgraphDistance from %s to true. %s",
	                 UpdateNavgraphDistances ? "true" : "false",
	                 __func__);
	UpdateNavgraphDistances = true;
	return;
}

/**
 * @brief Unsets the team color, we have to restart all over again.
 */
void
AspPlannerThread::unsetTeam(void)
{
	// TODO: I think it is clear what to do.
	throw fawkes::Exception("Should unset the team, but this is not implemented!");
	return;
}

/**
 * @brief Adds an order to asp.
 * @param[in] order The order.
 * @note Assumes the world mutex is locked!
 */
void
AspPlannerThread::addOrderToASP(const OrderInformation &order)
{
	Clingo::SymbolVector  params = {Clingo::Number(order.Number),
                                 Clingo::Number(order.Quantity),
                                 Clingo::String(order.Base),
                                 Clingo::String(order.Cap),
                                 Clingo::String(order.Rings[1]),
                                 Clingo::String(order.Rings[2]),
                                 Clingo::String(order.Rings[3]),
                                 Clingo::Number(realGameTimeToAspGameTime(order.DeliveryBegin)),
                                 Clingo::Number(realGameTimeToAspGameTime(order.DeliveryEnd))};
	constexpr const char *reason = "New order";

	queueGround({"newOrder", std::move(params)}, reason, InterruptSolving::Critical);

	for (const auto &color : BaseColors) {
		queueRelease(Clingo::Function("base", {Clingo::Number(order.Number), Clingo::String(color)}),
		             reason);
	} // for ( const auto& color : BaseColors )

	for (const auto &color : CapColors) {
		queueRelease(Clingo::Function("cap",
		                              {Clingo::Number(order.Number), Clingo::String(color.Color)}),
		             reason);
	} // for ( const auto& color : CapColors )

	for (const auto &color : RingColors) {
		for (auto ring = 1; ring <= 3; ++ring) {
			queueRelease(Clingo::Function("ring",
			                              {Clingo::Number(order.Number),
			                               Clingo::Number(ring),
			                               Clingo::String(color.Color)}),
			             reason);
		} // for ( auto ring = 1; ring <= 3; ++ring )
	}   // for ( const auto& color : RingColors )

	for (auto qty = order.Quantity + 1; qty <= MaxQuantity; ++qty) {
		for (auto ring = 1; ring <= 3; ++ring) {
			for (const auto &location : RingLocations) {
				queueRelease(generateMountRingExternal(location, order.Number, qty, ring), reason);
			} // for ( const auto& location : RingLocations )
		}   // for ( auto ring = 1; ring <= 3; ++ring )
		for (const auto &location : CapLocations) {
			queueRelease(generateMountCapExternal(location, order.Number, qty), reason);
		} // for ( const auto& location : CapLocations )
		queueRelease(generateDeliverExternal(DeliveryLocation, order.Number, qty), reason);
		queueRelease(generateLateDeliverExternal(DeliveryLocation, order.Number, qty), reason);
	} // for ( auto qty = order.Quantity + 1; qty <= MaxQuantity; ++qty )

	for (auto qty = 1; qty <= order.Quantity; ++qty) {
		const auto capIndex =
		  std::find_if(
		    CapColors.begin(),
		    CapColors.end(),
		    [&order](const CapColorInformation &info) noexcept { return info.Color == order.Cap; })
		    ->Machine[2]
		  - '1';

		OrderTasks tasks{Clingo::Symbol(),
		                 Clingo::Symbol(),
		                 Clingo::Symbol(),
		                 Clingo::Symbol(),
		                 generateMountCapExternal(CapLocations[capIndex], order.Number, qty),
		                 generateDeliverExternal(DeliveryLocation, order.Number, qty),
		                 generateLateDeliverExternal(DeliveryLocation, order.Number, qty)};

		// Make an explicit copy because queueAssign takes an rvalue.
		queueAssign(Clingo::Symbol(tasks.CapTask), reason);
		queueAssign(Clingo::Symbol(tasks.DeliverTasks[0]), reason);
		queueAssign(Clingo::Symbol(tasks.DeliverTasks[1]), reason);
		queueRelease(generateMountCapExternal(CapLocations[1 - capIndex], order.Number, qty), reason);

		auto ring = 1;
		for (; ring <= 3; ++ring) {
			const auto &color(order.Rings[ring]);
			if (color == "none") {
				break;
			} // if ( color == "none" )

			const auto ringIndex = std::find_if(
			                         RingColors.begin(),
			                         RingColors.end(),
			                         [&order, &color](const RingColorInformation &info) noexcept {
				                         return info.Color == color;
			                         })
			                         ->Machine[2]
			                       - '1';

			tasks.RingTasks[ring] =
			  generateMountRingExternal(RingLocations[ringIndex], order.Number, qty, ring);
			queueAssign(Clingo::Symbol(tasks.RingTasks[ring]), reason);
			queueRelease(generateMountRingExternal(RingLocations[1 - ringIndex], order.Number, qty, ring),
			             reason);
		} // for ( auto ring = 1; ring <= 3; ++ring )

		for (; ring <= 3; ++ring) {
			for (const auto &location : RingLocations) {
				queueRelease(generateMountRingExternal(location, order.Number, qty, ring), reason);
			} // for ( const auto& location : RingLocations )
		}   // for ( ; ring <= 3; ++ring )

		OrderTaskMap.insert({{order.Number, qty}, std::move(tasks)});
	} // for ( auto qty = 1; qty <= order.Quantity; ++qty )
	return;
}

/**
 * @brief Adds the ring info to asp.
 * @param[in] info The info.
 */
void
AspPlannerThread::addRingColorToASP(const RingColorInformation &info)
{
	const auto            color(Clingo::String(info.Color));
	constexpr const char *reason = "Ring color";
	queueGround({"setRingInfo", {color, Clingo::Number(info.Cost), Clingo::String(info.Machine)}},
	            reason);

	for (const auto &machine : {"RS1", "RS2"}) {
		queueRelease(Clingo::Function("ringStationAssignment", {Clingo::String(machine), color}),
		             reason);
	} // for ( const auto& machine : {"RS1", "RS2"} )

	for (auto cost = 0; cost <= 2; ++cost) {
		queueRelease(Clingo::Function("ringColorCost", {color, Clingo::Number(cost)}), reason);
	} // for ( auto cost = 0; cost <= 2; ++cost )
	return;
}

/**
 * @brief Adds a zone to explore.
 * @param[in] zone The zone number.
 */
void
AspPlannerThread::addZoneToExplore(const int zone)
{
	assert(zone >= 1 && zone <= 24);
	queueAssign(generateExploreLocationExternal(zone), "Zone added");
	queueAssign(generateExploreTaskExternal(zone), "Zone added");
	return;
}

/**
 * @brief Releases the externals for a zone exploration.
 * @param[in] zone The zone number.
 * @param[in] removeAndFillNodes If @link ZonesToExplore the vector @endlink
 * should be cleaned and the fillNavgraphNodesForASP() should be called.
 */
void
AspPlannerThread::releaseZone(const int zone, const bool removeAndFillNodes)
{
	assert(zone >= 1 && zone <= 24);
	if (removeAndFillNodes) {
		MutexLocker locker(&WorldMutex);
		ZonesToExplore.erase(std::find(ZonesToExplore.begin(), ZonesToExplore.end(), zone));
		fillNavgraphNodesForASP(false);
	} // if ( removeAndFillNodes )
	queueRelease(generateExploreLocationExternal(zone), "Zone released");
	queueRelease(generateExploreTaskExternal(zone), "Zone released");
	return;
}

static inline Clingo::SymbolVector
splitParameters(string_view string)
{
	std::vector<Clingo::Symbol> ret;
	ret.reserve(std::count(string.begin(), string.end(), ','));

	auto comma       = string.find(',');
	auto paranthesis = string.find('(');
	while (comma != string_view::npos || paranthesis != string_view::npos) {
		if (comma != string_view::npos && comma < paranthesis) {
			auto param = string.substr(0, comma);

			if (std::isdigit(param[0])) {
				char *unused;
				ret.push_back(Clingo::Number(std::strtol(param.data(), &unused, 10)));
			} // if ( std::isdigit(param[0]) )
			else {
				assert(param[0] == '"');
				ret.push_back(Clingo::String(std::string(param.substr(1, param.size() - 2))));
			} // else -> if ( std::isdigit(param[0]) )
			string.remove_prefix(comma + 1);
		} // if ( comma != string_view::npos && comma < paranthesis )
		else if (paranthesis != string_view::npos) {
			auto index = paranthesis + 1;
			for (auto count = 1; count != 0; ++index) {
				if (string[index] == ')') {
					--count;
				} // if ( string[index] == ')' )
				else if (string[index] == '(') {
					++count;
				} // else if ( string[index] == '(' )
			}   // for ( auto count = 1u, index = paranthesis + 1; count != 0; ++index )

			const auto substrStart(paranthesis + 1), substrEnd(index - 1 - substrStart);
			ret.push_back(Clingo::Function(std::string(string.substr(0, paranthesis)).c_str(),
			                               splitParameters(string.substr(substrStart, substrEnd))));

			/* +1 for the following comma, but when there is no comma, i.e. this
       * function was the last parameter bound the removed size by the strings
       * size. Removing more is undefined. */
			string.remove_prefix(std::min(index + 1, string.size()));
		} // else if ( paranthesis != string_view::npos )
		comma       = string.find(',');
		paranthesis = string.find('(');
	} // while ( comma != string_view::npos || paranthesis != string_view::npos )

	if (string.size() != 0) {
		if (std::isdigit(string[0])) {
			char *unused;
			ret.push_back(Clingo::Number(std::strtol(string.data(), &unused, 10)));
		} // if ( std::isdigit(string[0]) )
		else {
			assert(string[0] == '"');
			ret.push_back(Clingo::String(std::string(string.substr(1, string.size() - 2))));
		} // else -> if ( std::isdigit(string[0]) )
	}   // if ( string.size() != 0 )

	return ret;
}

/**
 * @brief Creates a task description.
 * @param[in] task The task as string.
 * @param[in] end The estimated end for the task.
 * @return The new description.
 */
static inline TaskDescription
createTaskDescription(const std::string &task, const int end)
{
	auto type = TaskDescription::None;

	string_view taskView(task);
	const auto  paramsBegin = taskView.find('(');
	string_view taskName(taskView.substr(0, paramsBegin));
	string_view params(taskView.substr(paramsBegin + 1));
	// Remove the ) from the parameters.
	params.remove_suffix(1);

	if (taskName == "deliver" || taskName == "lateDeliver") {
		type = TaskDescription::Deliver;
	} // if ( taskName == "deliver" || taskName == "lateDeliver" )
	else if (taskName == "feedRS") {
		type = TaskDescription::FeedRS;
	} // else if ( taskName == "feedRS" )
	else if (taskName == "getBase") {
		type = TaskDescription::GetBase;
	} // else if ( taskName == "getBase" )
	else if (taskName == "getProduct") {
		type = TaskDescription::GetProduct;
	} // else if ( taskName == "getProduct" )
	else if (taskName == "goto") {
		type = TaskDescription::Goto;
	} // else if ( taskName == "goto" )
	else if (taskName == "mountCap") {
		type = TaskDescription::MountCap;
	} // else if ( taskName == "mountCap" )
	else if (taskName == "mountRing") {
		type = TaskDescription::MountRing;
	} // else if ( taskName == "mountRing" )
	else if (taskName == "prepareCS") {
		type = TaskDescription::PrepareCS;
	} // else if ( taskName == "prepareCS" )
	else {
		throw fawkes::Exception("Unknown task name: %s!", taskName.to_string().c_str());
	} // else -> all tasks

	return TaskDescription{type,
	                       Clingo::Function(taskName.to_string().c_str(), splitParameters(params)),
	                       end};
}

/**
 * @brief Sets the interrupt flag, depending on the offset a task feedback has.
 * @param[in] offset The offset.
 */
void
AspPlannerThread::checkForInterruptBasedOnTimeOffset(int offset)
{
	auto getOffset = [this](const char *severity) {
		constexpr auto infix = "/interrupt-thresholds/offset-";
		return std::min(config->get_int(std::string(ConfigPrefix) + infix + severity + "-absolute"),
		                config->get_int(std::string(ConfigPrefix) + infix + severity + "-percent")
		                  * TimeResolution / 100);
	};

	static const auto criticalOffset   = getOffset("critical");
	static const auto highlOffset      = getOffset("high");
	static const auto normalOffset     = getOffset("normal");
	static const auto justStarteOffset = getOffset("started");

	logger->log_info(LoggingComponent, "Plan-Feedback offset is %d.", offset);
	offset = std::abs(offset);

	if (offset >= criticalOffset) {
		setInterrupt(InterruptSolving::Critical, "Very high plan offset");
	} // if ( offset >= criticalOffset )
	else if (offset >= highlOffset) {
		setInterrupt(InterruptSolving::High, "High plan offset");
	} // else if ( offset >= highlOffset )
	else if (offset >= normalOffset) {
		setInterrupt(InterruptSolving::Normal, "Plan offset");
	} // else if ( offset >= normalOffset )
	else if (offset >= justStarteOffset) {
		setInterrupt(InterruptSolving::JustStarted, "Very small offset");
	} // else if ( offset >= justStarteOffset )
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
AspPlannerThread::robotBegunWithTask(const std::string &robot,
                                     const std::string &task,
                                     const int          begin,
                                     const int          end)
{
	MutexLocker worldLocker(&WorldMutex);
	MutexLocker planLocker(&PlanMutex);

	auto &robotPlan(Plan[robot]);
	auto &robotInfo(Robots[robot]);

	assert(robotPlan.CurrentTask.empty());
	assert(!robotInfo.Doing.isValid());

	auto &firstNotDoneTask(robotPlan.Tasks[robotPlan.FirstNotDone]);

	if (firstNotDoneTask.Task != task) {
		logger->log_error(LoggingComponent, "Plan invalid, the robot started another task.");
		firstNotDoneTask.Task  = task;
		firstNotDoneTask.Begin = begin;
		firstNotDoneTask.End   = end;
		setInterrupt(InterruptSolving::Critical, "Robot started \"wrong\" task");
	} // if ( firstNotDoneTask.Task != task )

	robotPlan.CurrentTask  = task;
	firstNotDoneTask.Begun = true;
	const auto offset      = begin - firstNotDoneTask.Begin;
	static_assert(std::is_signed<decltype(offset)>::value, "Offset has to have a sign!");
	firstNotDoneTask.Begin = begin;
	planLocker.unlock();
	worldLocker.unlock();
	checkForInterruptBasedOnTimeOffset(offset);
	worldLocker.relock();
	planLocker.relock();

	robotInfo.Doing = createTaskDescription(task, firstNotDoneTask.End);

	const auto location = robotInfo.Doing.TaskSymbol.arguments().front();
	const auto iter     = LocationInUse.find(location);

	if (iter != LocationInUse.end() && iter->second != robot) {
		logger->log_warn(LoggingComponent,
		                 "The robots %s and %s are trying to use %s! Tell %s to "
		                 "stop immediately and delete its current plan!",
		                 iter->second.c_str(),
		                 robot.c_str(),
		                 location.to_string().c_str(),
		                 robot.c_str());
		robotPlan.CurrentTask.clear();
		firstNotDoneTask.Begun = false;
		robotInfo.Doing        = {};
		tellRobotToStop(robot);
		setInterrupt(InterruptSolving::Critical, "Location conflict");
		for (auto index = robotPlan.FirstNotDone; index < robotPlan.Tasks.size(); ++index) {
			removeFromPlanDB(robot, index);
		} // for ( auto index = robotPlan.FirstNotDone; index <
		// robotPlan.Tasks.size(); ++index )
		robotPlan.Tasks.erase(robotPlan.Tasks.begin() + robotPlan.FirstNotDone, robotPlan.Tasks.end());
	} // if ( iter != LocationInUse.end() && iter->second != robot )
	else {
		LocationInUse.insert({location, robot});
	} // else -> if ( iter != LocationInUse.end() && iter->second != robot )
	return;
}

/**
 * @brief A robot updates the time estimation for a task, add it to the program.
 * @param[in] robot The robot.
 * @param[in] task The task.
 * @param[in] end The new estimated end time.
 */
void
AspPlannerThread::robotUpdatesTaskTimeEstimation(const std::string &robot,
                                                 const std::string &task,
                                                 const int          end)
{
	MutexLocker worldLocker(&WorldMutex);
	MutexLocker planLocker(&PlanMutex);

	auto &robotPlan(Plan[robot]);
	auto &robotInfo(Robots[robot]);

	assert(robotPlan.CurrentTask == task);
	assert(robotPlan.Tasks[robotPlan.FirstNotDone].Task == task);
	assert(robotInfo.Doing.isValid());

	const auto offset = end - robotPlan.Tasks[robotPlan.FirstNotDone].End;
	static_assert(std::is_signed<decltype(offset)>::value, "Offset has to have a sign!");
	/* Do NOT update the end value, because we calculate the offset based on that.
   * If we receive multiple small offsets that do not trigger replaning this
   * does not work as intended. robotPlan.Tasks[robotPlan.FirstNotDone].End =
   * end; */
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
AspPlannerThread::robotFinishedTask(const std::string &robot,
                                    const std::string &task,
                                    const int          end,
                                    const bool         success)
{
	MutexLocker worldLocker(&WorldMutex);
	MutexLocker planLocker(&PlanMutex);

	auto &robotPlan(Plan[robot]);
	auto &robotInfo(Robots[robot]);

	auto &firstNotDoneTask(robotPlan.Tasks[robotPlan.FirstNotDone]);

	assert(robotPlan.CurrentTask == task);
	assert(firstNotDoneTask.Task == task);
	assert(robotInfo.Doing.isValid());

	const auto offset = end - firstNotDoneTask.End;
	static_assert(std::is_signed<decltype(offset)>::value, "Offset has to have a sign!");
	firstNotDoneTask.End = end;
	planLocker.unlock();
	worldLocker.unlock();
	checkForInterruptBasedOnTimeOffset(offset);
	worldLocker.relock();
	planLocker.relock();

	firstNotDoneTask.Done = true;
	++robotPlan.FirstNotDone;
	robotPlan.CurrentTask.clear();

	const auto location = robotInfo.Doing.TaskSymbol.arguments().front();
	assert(LocationInUse[location] == robot);
	LocationInUse.erase(location);

	if (!success) {
		robotInfo.Doing         = {};
		firstNotDoneTask.Failed = true;
		setInterrupt(InterruptSolving::Critical, "Task failed");
		for (auto i = robotPlan.FirstNotDone; i < robotPlan.Tasks.size(); ++i) {
			removeFromPlanDB(robot, i);
		} // for ( auto i = robotPlan.FirstNotDone; i < robotPlan.Tasks.size(); ++i
		// )
		robotPlan.Tasks.erase(robotPlan.Tasks.begin() + robotPlan.FirstNotDone, robotPlan.Tasks.end());
		return;
	} // if ( !success )

	auto generateProduct = [this](const string_view &machine,
	                              std::string &&     baseColor) -> ProductIdentifier {
		if (static_cast<int>(Products.size()) >= MaxProducts) {
			logger->log_error(LoggingComponent,
			                  "Have to generate a product, this would be #%zu alive, but only "
			                  "%d are configured. This product will not be part of the ASP program "
			                  "until products with a lower "
			                  "id will be destroyed!",
			                  Products.size() + 1,
			                  MaxProducts);
		} // if ( static_cast<int>(Products.size()) >= MaxProducts )

		logger->log_info(LoggingComponent,
		                 "%s generated product #%zu with base color %s at %d.",
		                 machine.data(),
		                 Products.size(),
		                 baseColor.c_str(),
		                 GameTime);
		Products.push_back({std::move(baseColor)});
		return {static_cast<decltype(ProductIdentifier::ID)>(Products.size() - 1)};
	};
	auto destroyProduct = [this](const string_view &machine, const ProductIdentifier &id) {
		logger->log_info(
		  LoggingComponent, "%s destroies product #%d at %d.", machine.data(), id.ID, GameTime);
		Products.erase(Products.begin() + id.ID);
		for (auto &pair : Robots) {
			if (pair.second.Holding.ID > id.ID) {
				--pair.second.Holding.ID;
			} // if ( pair.second.Holding.ID > id.ID )
		}   // for ( auto& pair : Robots )
		for (auto &pair : Machines) {
			if (pair.second.Storing.ID > id.ID) {
				--pair.second.Storing.ID;
			} // if ( pair.second.Storing.ID > id.ID )
		}   // for ( auto& pair : Machines )
		return;
	};

	auto machinePickup = [this](const std::string &machine, const ProductIdentifier &id) {
		auto &machineInfo(Machines[machine]);
		assert(!machineInfo.Storing.isValid());
		machineInfo.Storing      = id;
		machineInfo.WorkingUntil = GameTime + WorkingDurations[machine];
		logger->log_info(LoggingComponent,
		                 "Machine %s works on product #%d from %d until %d.",
		                 machine.c_str(),
		                 id.ID,
		                 GameTime,
		                 machineInfo.WorkingUntil);
		return;
	};

	auto machineDrops = [this](const std::string &machine) {
		auto &machineInfo(Machines[machine]);
		assert(machineInfo.Storing.isValid());
		auto id                  = machineInfo.Storing;
		machineInfo.Storing      = {};
		machineInfo.WorkingUntil = 0;
		logger->log_info(LoggingComponent,
		                 "Product #%d was taken from machine %s at %d.",
		                 id.ID,
		                 machine.c_str(),
		                 GameTime);
		return id;
	};

	auto robotPickups = [this, &robotInfo, &robot](const ProductIdentifier &id) {
		assert(!robotInfo.Holding.isValid());
		robotInfo.Holding = id;
		logger->log_info(
		  LoggingComponent, "Robot %s picks up product #%d at %d.", robot.c_str(), id.ID, GameTime);
		return;
	};
	auto robotDrops = [this, &robotInfo, &robot](void) {
		assert(robotInfo.Holding.isValid());
		auto id           = robotInfo.Holding;
		robotInfo.Holding = {};
		logger->log_info(
		  LoggingComponent, "Robot %s drops product #%d at %d.", robot.c_str(), id.ID, GameTime);
		return id;
	};

	const decltype(auto) taskArguments(robotInfo.Doing.TaskSymbol.arguments());
	const decltype(auto) machine(taskArguments[0].arguments()[1].string());

	auto getOrder = [&taskArguments](void) {
		return std::make_pair<int, int>(taskArguments[1].number(), taskArguments[2].number());
	};

	constexpr const char *taskDoneReason = "Task done";

	switch (robotInfo.Doing.Type) {
	case TaskDescription::None: break; // Does not happen. (See assert above.)
	case TaskDescription::Deliver: {
		auto order(getOrder());
		queueRelease(std::move(OrderTaskMap[order].DeliverTasks[0]), taskDoneReason);
		queueRelease(std::move(OrderTaskMap[order].DeliverTasks[1]), taskDoneReason);
		destroyProduct("DS", robotDrops());
		break;
	} // case TaskDescription::Deliver
	case TaskDescription::FeedRS: {
		const auto &product(Products[robotInfo.Holding.ID]);
		if (!product.Rings[1].empty() || !product.Cap.empty()) {
			logger->log_warn(LoggingComponent,
			                 "%s used a non trivial product to feed a ring station!",
			                 robot.c_str());
		} // if ( !product.Rings[1].empty() || !product.Cap.empty() )
		destroyProduct(machine, robotDrops());
		auto &info(Machines[machine]);
		assert(info.FillState <= 3);
		++info.FillState;
		logger->log_info(LoggingComponent, "Fillstate of %s is now: %d", machine, info.FillState);
		break;
	} // case TaskDescription::FeedRS
	case TaskDescription::GetBase:
		robotPickups(generateProduct("BS", taskArguments[1].string()));
		break;
	case TaskDescription::GetProduct: robotPickups(machineDrops(machine)); break;
	case TaskDescription::Goto: break; // The robots location will be fetched from the navgraph.
	case TaskDescription::MountCap: {
		auto &machineInfo(Machines[machine]);
		assert(machineInfo.Prepared);
		auto order(getOrder());
		auto product = robotDrops();
		assert(Products[product.ID].Cap.empty());
		Products[product.ID].Cap = Orders[order.first].Cap;
		queueRelease(std::move(OrderTaskMap[order].CapTask), taskDoneReason);
		machinePickup(machine, product);
		machineInfo.Prepared = false;
		logger->log_info(LoggingComponent,
		                 "%s mounted a %s cap on product #%d at %d.",
		                 machine,
		                 Products[product.ID].Cap.c_str(),
		                 product.ID,
		                 GameTime);
		break;
	} // case TaskDescription::MountCap
	case TaskDescription::MountRing: {
		auto       order(getOrder());
		const auto ringNumber = taskArguments[3].number();
		auto       product    = robotDrops();
		assert(Products[product.ID].Rings[ringNumber].empty());
		const auto ringColor = Orders[order.first].Rings[ringNumber];
		auto &     machineInfo(Machines[machine]);
		const auto ringInfo = std::find_if(RingColors.begin(),
		                                   RingColors.end(),
		                                   [&ringColor](const RingColorInformation &info) {
			                                   return info.Color == ringColor;
		                                   });
		assert(machineInfo.FillState >= ringInfo->Cost);
		Products[product.ID].Rings[ringNumber] = Orders[order.first].Rings[ringNumber];
		queueRelease(std::move(OrderTaskMap[order].RingTasks[ringNumber]), taskDoneReason);
		machinePickup(machine, product);
		machineInfo.FillState -= ringInfo->Cost;
		logger->log_info(LoggingComponent,
		                 "%s mounted the %d. ring with color %s on product #%d at %d. "
		                 "The new fillstate is %d.",
		                 machine,
		                 ringNumber,
		                 Products[product.ID].Rings[ringNumber].c_str(),
		                 product.ID,
		                 GameTime,
		                 machineInfo.FillState);
		break;
	} // case TaskDescription::MountRing
	case TaskDescription::PrepareCS: {
		auto &machineInfo(Machines[machine]);
		assert(!machineInfo.Prepared);
		machinePickup(machine, generateProduct(machine, "TRANSPARENT"));
		machineInfo.Prepared = true;
		logger->log_info(LoggingComponent, "Machine %s is now prepared.", machine);
		break;
	} // case TaskDescription::PrepareCS
	} // switch ( robotInfo.Doing.Type )
	robotInfo.Doing = {};
	return;
}
