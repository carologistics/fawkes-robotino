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
 * @property AspPlanerThread::MoreModels
 * @brief If we want to have more than one model (if available) from the solver.
 *
 * @property AspPlanerThread::LastTick
 * @brief The last tick for the asp program.
 *
 * @property AspPlanerThread::Horizon
 * @brief The horizon, up to which point (measured in gametime in seconds) the planer should plan.
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

	std::strcpy(suffix, "debug");
	ClingoAcc->Debug = config->get_bool(buffer);
	std::strcpy(suffix, "more-models");
	MoreModels  = config->get_bool(buffer);
	std::strcpy(suffix, "look-ahaed");
	LookAhaed = config->get_uint(buffer);
	std::strcpy(suffix, "time-resolution");
	TimeResolution = config->get_uint(buffer);

	std::strcpy(buffer + prefixLen, "time-estimations/max-drive-duration");
	MaxDriveDuration = config->get_uint(buffer);

	loadFilesAndGroundBase(locker);
	return;
}

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

	if ( CompleteRestart )
	{
		ClingoAcc->reset();
		loadFilesAndGroundBase(aspLocker);
		Horizon = 0;
	} //if ( CompleteRestart )
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
	for ( GroundRequest& request : requests )
	{
		parts.emplace_back(request.Name, request.Params);
	} //for ( GroundRequest& request : requests )

	//We are not allowed to clear requests before grounding! The params are just stored as "pointers".
	ClingoAcc->ground(parts);
	parts.clear();
	reqLocker.unlock();

	Clingo::Symbol horizonValueSymbol = Clingo::Number(Horizon);
	Clingo::SymbolSpan horizonSpan(&horizonValueSymbol, 1);
	Clingo::Symbol horizonSymbol = Clingo::Function("horizon", horizonSpan);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::False);

	//Cap the horizon at game end.
	const auto oldHorizon(std::move(Horizon));
	//Use a fixed look ahaed of one minute for the exploration phase.
	const auto usedLookAhaed = StillNeedExploring ? 60 : LookAhaed;
	Horizon = realGameTimeToAspGameTime(std::min(GameTime + usedLookAhaed, (15 + 4) * 60u));
	horizonValueSymbol = Clingo::Number(Horizon);
	horizonSymbol = Clingo::Function("horizon", horizonSpan);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::True);

	MutexLocker robLocker(&RobotsMutex);
	std::vector<Clingo::SymbolVector> robotSymbols;
	robotSymbols.reserve(Robots.size());
	for ( auto& paar : Robots )
	{
		auto& info(paar.second);
		releaseExternals(info, false);

		auto distanceToDuration = [this](const unsigned int distance) noexcept {
				//! @todo Werte holen!
				constexpr unsigned int constantCosts = 4;
				constexpr unsigned int costPerDistance = 2;
				return std::min(constantCosts + distance * costPerDistance, MaxDriveDuration);
			};

		for ( const auto machine : {"BS", "CS1", "CS2", "RS1", "RS2", "DS"} )
		{
			std::string target;
			target.reserve(7);
			target += TeamColor;
			target += "-";
			target += machine;
			target += "-X";

			for ( const auto side : {"I", "O"} )
			{
				target.back() = side[0];
				//! @todo NavGraphen Fragen!
				const unsigned int distance = 7;

				Clingo::Symbol symbol(Clingo::Function("driveDuration", {Clingo::String(paar.first.c_str()),
					Clingo::Function("m", {Clingo::String(TeamColor), Clingo::String(machine), Clingo::String(side)}),
					Clingo::Number(realGameTimeToAspGameTime(distanceToDuration(distance)))}));
				ClingoAcc->assign_external(symbol, Clingo::TruthValue::True);
				info.DriveDurations.emplace_back(std::move(symbol));
			} //for ( const auto side : {"I", "O"} )
		} //for ( const auto machine : {"BS", "CS1", "CS2", "RS1", "RS2", "DS"} )

		if ( StillNeedExploring )
		{
			for ( auto zone = 1; zone <= 24; ++zone )
			{
				//! @todo NavGraphen Fragen!
				const unsigned int distance = 7;

				Clingo::Symbol symbol(Clingo::Function("driveDuration", {Clingo::String(paar.first.c_str()),
					Clingo::Function("z", {Clingo::Number(zone)}),
					Clingo::Number(realGameTimeToAspGameTime(distanceToDuration(distance)))}));
				ClingoAcc->assign_external(symbol, Clingo::TruthValue::True);
				info.DriveDurations.emplace_back(std::move(symbol));
			} //for ( auto zone = 1; zone <= 24; ++zone )
		} //if ( StillNeedExploring )
	} //for ( auto& paar : Robots )
	robLocker.unlock();

	parts.reserve(Horizon - oldHorizon);
	Clingo::SymbolVector symbols;
	symbols.reserve((Horizon - oldHorizon) * (StillNeedExploring ? 2 : 1));
	for ( auto i = oldHorizon + 1; i <= Horizon; ++i )
	{
		symbols.emplace_back(Clingo::Number(i));
		parts.emplace_back("transition", Clingo::SymbolSpan(&symbols.back(), 1));
		if ( StillNeedExploring )
		{
			parts.emplace_back("explorationTransition", Clingo::SymbolSpan(&symbols.back(), 1));
		} //if ( StillNeedExploring )
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

	logger->log_info(LoggingComponent, "Loading program files from %s. Debug state: %s", path.c_str(),
		ClingoAcc->Debug ? "true" : "false");
	for ( const auto& file : files )
	{
		ClingoAcc->loadFile(path + file);
	} //for ( const auto& file : files )

	auto symbol(Clingo::Number(0));
	ClingoAcc->ground({Clingo::Part("base", Clingo::SymbolSpan())});
	ClingoAcc->ground({Clingo::Part("transition", Clingo::SymbolSpan(&symbol, 1))});

	if ( StillNeedExploring )
	{
		ClingoAcc->ground({Clingo::Part("explore", Clingo::SymbolSpan())});
		ClingoAcc->ground({Clingo::Part("explorationTransition", Clingo::SymbolSpan(&symbol, 1))});
	} //if ( StillNeedExploring )

	//We have to unlock, to prevent a deadlock in newModel.
	locker.unlock();
	ClingoAcc->startSolvingBlocking();
	locker.relock();
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
	for ( const auto& symbol : info.DriveDurations )
	{
		ClingoAcc->release_external(symbol);
	} //for ( const auto& symbol : info.DriveDurations )
	info.DriveDurations.clear();
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
		throw fawkes::Exception("The program is infeasable! We have no way to recover!");
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
	if ( ClingoAcc->Debug )
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
	} //if ( ClingoAcc->Debug )

	string_view view(name);

	switch ( arguments.size() )
	{
		case 0 :
		{
			if ( view == "explorationTaskDuration" )
			{
				static const unsigned int dur = [this](void) {
						char buffer[std::strlen(ConfigPrefix) + 40];
						std::strcpy(buffer, ConfigPrefix);
						std::strcpy(buffer + std::strlen(ConfigPrefix), "time-estimations/explore-zone");
						return config->get_uint(buffer);
					}();
				retFunction({Clingo::Number(realGameTimeToAspGameTime(dur))});
				return;
			} //if ( view == "explorationTaskDuration" )
			else if ( view == "maxDriveDuration" )
			{
				retFunction({Clingo::Number(realGameTimeToAspGameTime(MaxDriveDuration))});
				return;
			} //else if ( view == "maxDriveDuration" )
			break;
		} //case 0
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
 * @param[in] cyan If the team color is cyan.
 */
void
AspPlanerThread::setTeam(const bool cyan)
{
	Clingo::SymbolVector param(1, Clingo::String(cyan ? "C" : "M"));
	queueGround({"ourTeam", param, false}, InterruptSolving::Critical);
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
 */
void
AspPlanerThread::newTeamMate(const std::string& mate)
{
	Clingo::SymbolVector params;
	params.emplace_back(Clingo::String(mate.c_str()));
	params.emplace_back(Clingo::Number(realGameTimeToAspGameTime(GameTime)));
	queueGround({"addRobot", params, true}, InterruptSolving::Critical);
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
	params.emplace_back(Clingo::Number(zone));
	params.emplace_back(Clingo::Number(realGameTimeToAspGameTime(GameTime)));
	queueGround({"zoneToExplore", params, false}, InterruptSolving::JustStarted);
	return;
}
