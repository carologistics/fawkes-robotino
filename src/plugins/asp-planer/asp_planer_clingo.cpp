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

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <plugins/asp/aspect/clingo_access.h>

using fawkes::MutexLocker;

static constexpr int doubleCoordToInt(const double d) noexcept
{
	return static_cast<int>(d * 100);
}

static constexpr double intCoordToDouble(const int i) noexcept
{
	return i / 100.;
}

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
 *
 * @property AspPlanerThread::RequestMutex
 * @brief Protects AspPlanerThread::Requests.
 *
 * @property AspPlanerThread::Interrupt
 * @brief If the solving should be interrupted for the new requests.
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
	std::strcpy(suffix, "debug");
	ClingoAcc->Debug = config->get_bool(buffer);
	std::strcpy(suffix, "more-models");
	MoreModels  = config->get_bool(buffer);
	std::strcpy(suffix, "look-ahaed");
	LookAhaed = config->get_uint(buffer);


	logger->log_info(LoggingComponent, "Loading program files from %s. Debug state: %s", path.c_str(),
		ClingoAcc->Debug ? "true" : "false");
	for ( const auto& file : files )
	{
		ClingoAcc->loadFile(path + file);
	} //for ( const auto& file : files )

	auto symbol(Clingo::Number(0));
	ClingoAcc->ground({Clingo::Part("base", Clingo::SymbolSpan())});
	ClingoAcc->ground({Clingo::Part("transition", Clingo::SymbolSpan(&symbol, 1))});

	//We have to unlock, to prevent a deadlock in newModel.
	locker.unlock();
	ClingoAcc->startSolvingBlocking();
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
		if ( interruptSolving() )
		{
			logger->log_warn(LoggingComponent, "Interrupt solving process for new information.");
			assert(!Requests.empty());
			ClingoAcc->cancelSolving();
		} //if ( interruptSolving() )
		else
		{
			return;
		} //else -> if ( interruptSolving )
	} //if ( ClingoAcc->solving() )

	if ( Requests.empty() )
	{
		//Nothing todo.
		//! @todo Increment the lookahaed?!?
		return;
	} //if ( Requests.empty() )

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
	Horizon = std::min(GameTime + LookAhaed, (15 + 4) * 60u);
	horizonValueSymbol = Clingo::Number(Horizon);
	horizonSymbol = Clingo::Function("horizon", horizonSpan);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::True);

	MutexLocker robLocker(&RobotsMutex);
	parts.reserve(Horizon - oldHorizon + Robots.size());
	std::vector<Clingo::SymbolVector> robotSymbols;
	robotSymbols.reserve(Robots.size());
	for ( const auto& info : Robots )
	{
		Clingo::SymbolVector symbols;
		symbols.reserve(4);
		symbols.emplace_back(Clingo::String(info.first.c_str()));
		symbols.emplace_back(Clingo::Number(GameTime));
		symbols.emplace_back(Clingo::Number(doubleCoordToInt(info.second.X)));
		symbols.emplace_back(Clingo::Number(doubleCoordToInt(info.second.Y)));
		robotSymbols.emplace_back(symbols);
		parts.emplace_back("setRobotLocation", Clingo::SymbolSpan(&robotSymbols.back().front(), 4));
	} //for ( const auto& info : Robots )

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
	params.emplace_back(Clingo::Number(GameTime));
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
	params.emplace_back(Clingo::Number(GameTime));
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
