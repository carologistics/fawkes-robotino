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

/**
 * @struct GroundRequest
 * @brief A simple container for a ground request.
 *
 * @var GroundRequest::Name
 * @brief The name of the program to ground.
 *
 * @var GroundRequest::Params
 * @brief The parameters for the request.
 *
 * @var GroundRequest::AddTick
 * @brief If the current tick should be appended to the parameters.
 */

/**
 * @var AspPlanerThread::MoreModels
 * @brief If we want to have more than one model (if available) from the solver.
 *
 * @var AspPlanerThread::LastTick
 * @brief The last tick for the asp program.
 *
 * @var AspPlanerThread::LastGameTime
 * @brief The last game time for the asp program.
 *
 * @var AspPlanerThread::Horizon
 * @brief The horizon, up to which point (measured in gametime in seconds) the planer should plan.
 *
 * @var AspPlanerThread::RequestMutex
 * @brief Protects AspPlanerThread::Requests.
 *
 * @var AspPlanerThread::Requests
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

	logger->log_info(LoggingComponent, "Loading program files from %s. Debug state: %s", path.c_str(),
		ClingoAcc->Debug ? "true" : "false");
	for ( const auto& file : files )
	{
		ClingoAcc->loadFile(path + file);
	} //for ( const auto& file : files )

	ClingoAcc->ground({Clingo::Part("base", Clingo::SymbolSpan())});
	ClingoAcc->startSolving();
	return;
}

/**
 * @brief Does the loop for clingo.
 */
void
AspPlanerThread::loopClingo(void)
{
	MutexLocker reqLocker(&RequestMutex);
	if ( ClingoAcc->solving() || Requests.empty() )
	{
		return;
	} //if ( ClingoAcc->solving() || Requests.empty() )

	const Clingo::Symbol tickSymbol = Clingo::Number(LastTick++);
	std::vector<Clingo::Part> parts;
	for ( GroundRequest& request : Requests )
	{
		if ( request.AddTick )
		{
			request.Params.push_back(tickSymbol);
		} //if ( request.AddTick )
		parts.emplace_back(request.Name, request.Params);
	} //for ( GroundRequest& request : Requests )

	//We are not allowed to clear requests before grounding! The params are just stored as "pointers".
	ClingoAcc->ground(parts);
	parts.clear();
	Requests.clear();
	reqLocker.unlock();

	Clingo::Symbol horizonValueSymbol = Clingo::Number(Horizon);
	Clingo::SymbolSpan horizonSpan(&horizonValueSymbol, 1);
	Clingo::Symbol horizonSymbol = Clingo::Function("horizon", horizonSpan);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::False);

	//TODO: How to choose this number? Should it be configurable?
	const unsigned int lookAhaed = GameTime < ExplorationTime ? 120 : 300;
	Horizon = GameTime + lookAhaed;
	horizonValueSymbol = Clingo::Number(Horizon);
	ClingoAcc->assign_external(horizonSymbol, Clingo::TruthValue::True);

	Clingo::SymbolVector symbols;
	symbols.reserve(Horizon - LastGameTime + 1);

	//TODO: Everytime until horizon? Or just the new horizon?
	//TODO: Storing in vector and grounding once or grounding every single program?
//	for ( auto i = LastGameTime; i <= Horizon; ++i )
//	{
//		symbols.emplace_back(Clingo::Number(i));
//		parts.emplace_back("transition", Clingo::SymbolSpan(&symbols.back(), 1));
//	} //for ( auto i = LastGameTime; i <= Horizon; ++i )
	LastGameTime = GameTime;

	ClingoAcc->ground(parts);
//	ClingoAcc->startSolving();
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in finalize().
 */
void
AspPlanerThread::finalizeClingo(void)
{
	return;
}

/**
 * @brief Queues a request for grounding.
 */
void
AspPlanerThread::queueGround(GroundRequest&& request)
{
	MutexLocker locker(&RequestMutex);
	Requests.emplace_back(std::move(request));
	return;
}

/**
 * @brief Is called, when the solver has found a new model.
 * @return If the solver should search for additional models.
 */
bool
AspPlanerThread::newModel(void)
{
	MutexLocker locker(&SymbolMutex);
	Symbols = ClingoAcc->modelSymbols();
	NewSymbols = true;
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

void
AspPlanerThread::setTeam(const bool cyan)
{
	Clingo::SymbolVector param(1, Clingo::String(cyan ? "C" : "M"));
	queueGround({"ourTeam", param, false});
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
	params.emplace_back(Clingo::Id(mate.c_str()));
	params.emplace_back(Clingo::Number(GameTime));
	queueGround({"addRobot", params, true});
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
	params.emplace_back(Clingo::Id(mate.c_str()));
	params.emplace_back(Clingo::Number(GameTime));
	queueGround({"removeRobot", params, true});
	return;
}
