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
 * @var AspPlanerThread::ClingoDebug
 * @brief Wether there should be debug output regarding grounding, solving, etc.
 *
 * @var AspPlanerThread::MoreModels
 * @brief If we want to have more than one model (if available) from the solver.
 *
 * @var AspPlanerThread::ClingoMutex
 * @brief Protects all clingo related members.
 *
 * @var AspPlanerThread::Control
 * @brief The Clingo-Control.
 *
 * @var AspPlanerThread::Solving
 * @brief Wether clingo is solving.
 *
 * @var AspPlanerThread::LastTick
 * @brief The last tick for the asp program.
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
	auto clingoLogger = [this](const Clingo::WarningCode code, char const *msg) {
			fawkes::Logger::LogLevel level = fawkes::Logger::LL_NONE;
			switch ( code ) {
				case Clingo::WarningCode::AtomUndefined      :
				case Clingo::WarningCode::OperationUndefined :
				case Clingo::WarningCode::RuntimeError       : level = fawkes::Logger::LL_ERROR; break;
				case Clingo::WarningCode::Other              :
				case Clingo::WarningCode::VariableUnbounded  : level = fawkes::Logger::LL_WARN;
				case Clingo::WarningCode::FileIncluded       :
				case Clingo::WarningCode::GlobalVariable     : level = fawkes::Logger::LL_INFO; break;
			} //switch ( code )
			Log->log(level, "Clingo", msg);
			return;
		};

	MutexLocker locker(&ClingoMutex);
	Control = new Clingo::Control({}, clingoLogger);
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in init().
 */
void
AspPlanerThread::initClingo(void)
{
	constexpr auto infix = "planer/";
	const auto prefixLen = std::strlen(ConfigPrefix), infixLen = std::strlen(infix);
	char buffer[prefixLen + infixLen + 20];
	const auto suffix = buffer + prefixLen + infixLen;
	std::strcpy(buffer, ConfigPrefix);
	std::strcpy(buffer + prefixLen, infix);

	std::strcpy(suffix, "program-path");
	const auto path  = Config->get_string(buffer);
	std::strcpy(suffix, "program-files");
	const auto files = Config->get_strings(buffer);
	std::strcpy(suffix, "debug");
	ClingoDebug = Config->get_bool(buffer);
	std::strcpy(suffix, "more-models");
	MoreModels  = Config->get_bool(buffer);

	Log->log_info(LoggingComponent, "Loading program files from %s. Debug state: %s", path.c_str(),
		ClingoDebug ? "true" : "false");
	MutexLocker locker(&ClingoMutex);
	for ( const auto file : files )
	{
		Log->log_info(LoggingComponent, "Loading file program file %s.", file.c_str());
		Control->load((path + file).c_str());
	} //for ( const auto file : files )
	locker.unlock();

	ground({Clingo::Part("base", Clingo::SymbolSpan())});
	solve();
	return;
}

/**
 * @brief Does the loop for clingo.
 */
void
AspPlanerThread::loopClingo(void)
{
	MutexLocker cliLocker(&ClingoMutex);
	MutexLocker reqLocker(&RequestMutex);
	if ( Solving || Requests.empty() )
	{
		return;
	} //if ( Solving || Requests.empty() )

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
	ground(parts);
	parts.clear();
	Requests.clear();
	reqLocker.unlock();

	solve();
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in finalize().
 */
void
AspPlanerThread::finalizeClingo(void)
{
	MutexLocker locker(&ClingoMutex);
	delete Control;
	Control = nullptr;
	return;
}

/**
 * @brief Resets the clingo solver.
 */
void
AspPlanerThread::resetClingo(void)
{
	Log->log_info(LoggingComponent, "Resetting Clingo.");
	MutexLocker locker(&ClingoMutex);
	if ( Solving )
	{
		Control->interrupt();
		Solving = false;
	} //if ( Solving )
	RequestMutex.lock();
	Requests.clear();
	RequestMutex.unlock();
	LastTick = 0;

	finalizeClingo();
	constructClingo();
	initClingo();
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
 * @brief Grounds the given parts and if wished for prints that it does so.
 */
void
AspPlanerThread::ground(const Clingo::PartSpan& parts)
{
	if ( ClingoDebug )
	{
		Log->log_info(LoggingComponent, "Grounding %d parts:", parts.size());
		auto i = 0;
		for ( const Clingo::Part& part : parts )
		{
			std::string params;
			bool first = true;
			for ( const auto& param : part.params() )
			{
				if ( first )
				{
					first = false;
				} //if ( first )
				else
				{
					params += ", ";
				} //else -> if ( first )
				params += param.to_string();
			} //for ( const auto& param : part.params() )
			Log->log_info(LoggingComponent, "Part #%d: %s [%s]", ++i, part.name(), params.c_str());
		} //for ( const auto& part : parts )
	} //if ( ClingoDebug )

	MutexLocker locker(&ClingoMutex);
	Control->ground(parts);

	if ( ClingoDebug )
	{
		Log->log_info(LoggingComponent, "Grounding done.");
	} //if ( ClingoDebug )
	return;
}

/**
 * @brief Starts the solving process, if it isn't running.
 */
void
AspPlanerThread::solve(void)
{
	MutexLocker locker(&ClingoMutex);
	if ( Solving )
	{
		return;
	} //if ( Solving )
	Solving = true;
	if ( ClingoDebug )
	{
		Log->log_info(LoggingComponent, "Start solving.");
	} //if ( ClingoDebug )
	Control->solve_async([this](const Clingo::Model& model) { return newModel(model); },
		[this](const Clingo::SolveResult& result) { solvingFinished(result); return; });
	return;
}

/**
 * @brief Is called, when the solver has found a new model.
 * @param[in] newModel The new Model.
 * @return If the solver should search for additional models.
 */
bool
AspPlanerThread::newModel(const Clingo::Model& model)
{
	const Clingo::SymbolVector symbols = model.symbols();
	if ( ClingoDebug )
	{
		static Clingo::SymbolVector oldSymbols;
		Log->log_info(LoggingComponent, "New model found.");

		/* To save (de-)allocations just move found symbols at the end of the vector and move the end iterator to the
		 * front. After this everything in [begin, end) is in oldSymbols but not in symbols. */
		auto begin = oldSymbols.begin(), end = oldSymbols.end();

		for ( const Clingo::Symbol& symbol : symbols )
		{
			auto iter = std::find(begin, end, symbol);
			if ( iter == end )
			{
				Log->log_info(LoggingComponent, "New Symbol: %s", symbol.to_string().c_str());
			} //if ( iter == end )
			else
			{
				std::swap(*iter, *--end);
			} //else -> if ( iter == end )
		} //for ( const Clingo::Symbol& symbol : symbols )

		for ( ; begin != end; ++begin )
		{
			Log->log_info(LoggingComponent, "Symbol removed: %s", begin->to_string().c_str());
		} //for ( ; begin != end; ++begin )

		oldSymbols = symbols;
	} //if ( ClingoDebug )
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
	if ( ClingoDebug )
	{
		Log->log_info(LoggingComponent, "Solving done.");
	} //if ( ClingoDebug )
	MutexLocker locker(&ClingoMutex);
	Solving = false;

	if ( result.is_unsatisfiable() )
	{
		throw fawkes::Exception("The program is infeasable! We have no way to recover!");
	} //if ( result.is_unsatisfiable() )
	return;
}

void
AspPlanerThread::setTeam(const bool cyan)
{
	std::vector<Clingo::Symbol> param(1, Clingo::String(cyan ? "C" : "M"));
	queueGround({"ourTeam", param, false});
	return;
}

void
AspPlanerThread::unsetTeam(void)
{
	resetClingo();
	return;
}
