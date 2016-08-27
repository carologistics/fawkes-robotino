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

#include <clingo.hh>

#include <cstring>

/**
 * @var AspPlanerThread::Control
 * @brief The Clingo-Control.
 *
 * @var AspPlanerThread::ClingoDebug
 * @brief Wether there should be debug output regarding grounding, solving, etc.
 */

/**
 * @brief Takes care of everything regarding clingo interface in the constructor.
 */
void AspPlanerThread::constructClingo(void)
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

	Control = new Clingo::Control({}, clingoLogger);
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in init().
 */
void AspPlanerThread::initClingo(void)
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

	Log->log_info(LoggingComponent, "Loading program files from %s. Debug state: %s", path.c_str(),
		ClingoDebug ? "true" : "false");
	for ( const auto file : files )
	{
		Log->log_info(LoggingComponent, "Loading file program file %s.", file.c_str());
		Control->load((path + file).c_str());
	} //for ( const auto file : files )

	ground({Clingo::Part("base", Clingo::SymbolSpan())});
	return;
}

/**
 * @brief Takes care of everything regarding clingo interface in finalize().
 */
void AspPlanerThread::finalizeClingo(void)
{
	delete Control;
	Control = nullptr;
	return;
}

/**
 * @brief Grounds the given parts and if wished for prints that it does so.
 */
void AspPlanerThread::ground(const Clingo::PartSpan& parts)
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

	Control->ground(parts);

	if ( ClingoDebug )
	{
		Log->log_info(LoggingComponent, "Grounding done.");
	} //if ( ClingoDebug )
	return;
}

