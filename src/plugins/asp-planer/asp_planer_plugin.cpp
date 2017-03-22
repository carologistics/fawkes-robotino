
/***************************************************************************
 *  asp_planer_plugin.cpp - Planer plugin to schedule commands given to the
 *                          asp agent
 *
 *  Created on Thu Aug 18 04:20:02 2016
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

#include <aspect/blocked_timing.h>
#include <core/plugin.h>
#include <plugins/rrd/aspect/rrd.h>
#include "asp_planer_thread.h"

#include <fstream>

class AspRRDThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::RRDAspect
{
	private:
	fawkes::RRDDefinition *RRDDef;
	fawkes::RRDGraphDefinition *RRDGraph;
	static constexpr auto RRDStepSize = std::chrono::seconds{5};

	public:
	AspRRDThread(void) : Thread("AspPlanerRRDThread", Thread::OPMODE_WAITFORWAKEUP),
		BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP),
		RRDDef(nullptr), RRDGraph(nullptr)
	{
		return;
	}
	~AspRRDThread(void)
	{
		delete RRDDef;
		delete RRDGraph;
		return;
	}

	void init(void) override
	{
		using namespace fawkes;
		using namespace std::chrono_literals;

		std::vector<RRDDataSource> rrds;
		constexpr double max = 1024. * 1024. * 1024. * 15; //We do not excpect more than 15 GiB needed memory.
		rrds.emplace_back("Memory", RRDDataSource::GAUGE, 2 * RRDStepSize.count(), 0., max);
		RRDDef = new RRDDefinition("Memory", rrds, RRDStepSize.count(), true);
		rrd_manager->add_rrd(RRDDef);

		std::vector<RRDGraphDataDefinition> graphDefinitions;
		std::vector<RRDGraphElement*> graphElements;

		graphDefinitions.emplace_back("Memory", RRDArchive::AVERAGE, RRDDef);

		graphElements.push_back(new RRDGraphLine("Memory", 1, "FF0000", "Memory", false));
		graphElements.push_back(new RRDGraphGPrint("Memory", RRDArchive::MIN,     "Minimum\\:%8.2lf %s"));
		graphElements.push_back(new RRDGraphGPrint("Memory", RRDArchive::AVERAGE, "Average\\:%8.2lf %s"));
		graphElements.push_back(new RRDGraphGPrint("Memory", RRDArchive::MAX,     "Maximum\\:%8.2lf %s\\n"));

//		const auto now = std::chrono::duration_cast<std::chrono::seconds>(::Clock::now().time_since_epoch());
		const auto now = std::chrono::seconds{std::time(nullptr)};
		const std::chrono::seconds end = now + 16min;
		RRDGraph = new RRDGraphDefinition("Memory", RRDDef, "Memory consumption", "Used memory (GiB)",
			graphDefinitions, graphElements, now.count(), end.count(), RRDStepSize.count(), RRDStepSize.count(), false);

		rrd_manager->add_graph(RRDGraph);
		addRRDEntry();
		return;
	}

	void loop(void) override
	{
		const auto now = Clock::now();
		static std::remove_const<decltype(now)>::type lastAdd;

		if ( now - lastAdd >= RRDStepSize )
		{
			addRRDEntry();
			lastAdd = now;
		} //if ( now - lastAdd >= RRDStepSize )
		return;
	}

	void finalize(void) override
	{
		rrd_manager->remove_rrd(RRDDef);
		return;
	}

	void addRRDEntry(void)
	{
		std::ifstream file("/proc/self/status");

		if ( !file.is_open() ) {
//			logger->log_error(LoggingComponent, "Could not open \"/proc/self/status\"!");
			return;
		} //if ( !file.is_open() )

		bool done = false;
		std::string line;

		std::int64_t memory = 0;

		do //while ( !file.eof() && !done )
		{
			std::getline(file, line);
			if ( line.find("VmRSS:") == 0 )
			{
				/*The line looks like: VmRSS:	  617604 kB
				 * Or:                  VmRSS:	10001424 kB
				 * See the missing spaces, so we can't look from the back for ' ', but have to look for ' ' or '\t'.
				 * There is no API in std::string for that, so we look from the front for a digit. */
				done = true;

				const auto numberPosEnd = line.rfind(' '), numberPosBegin = line.find_first_of("123456789");

				const auto end = line.begin() + numberPosEnd;
				for ( auto iter = line.begin() + numberPosBegin; iter != end; ++iter )
				{
					static_assert('1' - '0' == 1, "");
					memory *= 10;
					memory += *iter - '0';
				} //for ( auto iter = line.begin() + numberPosBegin; iter != end; ++iter )

				memory *= 1024;
			} //if ( line.find("VmRSS:") == 0 )
		} while ( !file.eof() && !done );

		try
		{
			if ( memory )
			{
				constexpr double gebibyte = 1024. * 1024. * 1024.;
				rrd_manager->add_data(RRDDef->get_name(), "N:%f", static_cast<double>(memory) / gebibyte);
			} //if ( memory )
		} //try
		catch ( fawkes::Exception& e )
		{
//			logger->log_error(LoggingComponent, "Error while adding entry in RRD: %s", e.what());
		} //catch ( fawkes::Exception& e )
		return;
	}
};

/** ASP planer plugin.
 */
class AspPlanerPlugin : public fawkes::Plugin
{
	public:
	/** Constructor.
	* @param config Fawkes configuration
	*/
	AspPlanerPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new AspPlanerThread);
		thread_list.push_back(new AspRRDThread);
		return;
	}
};


PLUGIN_DESCRIPTION("ASP-based planer plugin")
EXPORT_PLUGIN(AspPlanerPlugin)
