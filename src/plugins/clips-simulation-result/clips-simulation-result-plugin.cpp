#include "clips-simulation-result-thread.h"

#include <core/plugin.h>

using namespace fawkes;

class ClipsSimulationResultPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ClipsSimulationResultPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ClipsSimulationResultThread());
	}
};

PLUGIN_DESCRIPTION("CLIPS feature to write simulation results into a csv file")
EXPORT_PLUGIN(ClipsSimulationResultPlugin)