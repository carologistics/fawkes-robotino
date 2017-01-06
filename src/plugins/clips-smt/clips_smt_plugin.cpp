
/***************************************************************************
 *  clips_smt_plugin.cpp - Smt CLIPS Feature Plugin
 *
 *  Created on Fry Dec 16 14:42 2016 by Igor Nicolai Bongartz
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

#include "clips_smt_thread.h"
#include <core/plugin.h>

using namespace fawkes;


/** CLIPS smt plugin.
 * @author Igor Nicolai Bongartz
 */
class ClipsSmtPlugin : public fawkes::Plugin
{
 public:
	/** Constructor.
	* @param config Fawkes configuration
	*/
	ClipsSmtPlugin(Configuration *config) : Plugin(config)
	{
	thread_list.push_back(new ClipsSmtThread());
}
};


PLUGIN_DESCRIPTION("CLIPS feature to access the Smt solver")
EXPORT_PLUGIN(ClipsSmtPlugin)
