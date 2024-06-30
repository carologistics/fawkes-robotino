/***************************************************************************
 *  rto_data_in_plugin.cpp - Get data input reading from robotino node
 *
 *  Created: Jun 2024
 *  Copyright  2024  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "rto_data_in_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to listen to robotino input data from ros driver
 * @author Tarik Viehmann
 */
class RTODataInPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
    * @param config Fawkes configuration
    */
	explicit RTODataInPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new RTODataInThread());
	}
};

PLUGIN_DESCRIPTION("ROS bridge to read data inputs of robotino")
EXPORT_PLUGIN(RTODataInPlugin)
