/***************************************************************************
 *  clips_gym_rcll_plugin.cpp - 
 *
 *  Created: 
 *  Copyright  2022  
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

#include "clips_gym_rcll_thread.h"

#include <core/plugin.h>

#include <iostream>
/** @class CLIPS Gym plugin
 *  Interface to use the clips executive from a customized OpenAI Gym Environment in python
 *  @author Sonja Ginter
 */

class ClipsGymRCLLPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
     *  @param config Fawkes configuration to read config values from.
     */
	explicit ClipsGymRCLLPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		std::cout << "\nClipsGymRCLLPlugin adding ClipsGymThread instance to thread list" << std::endl;
		thread_list.push_back(ClipsGymRCLLThread::getInstance()); //new ClipsGymThread());
	}
};

PLUGIN_DESCRIPTION("Interface to use the CX from a customized OpenAI Gym Environment in python")
EXPORT_PLUGIN(ClipsGymRCLLPlugin)