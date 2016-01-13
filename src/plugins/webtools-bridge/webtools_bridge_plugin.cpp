/***************************************************************************
 *  webtools_bridge_plugin.cpp - Websocket access to diffrent components mimicing the rosbridge protocol
 *
 *  Created: Wed Jan 13 16:33:00 2016 
 *  Copyright  2016 Mostafa Gomaa 
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

#include <core/plugin.h>

#include "webtools_bridge_thread.h"

using namespace fawkes;

class WebtoolsBridgePlugin : public fawkes::Plugin
{
 public:

 WebtoolsBridgePlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new WebtoolsBridgeThread());
  }
};

PLUGIN_DESCRIPTION("webtools-bridge")
EXPORT_PLUGIN(WebtoolsBridgePlugin)
