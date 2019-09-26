/***************************************************************************
 *  webtools_bridge_plugin.cpp -  Gives Websocket access ,by mimicking the
 *rosbridge protocol, to different Fawkes components
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

#include "webtools_bridge_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** @class WebtoolsBridgePlugin
 * The plug-in implements a websocket server enabling sessions to make topic
 * subscription, advertisement, as well as, service calls to different Fawkes
 * components using rosbridge protocol JSON messages.
 * Implementation is done in similar way fashion to rosbridge server. In fact
 * when the requests are intended to ros, the plug-in acts merely as a proxy
 * server and routes the request to rosbridge server.
 *
 * @author Mostafa Gomaa
 */
class WebtoolsBridgePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	WebtoolsBridgePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new WebtoolsBridgeThread());
	}
};

PLUGIN_DESCRIPTION("webtools-bridge")
EXPORT_PLUGIN(WebtoolsBridgePlugin)
