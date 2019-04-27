/***************************************************************************
 *  webtools_bridge_thread.h -  Gives Websocket access ,by mimicking the
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

#ifndef __PLUGINS_WEBTOOLS_BRIDGE_THREAD_H_
#define __PLUGINS_WEBTOOLS_BRIDGE_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <plugins/clips/aspect/clips_manager.h>

#include <string>
#include <vector>

namespace fawkes {
class Position3DInterface;
}

class BridgeManager;

class WebSocketServer;

class WebtoolsBridgeThread : public fawkes::Thread,
                             public fawkes::ClockAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::CLIPSManagerAspect
{
public:
	WebtoolsBridgeThread();
	virtual ~WebtoolsBridgeThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	std::shared_ptr<BridgeManager>   bridge_manager_;
	std::shared_ptr<WebSocketServer> web_server_;
};

#endif
