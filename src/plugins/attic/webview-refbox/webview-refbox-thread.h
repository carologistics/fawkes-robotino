
/***************************************************************************
 *  webview-refbox-thread.h - RCLL refbox webview
 *
 *  Created: Tue Mar 17 17:51:34 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_REFBOX_WEBVIEW_REFBOX_THREAD_H_
#define __PLUGINS_WEBVIEW_REFBOX_WEBVIEW_REFBOX_THREAD_H_

#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <core/threading/thread.h>
#include <plugins/mongodb/aspect/mongodb.h>

class WebviewRCLLRefBoxRequestProcessor;

class WebviewRCLLRefBoxThread : public fawkes::Thread,
                                public fawkes::LoggingAspect,
                                public fawkes::ClockAspect,
                                public fawkes::ConfigurableAspect,
                                public fawkes::MongoDBAspect,
                                public fawkes::WebviewAspect
{
public:
	WebviewRCLLRefBoxThread();
	virtual ~WebviewRCLLRefBoxThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	WebviewRCLLRefBoxRequestProcessor *web_proc_;
};

#endif
