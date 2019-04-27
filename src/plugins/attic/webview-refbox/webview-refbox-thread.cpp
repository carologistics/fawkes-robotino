
/***************************************************************************
 *  webview-refbox-thread.cpp - RCLL refbox webview
 *
 *  Created: Tue Mar 17 17:53:14 2015
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

#include "webview-refbox-thread.h"

#include "webview-refbox-processor.h"

#include <webview/nav_manager.h>
#include <webview/request_manager.h>
#include <webview/url_manager.h>

using namespace fawkes;

#define REFBOX_URL_PREFIX "/refbox"

/** @class WebviewRCLLRefBoxThread "webview-refbox-thread.h"
 * Show refbox log data via webview.
 * @author Tim Niemueller
 */

/** Constructor. */
WebviewRCLLRefBoxThread::WebviewRCLLRefBoxThread()
: Thread("WebviewRCLLRefBoxThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
WebviewRCLLRefBoxThread::~WebviewRCLLRefBoxThread()
{
}

void
WebviewRCLLRefBoxThread::init()
{
	std::string nav_entry = "RCLL RefBox";
	try {
		nav_entry = config->get_string("/webview/refbox/nav-entry");
	} catch (Exception &e) {
	} // ignored, use default

	web_proc_ = new WebviewRCLLRefBoxRequestProcessor(REFBOX_URL_PREFIX, mongodb_client);
	webview_url_manager->register_baseurl(REFBOX_URL_PREFIX, web_proc_);
	webview_nav_manager->add_nav_entry(REFBOX_URL_PREFIX, nav_entry.c_str());
}

void
WebviewRCLLRefBoxThread::finalize()
{
	webview_url_manager->unregister_baseurl(REFBOX_URL_PREFIX);
	webview_nav_manager->remove_nav_entry(REFBOX_URL_PREFIX);
	delete web_proc_;
}

void
WebviewRCLLRefBoxThread::loop()
{
}
