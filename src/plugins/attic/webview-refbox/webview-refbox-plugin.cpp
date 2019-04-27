
/***************************************************************************
 *  webview-refbox-plugin.cpp - RCLL refbox log visualization
 *
 *  Created: Tue Mar 17 17:50:21 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "webview-refbox-thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** RCLL refbox webview plugin.
 * @author Tim Niemueller
 */
class WebviewRCLLRefBoxPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	WebviewRCLLRefBoxPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new WebviewRCLLRefBoxThread());
	}
};

PLUGIN_DESCRIPTION("RCLL refbox log data")
EXPORT_PLUGIN(WebviewRCLLRefBoxPlugin)
