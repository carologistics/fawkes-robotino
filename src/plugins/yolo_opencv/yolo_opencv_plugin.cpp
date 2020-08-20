/***************************************************************************
 *  yolo_opencv_plugin.cpp - Plugin to run Yolo
 *  Created: Thu Aug 20 17:43:00
 *  Copyright  2020 Niklas Sebastian Eltester
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

#include "yolo_opencv_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to ...
 *  * @author Niklas Sebastian Eltester
 *   */
class YoloOpenCVPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   *    * @param config Fawkes configuration
   *       */
	YoloOpenCVPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new YoloOpenCVThread());
	}
};

PLUGIN_DESCRIPTION("Plugin to run Yolo")
EXPORT_PLUGIN(YoloOpenCVPlugin)
