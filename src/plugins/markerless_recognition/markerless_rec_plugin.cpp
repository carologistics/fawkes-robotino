/***************************************************************************
 *  *  markerless_rec_plugin.cpp - Plugin for recognizing machines without using the markers
 *   *
 *    *  Created: Thu Sep 7 10:0:00 2017
 *     *  Copyright  2017 Sebastian Schoenitz, Daniel Habering,
 *     				Carsten Stoffels
 *      ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *   *  it under the terms of the GNU General Public License as published by
 *    *  the Free Software Foundation; either version 2 of the License, or
 *     *  (at your option) any later version.
 *      *
 *       *  This program is distributed in the hope that it will be useful,
 *        *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *         *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          *  GNU Library General Public License for more details.
 *           *
 *            *  Read the full text in the LICENSE.GPL file in the doc directory.
 *             */

#include <core/plugin.h>

#include "markerless_rec_thread.h"

using namespace fawkes;

/** Plugin to ...
 *  * @author Daniel Habering, Sebastian Schoenitz, Carsten Stoffels
 *   */
class MarkerlessRecognitionPlugin : public fawkes::Plugin
{
	 public:
	   /** Constructor.
	   *    * @param config Fawkes configuration
	   *       */
	    MarkerlessRecognitionPlugin(Configuration *config)
	     : Plugin(config)
	    {
	       thread_list.push_back(new MarkerlessRecognitionThread());
	    }
};

PLUGIN_DESCRIPTION("Plugin for recognizing machines using the RealSense data")
EXPORT_PLUGIN(Markerless_Rec_Plugin)
	
