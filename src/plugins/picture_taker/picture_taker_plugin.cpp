/***************************************************************************
 *  picture_taker_thread.h - Thread to take a picture
 *  Created: Thu May 7 10:10:00 2017
 *  Copyright  2019  Daniel Habering
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

#include "picture_taker_thread.h"

using namespace fawkes;

/** Plugin to ...
 *  * @author Daniel Habering, Sebastian Schoenitz, Carsten Stoffels
 *   */
class PictureTakerPlugin : public fawkes::Plugin {
public:
  /** Constructor.
   *    * @param config Fawkes configuration
   *       */
  PictureTakerPlugin(Configuration *config) : Plugin(config) {
    thread_list.push_back(new PictureTakerThread());
  }
};

PLUGIN_DESCRIPTION("Plugin to take and store pictures")
EXPORT_PLUGIN(PictureTakerPlugin)
