
/***************************************************************************
 *  localization_plugin.cpp - empty fawkes shell for localization plugin
 *
 *  Created: Do 31. Mai 00:15:20 CEST 2012
 *  Copyright  2012  Daniel Ewert
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

#include <core/plugin.h>

#include "localization_thread.h"

using namespace fawkes;

/**
 * Empty hull without real functionality, shows communication infrastructure 
 * @author Daniel Ewert
 */
class LocalizationPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  LocalizationPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new LocalizationThread());
  }
};

PLUGIN_DESCRIPTION("can be used as starting point for a localization plugin")
EXPORT_PLUGIN(LocalizationPlugin)
