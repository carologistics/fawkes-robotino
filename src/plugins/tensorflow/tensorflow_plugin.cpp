/***************************************************************************
 *  tensorflow_plugin.cpp - Plugin for using tensorflow inside fawkes
 *
 *  Created: Thu May 5 10:23:50 2019
 *  Copyright  2019 Morian Sonnet
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

#include "tensorflow_thread.h"

using namespace fawkes;

/** Plugin to use tensorflow inside fawkes
 *  * @author Morian Sonnet
 *   */
class TensorflowPlugin : public fawkes::Plugin {
public:
  /** Constructor.
   *    * @param config Fawkes configuration
   *       */
  TensorflowPlugin(Configuration *config) : Plugin(config) {
    std::string prefix = "/tensorflow/";
    thread_list.push_back(new TensorflowThread(prefix));
  }
};

PLUGIN_DESCRIPTION("Plugin for using tensorflow inside Fawkes")
EXPORT_PLUGIN(TensorflowPlugin)
