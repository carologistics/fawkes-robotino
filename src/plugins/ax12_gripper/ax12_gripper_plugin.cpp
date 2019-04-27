/***************************************************************************
 *  ax12_gripper_plugin.cpp - Plugin used to control a gripper
 *  using Robotis AX12A - servos
 *
 *  Created: Mon Feb 09 20:28:20 2015
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *                  2015  Nicolas Limpert
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

//#include "gripper_thread.h"
#include "ax12_gripper_thread.h"

using namespace fawkes;

/** Plugin to control gripper using Robotis servos
 * @author Nicolas Limpert
 */
class AX12GripperPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	AX12GripperPlugin(Configuration *config) : Plugin(config)
	{
		std::string prefix = "/hardware/ax12_gripper/";
		bool        active = true;
		try {
			active = config->get_bool((prefix + "active").c_str());
		} catch (Exception &e) {
		} // ignored, assume enabled

		if (active) {
			// GripperAX12AThread * act_thread = new GripperAX12AThread(prefix);
			// GripperSensorThread * sensor_thread = new GripperSensorThread();
			// sensor_thread->add_act_thread(act_thread);
			thread_list.push_back(new GripperAX12AThread(prefix));
			// thread_list.push_back(sensor_thread);
		}
	}
};

PLUGIN_DESCRIPTION("AX12 Gripper Plugin")
EXPORT_PLUGIN(AX12GripperPlugin)
