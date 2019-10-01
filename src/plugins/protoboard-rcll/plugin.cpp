
/***************************************************************************
 * Protoboard plugin template instantiation for the LLSF
 *
 * Copyright 2019 Victor Mataré
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

#include <interfaces/PrepareMachineInterface.h>
#include <interfaces/SendBeaconInterface.h>
#include <libs/protoboard/blackboard_manager.h>
#include <libs/protoboard/plugin.h>

namespace protoboard {

std::vector<std::string>
proto_dirs()
{
	return {BASEDIR "/src/libs/llsf_msgs"};
}

} // namespace protoboard

using ProtoboardPluginRCLL = ProtoboardPlugin<
  protoboard::bb_iface_manager<fawkes::SendBeaconInterface,
                               protoboard::type_list<fawkes::SendBeaconInterface::SendBeaconMessage,
                                                     fawkes::SendBeaconInterface::SetPeerMessage>>,
  protoboard::bb_iface_manager<
    fawkes::PrepareMachineInterface,
    protoboard::type_list<fawkes::PrepareMachineInterface::PrepareBSMessage,
                          fawkes::PrepareMachineInterface::PrepareCSMessage,
                          fawkes::PrepareMachineInterface::PrepareDSMessage,
                          fawkes::PrepareMachineInterface::PrepareRSMessage,
                          fawkes::PrepareMachineInterface::PrepareSSMessage,
                          fawkes::PrepareMachineInterface::SetPeerMessage>>>;

PLUGIN_DESCRIPTION("ProtoBoard plugin for the RoboCup Logistics League")
EXPORT_PLUGIN(ProtoboardPluginRCLL)
