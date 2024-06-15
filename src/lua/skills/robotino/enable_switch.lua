-- toggle_switch.lua
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.
module(..., skillenv.module_init)
documentation = [==[Toggle the switch interface passed as parameter.
@param iface: Interface to switch (one of laser, motor, lightFront, machine-signal, delivery)
@param enable: true/false, i.e. on/off
]==]

-- Crucial skill information
name = "enable_switch"
fsm = SkillHSM:new{name = name, start = "SWITCH", debug = false}
depends_skills = {}
depends_interfaces = {
    {v = "laserSwitch", type = "SwitchInterface", id = "/laser-cluster/ampel"},
    {v = "motorSwitch", type = "SwitchInterface", id = "Robotino Motor"},
    {
        v = "conveyorSwitch",
        type = "SwitchInterface",
        id = "conveyor_plane/switch"
    }, {v = "realsenseSwitch", type = "SwitchInterface", id = "realsense"},
    {v = "realsense2Switch", type = "SwitchInterface", id = "realsense2"}
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{export_to = _M, {"SWITCH", JumpState}}

fsm:add_transitions{{"SWITCH", "FINAL", cond = true}}

function SWITCH:init()
    ifmap = {
        laser = laserSwitch,
        motor = motorSwitch,
        conveyor = conveyorSwitch,
        realsense = realsenseSwitch,
        realsense2 = realsense2Switch
    }
    iface = ifmap[self.fsm.vars.iface]
    printf("Called for %s (%s)", self.fsm.vars.iface, iface:uid())
    if self.fsm.vars.enable then
        msg = iface.EnableSwitchMessage:new()
    else
        msg = iface.DisableSwitchMessage:new()
    end
    iface:msgq_enqueue(msg)
end
