----------------------------------------------------------------------------
--  get_consumed.lua
--
--  Created: Tue Oct 08 21:01:29 2013
--  Copyright  2013  Frederik Zwilling
--             2014  Tobias Neumann
----------------------------------------------------------------------------

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

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "get_consumed"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"drive_to", "motor_move", "fetch_puck", "get_rid_of_puck"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "sensor", type="RobotinoSensorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Drive to Maschine and ged a consumed puck]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local LOSTPUCK_DIST = 0.08
if config:exists("/hardware/robotino/puck_sensor/trigger_dist") then
   THRESHOLD_DISTANCE = config:get_float("/hardware/robotino/puck_sensor/trigger_dist")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/trigger_dist FOUND! Using default value\n");
end

local PUCK_SENSOR_INDEX = 8
if config:exists("/hardware/robotino/puck_sensor/index") then
   -- you can find the config value in /cfg/host.yaml
   PUCK_SENSOR_INDEX = config:get_uint("/hardware/robotino/puck_sensor/index")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/index FOUND! Using default value\n");
end

function have_puck()
    local curDistance = sensor:distance(PUCK_SENSOR_INDEX)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        return true
    end
    return false
end

fsm:define_states{ export_to=_M, closure={have_puck=have_puck},
   {"INIT", JumpState},
   {"GOTO_MACHINE_RECYCLE", SkillJumpState, skills={{drive_to}}, final_to="GRAP_CONSUMEND", fail_to="GOTO_MACHINE_RECYCLE"},
   {"GRAP_CONSUMEND", SkillJumpState, skills={{fetch_puck}}, final_to="FINAL", fail_to="FAILED"},
   {"GET_RID_OF_PUCK", SkillJumpState, skills={{get_rid_of_puck}}, final_to="GOTO_MACHINE_RECYCLE", fail_to="GOTO_MACHINE_RECYCLE"},
}

fsm:add_transitions{
   {"GOTO_MACHINE_RECYCLE", "GET_RID_OF_PUCK", cond = "have_puck()", desc = "Picked up another puck while driving => escape"},
   {"INIT", "GOTO_MACHINE_RECYCLE", cond=true},
}

function GOTO_MACHINE_RECYCLE:init()
   -- self.skills[1].same_place = true
   self.skills[1].place = navgraph:closest_node_to(self.fsm.vars.place, tostring(self.fsm.vars.place) .. "_recycle"):name()
end
