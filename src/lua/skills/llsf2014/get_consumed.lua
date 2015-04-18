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
   {v = "motor",  type = "MotorInterface", id="Robotino" },
   {v = "puck",   type="Position3DInterface", id="puck_0"},
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

function puck_visible()
   return puck:visibility_history() >= 1
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
   {"GOTO_MACHINE_RECYCLE", SkillJumpState, skills={{drive_to}},        final_to="MOVE_SIDEWAYS",        fail_to="GOTO_MACHINE_RECYCLE"},
   {"MOVE_SIDEWAYS",        SkillJumpState, skills={{motor_move}},      final_to="FAILED",               fail_to="MOVE_SIDEWAYS"},
   {"GRAP_CONSUMEND",       SkillJumpState, skills={{fetch_puck}},      final_to="FINAL",                fail_to="FAILED"},
   {"GET_RID_OF_PUCK",      SkillJumpState, skills={{get_rid_of_puck}}, final_to="GOTO_MACHINE_RECYCLE", fail_to="GOTO_MACHINE_RECYCLE"},
}

fsm:add_transitions{
   {"MOVE_SIDEWAYS", "GRAP_CONSUMEND",         cond = puck_visible,  desc = "see puck => grap"},
   {"GOTO_MACHINE_RECYCLE", "GET_RID_OF_PUCK", cond = "have_puck()", desc = "Picked up another puck while driving => escape"},
   {"INIT", "GOTO_MACHINE_RECYCLE", cond=true},
}

function GOTO_MACHINE_RECYCLE:init()
   self.skills[1].same_place = true
   local node = navgraph:node(self.fsm.vars.place)
   local ori  = node:property_as_float("orientation")
   local off_x      = 0.6
   local off_y      = 0.6
   local off_direct = math.sqrt( off_x*off_x + off_y*off_y )
   local off_ori    = math.atan2(off_y, off_x)
   local end_ori
   if node:has_property("leave_right") then
      self.skills[1].x   = node:x() + math.cos(ori - off_ori) * off_direct
      self.skills[1].y   = node:y() + math.sin(ori - off_ori) * off_direct
      end_ori = ori + math.pi / 2
   else
      self.skills[1].x   = node:x() + math.cos(ori + off_ori) * off_direct
      self.skills[1].y   = node:y() + math.sin(ori + off_ori) * off_direct
      end_ori = ori - math.pi / 2
   end

   self.skills[1].ori = end_ori

   printf(ori .. "   " .. end_ori)
end

function MOVE_SIDEWAYS:init()
   self.skills[1].y = -0.5
   if navgraph:node(self.fsm.vars.place):has_property("leave_right") then
      self.skills[1].y = self.skills[1].y * (-1)
   end
   self.skills[1].vel_trans = 0.05
end

function GRAP_CONSUMEND:init()
   if navgraph:node(self.fsm.vars.place):has_property("leave_right") then
      self.skills[1].move_sideways = "right"
   else
      self.skills[1].move_sideways = "left"
   end
end
