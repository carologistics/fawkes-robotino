
----------------------------------------------------------------------------
--  test_agent.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
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
name               = "get_s0"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"ppgoto", "fetch_puck", "leave_IS", "motor_move", "global_motor_move", "get_rid_of_puck"}
depends_interfaces = {
  {v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
  {v = "ppnavi", type = "NavigatorInterface"},
  {v = "motor", type = "MotorInterface", id="Robotino"},
  {v = "puck_0", type="Position3DInterface", id="puck_0"}
}

documentation      = [==[Get a new S0 resource puck
This skill drives to an input storage and fetches a puck from there.

Parameters:
      place: name of insertion area (e.g. Ins1)
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants
local LOSTPUCK_DIST = 0.08
local PUCK_SENSOR_INDEX = 8
if config:exists("/hardware/robotino/puck_sensor/trigger_dist") then
   LOSTPUCK_DIST = config:get_float("/hardware/robotino/puck_sensor/trigger_dist")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/trigger_dist FOUND! Using default value\n");
end
if config:exists("/hardware/robotino/puck_sensor/index") then
   -- you can find the config value in /cfg/host.yaml
   PUCK_SENSOR_INDEX = config:get_uint("/hardware/robotino/puck_sensor/index")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/index FOUND! Using default value\n");
end

function puck_visible()
   return puck_0:visibility_history() >= 1
end

function have_place(self)
   if self.fsm.vars.place ~= nil then
      return true
   end
end

fsm:define_states{ export_to=_M,
   closure={have_place=have_place, sensor = sensor, LOSTPUCK_DIST = LOSTPUCK_DIST, PUCK_SENSOR_INDEX = PUCK_SENSOR_INDEX},
   {"INIT", JumpState},
   {"GOTO_IS", SkillJumpState, skills={{ppgoto}}, final_to="SKILL_GLOBAL_MOTOR_MOVE", fail_to="FAILED"},
   {"SKILL_GLOBAL_MOTOR_MOVE", SkillJumpState, skills={{global_motor_move}}, final_to="MOVE_SIDEWAYS", fail_to="FAILED"},
   {"MOVE_SIDEWAYS", SkillJumpState, skills={{motor_move}}, final_to="FAILED", fail_to="FAILED"},--when this is final we reached the end of the insertion area, so we fail
   {"SKILL_FETCH_PUCK", SkillJumpState, skills={{fetch_puck}}, final_to="SKILL_LEAVE_AREA",
      fail_to="MOVE_SIDEWAYS"},
   {"SKILL_LEAVE_AREA", SkillJumpState, skills={{leave_IS}}, final_to="FINAL", fail_to="FAILED"},
   {"GET_RID_OF_PUCK", SkillJumpState, skills={{get_rid_of_puck}}, final_to="GOTO_IS", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not have_place(self)", desc="Called get_s0 without parameter place!"},
   {"INIT", "GOTO_IS", cond=have_place},
   {"MOVE_SIDEWAYS", "SKILL_FETCH_PUCK", cond=puck_visible},
   {"GOTO_IS", "GET_RID_OF_PUCK", cond = "sensor:distance(PUCK_SENSOR_INDEX) <= LOSTPUCK_DIST and sensor:distance(PUCK_SENSOR_INDEX) > 0", desc = "Picked up another puck while driving, escape"}
}

function SKILL_FETCH_PUCK:init()
   ppnavi:msgq_enqueue_copy(ppnavi.StopMessage:new())
end

function GOTO_IS:init()
   if self.fsm.vars.place == nil then
      printf("Called get_s0 without parameter place!")
   end

   self.fsm.vars.move_right_count = 0
   self.skills[1].place = self.fsm.vars.place
end

function SKILL_GLOBAL_MOTOR_MOVE:init()
   self.skills[1].place = self.fsm.vars.place
end

function MOVE_SIDEWAYS:init()
   if self.fsm.vars.place == "Ins1" or self.fsm.vars.place == "Ins2" then
      self.skills[1].y = -1
      self.skills[1].vel_trans = 0.05
   elseif self.fsm.vars.place == "Ins1Sec" or self.fsm.vars.place == "Ins2Sec" then
      self.skills[1].y = 1
      self.skills[1].vel_trans = 0.05
   end
end

function SKILL_LEAVE_AREA:init()
   self.skills[1].place = self.fsm.vars.place
end
function SKILL_FETCH_PUCK:init()
   self.skills[1].move_sideways = "true"
end
