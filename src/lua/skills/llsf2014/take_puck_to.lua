
----------------------------------------------------------------------------
--  take_puck_to.lua
--
--  Copyright       2012  Victor Mataré
--             2013-2015  The Carologistics Team
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
name               = "take_puck_to"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "relgoto", "fetch_puck", "ppgoto", "search_lost_puck" }
depends_interfaces = {
   { v = "sensor", type = "RobotinoSensorInterface", id = "Robotino" },
}

documentation      = [==[Go to target without losing teh puck]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local AVG_LEN = 5
local MAX_RETRIES = 3
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

-- Imports
local pm = require 'puck_loc_module'
local machine_pos = require 'machine_pos_module'
local tf = require 'tf_module'

function lost_puck()
   local val = fsm.vars.avg_val
   local idx = fsm.vars.avg_idx

   val[idx] = sensor:distance(PUCK_SENSOR_INDEX)
   idx = idx + 1
   if idx > AVG_LEN then idx = 1 end

   local sum = 0
   local count = 0
   for i,v in ipairs(val) do
      sum = sum + v
      count = count + 1
   end
   local avg = sum / count
   --printf("moving avg: %f", avg)

   fsm.vars.avg_val = val
   fsm.vars.avg_idx = idx

   if avg > LOSTPUCK_DIST or avg == 0 then return true end
   return false
end


fsm:define_states{ export_to=_M,
   closure = { MAX_RETRIES=MAX_RETRIES,
      sensor=sensor, LOSTPUCK_DIST=LOSTPUCK_DIST, PUCK_SENSOR_INDEX=PUCK_SENSOR_INDEX },
   {"INIT", JumpState},
   {"SKILL_GOTO", SkillJumpState, skills={{ppgoto}}, final_to="FINAL", fail_to="RETRY_GOTO"},
   {"STOP", SkillJumpState, skills={{ppgoto}}, final_to="SKILL_SEARCH_PUCK",
      fail_to="SKILL_SEARCH_PUCK"},
   {"SKILL_SEARCH_PUCK", SkillJumpState, skills={{search_lost_puck}}, final_to="SKILL_FETCH_PUCK",
      fail_to="FAILED"},
   {"SKILL_FETCH_PUCK", SkillJumpState, skills={{fetch_puck}}, final_to="RETRY_GOTO",
      fail_to="FAILED"},
   {"RETRY_GOTO", JumpState}
}

fsm:add_transitions{
   { "INIT", "SKILL_GOTO", cond=true, desc="true" },
   { "SKILL_GOTO", "STOP", cond=lost_puck, desc="Lost puck" },
   { "RETRY_GOTO", "SKILL_GOTO", cond="vars.goto_retries <= MAX_RETRIES", desc="Retry goto" },
   { "RETRY_GOTO", "FAILED", cond="vars.goto_retries > MAX_RETRIES", desc="giveup goto" }
}

function INIT:init()
   self.fsm.vars.goto_retries = 0
   self.fsm.vars.avg_idx = 1
   self.fsm.vars.avg_val = {}
end

function RETRY_GOTO:init()
   self.fsm.vars.goto_retries = self.fsm.vars.goto_retries + 1
end

function SKILL_GOTO:init()
   self.skills[1].place = self.fsm.vars.place or self.fsm.vars.goto_name
   self.fsm.vars.avg_idx = 1
   self.fsm.vars.avg_val = {}
end

function STOP:init()
   self.skills[1].stop = true
end

