
----------------------------------------------------------------------------
--  take_puck_to.lua
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
name               = "take_puck_to"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "relgoto", "fetch_puck", "ppgoto" }
depends_interfaces = {
   { v = "sensor", type = "RobotinoSensorInterface", id = "Robotino" },
   { v = "omnivisionSwitch", type = "SwitchInterface", id = "omnivisionSwitch" },
   { v = "omnipuck", type = "Position3DInterface", id = "OmniPuck1" },
}

documentation      = [==[Go to target without losing teh puck]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local AVG_LEN = 10
local MAX_RETRIES = 3
-- you can find the config value in /cfg/host.yaml
local LOSTPUCK_DIST = config:get_float("/skills/take_puck_to/front_sensor_dist")

-- Imports
local pm = require 'puck_loc_module'
local machine_pos = require 'machine_pos_module'
local tf = require 'tf_module'

function lost_puck()
   local val = fsm.vars.avg_val
   local idx = fsm.vars.avg_idx

   val[idx] = sensor:distance(8)
   idx = idx + 1
   if idx > AVG_LEN then idx = 1 end

   local sum = 0
   local count = 0
   for i,v in ipairs(val) do
      sum = sum + v
      count = count + 1
   end
   local avg = sum / count
--   printf("moving avg: %f", avg)

   fsm.vars.avg_val = val
   fsm.vars.avg_idx = idx

   if avg > LOSTPUCK_DIST or avg == 0 then return true end
   return false
end


fsm:define_states{ export_to=_M,
   closure = { MAX_RETRIES=MAX_RETRIES,
      omnivisionSwitch = omnivisionSwitch, sensor=sensor, LOSTPUCK_DIST=LOSTPUCK_DIST },
   {"INIT", JumpState},
   {"SKILL_GOTO", SkillJumpState, skills={{ppgoto}}, final_to="FINAL", fail_to="RETRY_GOTO"},
   {"STOP", SkillJumpState, skills={{ppgoto}}, final_to="WAIT",
      fail_to="WAIT"},
   {"WAIT", JumpState},
--   {"RELSTOP", SkillJumpState, skills={{relgoto}}, final_to="WAIT2",
--      fail_to="WAIT2"},
--   {"WAIT2", JumpState},
   {"SKILL_FETCH_PUCK", SkillJumpState, skills={{fetch_puck}}, final_to="RETRY_GOTO",
      fail_to="FAILED"},
   {"RETRY_GOTO", JumpState}
}

fsm:add_transitions{
   { "INIT", "SKILL_GOTO", cond=true, desc="OK" },
   { "SKILL_GOTO", "FAILED", cond="not omnivisionSwitch:has_writer()",
      precond=true, desc="No omnivision writer" },
   { "SKILL_GOTO", "STOP", cond="sensor:distance(8) > LOSTPUCK_DIST", desc="Lost puck" },
   { "WAIT", "SKILL_FETCH_PUCK", timeout=3 },
--   { "WAIT2", "SKILL_FETCH_PUCK", timeout=2 },
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

--function RELSTOP:init()
--   self.skills[1].x = 0
--   self.skills[1].y = 0
--   self.skills[1].ori = 0
--end

