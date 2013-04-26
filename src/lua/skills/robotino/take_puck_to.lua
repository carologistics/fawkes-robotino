
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
depends_skills     = { "goto", "relgoto", "motor_move", "ppgoto" }
depends_interfaces = {
   { v = "sensor", type = "RobotinoSensorInterface", id = "Robotino" },
   { v = "omnivisionSwitch", type = "SwitchInterface", id = "omnivisionSwitch" },
   { v = "omnipuck1", type = "Position3DInterface", id = "OmniPuck1" },
   { v = "omnipuck2", type = "Position3DInterface", id = "OmniPuck2" },
   { v = "omnipuck3", type = "Position3DInterface", id = "OmniPuck3" },
   { v = "omnipuck4", type = "Position3DInterface", id = "OmniPuck4" },
   { v = "omnipuck5", type = "Position3DInterface", id = "OmniPuck5" },
}

documentation      = [==[Go to target without losing teh puck]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local AVG_LEN = 10
local FIND_TIMEOUT = 30
local TIMEOUT_MOVE_BACK = 10
local TIMEOUT_MOVE_LEFT = 20
local ORI_OFFSET = 0.15
local MAX_RETRIES = 3
local LOSTPUCK_DIST = 0.05
local omnipucks = { omnipuck1, omnipuck2, omnipuck3, omnipuck4, omnipuck5 }

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

function find_best_puck()
   local p_rel, p_loc, d
   local min_d = 1000

   -- iterate over omnipuck interfaces
   for i,op in ipairs(omnipucks) do
      p_rel = pm.get_puck_loc(op)

      -- omnipuck iface op found a puck with a high visibility history
      if p_rel then
         p_loc = tf.transform(
            {
               x = p_rel.x,
               y = p_rel.y,
               ori = math.atan2(p_rel.y, p_rel.x)
            }, "/base_link", "/map")

         if p_loc.x == 0 or p_loc.y == 0 then break end

         for k,field in pairs(machine_pos.fields) do
            if not ( (p_loc.x > field.x and p_loc.x < field.x + machine_pos.field_size)
             and (p_loc.y > field.y and p_loc.y < field.y + machine_pos.field_size) ) then
                -- puck is not in a machine field, so we're done
               fsm.vars.puck_loc = p_loc
               fsm.vars.puck_rel = p_rel
               return true
            end
         end
      end
   end

   return false
end

function no_puck_found()
   return (not find_best_puck()) and (os.time() - fsm.vars.start_time > FIND_TIMEOUT/2)
end

function no_writer()
   return not omnivisionSwitch:has_writer()
end

function best_puck_in_front()
   if not find_best_puck() then
      return false
   end
   if math.abs(math.atan2(fsm.vars.puck_rel.y, fsm.vars.puck_rel.x)) < ORI_OFFSET then
      return true
   end
   return false
end

function best_puck_not_in_front()
   return not best_puck_in_front()
end

function timeout()
   return best_puck_not_in_front() and (os.time() - fsm.vars.start_time > FIND_TIMEOUT)
end

function timeout_move_left()
   return no_puck_found() and os.time() - fsm.vars.start_time > TIMEOUT_MOVE_BACK
end

function timeout_move_right()
   return no_puck_found() and timeout_move_left() and os.time() - fsm.vars.start_time > TIMEOUT_MOVE_LEFT
end

function retry_goto()
   return fsm.vars.goto_retries < MAX_RETRIES
end


fsm:define_states{ export_to=_M,
   closure = { lost_puck = lost_puck },
   {"INIT", JumpState},
   {"SKILL_GOTO", SkillJumpState, skills={{ppgoto}}, final_to="FINAL", fail_to="RETRY_GOTO"},
   {"CLEANFAIL", JumpState},
   {"LOST_PUCK", SkillJumpState, skills={{ppgoto}}, final_to="TURN_ON_OMNIVISION",
      fail_to="TURN_ON_OMNIVISION"},
   {"RETRY_GOTO", JumpState},
   {"TURN_ON_OMNIVISION", JumpState},
   {"LOCATE_PUCK", JumpState},
   {"WAIT_FOR_VISION", JumpState},
   {"TURN_TO_PUCK", SkillJumpState, skills={{relgoto}}, final_to="VERIFY_TURN",
      fail_to="CLEANFAIL" },
   {"VERIFY_TURN", JumpState},
   {"SKILL_FETCH_PUCK", SkillJumpState, skills={{motor_move}}, final_to="SKILL_GOTO",
      fail_to="CLEANFAIL"}
}

fsm:add_transitions{
   { "INIT", "SKILL_GOTO", cond=true },
   { "SKILL_GOTO", "CLEANFAIL", cond=lost_puck, precond=true, desc="Called without puck" },
   { "SKILL_GOTO", "CLEANFAIL", cond=no_writer, precond=true, desc="No omnivision writer" },
   { "SKILL_GOTO", "LOST_PUCK", cond=lost_puck, desc="Lost puck" },
   { "RETRY_GOTO", "SKILL_GOTO", cond=retry_goto, desc="Retry goto" },
   { "RETRY_GOTO", "CLEANFAIL", cond="not retry_goto", desc="giveup goto" },
   { "TURN_ON_OMNIVISION", "LOCATE_PUCK", timeout=1, desc="Wait for Omnivision" },
   { "LOCATE_PUCK", "SKILL_GOTO", cond="not lost_puck()" },
   { "LOCATE_PUCK", "CLEANFAIL", cond=timeout_move_left, desc="move left a bit" },
   { "LOCATE_PUCK", "WAIT_FOR_VISION", cond=no_puck_found, desc="No valid Puck found" },
   { "LOCATE_PUCK", "TURN_TO_PUCK", cond=find_best_puck, desc="Found lost puck" },
--   { "MOVE_BACK", "LOCATE_PUCK", skill=relgoto, fail_to="MOVE_LEFT" },
   { "WAIT_FOR_VISION", "LOCATE_PUCK", timeout=0.33333, desc="Wait, retry" },
   { "VERIFY_TURN", "LOCATE_PUCK", cond=best_puck_not_in_front },
   { "VERIFY_TURN", "SKILL_FETCH_PUCK", cond=best_puck_in_front },
   { "VERIFY_TURN", "CLEANFAIL", cond=timeout, desc="Couldn't recover: Timeout" },
   { "CLEANFAIL", "FAILED", true }
}

function CLEANFAIL:init()
   local msg = omnivisionSwitch.DisableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)
end

function INIT:init()
   self.fsm.vars.goto_retries = 0
   self.fsm.vars.avg_idx = 1
   self.fsm.vars.avg_val = {}
end

--function MOVE_BACK:init()
--   self.args = { rel_x = -0.1 }
--end

function RETRY_GOTO:init()
   self.fsm.vars.goto_retries = self.fsm.vars.goto_retries + 1
end

function SKILL_GOTO:init()
   self.skills[1].place = self.fsm.vars.place or self.fsm.vars.goto_name
   self.fsm.vars.avg_idx = 1
   self.fsm.vars.avg_val = {}
end

function LOST_PUCK:init()
   self.skills[1].stop = true
end

function TURN_ON_OMNIVISION:init()
   -- turn on omnivision
   local msg = omnivisionSwitch.EnableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)
   -- remember current time
   self.fsm.vars.start_time = os.time()
end

function TURN_TO_PUCK:init()
   self.skills[1].rel_x = 0 
   self.skills[1].rel_y = 0 
   self.skills[1].rel_ori = math.atan2(self.fsm.vars.puck_rel.y, self.fsm.vars.puck_rel.x)
end

function SKILL_FETCH_PUCK:init()
   -- turn on omnivision
   local msg = omnivisionSwitch.DisableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)
   self.skills[1].x = math.sqrt(self.fsm.vars.puck_rel.x^2 + self.fsm.vars.puck_rel.y^2)
end

