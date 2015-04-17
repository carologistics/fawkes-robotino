
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
name               = "search_lost_puck"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "motor_move" }
depends_interfaces = {
   {v = "puck_0", type = "Position3DInterface", id="puck_0"},
   {v = "puck_1", type = "Position3DInterface", id="puck_1"},
   {v = "puck_2", type = "Position3DInterface", id="puck_2"},
}

documentation      = [==[Turn until a (recently lost) puck is in view]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local MAX_TRIES=2
local pucks = { puck_0, puck_1, puck_2 }

function visible()
   for _,o in ipairs(pucks) do
      if o:visibility_history() >= 1 then
         return true
      end
   end
   return false
end

fsm:define_states{ export_to=_M, closure={visible=visible, MAX_TRIES=MAX_TRIES},
   {"INIT", JumpState},
   {"WAIT_L", JumpState},
   {"TURN_LEFT", SkillJumpState, skills={{motor_move}}, final_to="WAIT_R", fail_to="FAILED"},
   {"WAIT_R", JumpState},
   {"TURN_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="WAIT_L", fail_to="FAILED"},
   {"WAIT_END", JumpState}
}

fsm:add_transitions{
   {"INIT", "WAIT_L", cond=true},
   {"WAIT_L", "TURN_LEFT", timeout=1},
   {"WAIT_L", "WAIT_END", cond=visible},
   {"WAIT_L", "FAILED", cond="vars.tries >= vars.max_tries"},
   {"TURN_LEFT", "WAIT_END", cond=visible},
   {"WAIT_R", "TURN_RIGHT", timeout=1},
   {"WAIT_R", "WAIT_END", cond=visible},
   {"WAIT_R", "FAILED", cond="vars.tries >= vars.max_tries"},
   {"TURN_RIGHT", "WAIT_END", cond=visible},
   {"WAIT_END", "FINAL", timeout=1.5}
}

function INIT:init()
   self.fsm.vars.tries = 0
   self.fsm.vars.max_tries = self.fsm.vars.max_tries or MAX_TRIES
end

function TURN_LEFT:init()
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.skills[1].ori = 0.2 * self.fsm.vars.tries * math.pi
   self.skills[1].vel_rot = 0.4
end

function TURN_RIGHT:init()
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.skills[1].ori = -0.2 * self.fsm.vars.tries * math.pi
   self.skills[1].vel_rot = 0.4
end
