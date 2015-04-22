
----------------------------------------------------------------------------
--  explore_signal.lua - Position and detect signal
--
--  Copyright  2015 Victor Mataré
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
name               = "explore_signal"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"mps_align", "mps_detect_signal", "drive_to" }
depends_interfaces = {
}

documentation      = [==[Detect signal at a certain MPS
]==]

local MAX_TRIES = 3
local ALIGN_POS = {x=0.5, y=0, ori=0}

-- Initialize as skill module
skillenv.skill_module(_M)

-- States
fsm:define_states{
   export_to=_M,
   closure={MAX_TRIES=MAX_TRIES},
   {"INIT", JumpState},
   {"SKILL_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="SKILL_DETECT", fail_to="DECIDE_RETRY"},
   {"DECIDE_RETRY", JumpState},
   {"SKILL_DETECT", SkillJumpState, skills={{mps_detect_signal}}, final_to="FINAL", fail_to="DECIDE_RETRY"},
   {"REPOSITION", SkillJumpState, skills={{drive_to}}, final_to="SKILL_ALIGN", fail_to="DECIDE_RETRY"}
}

-- Transitions
fsm:add_transitions{
   {"INIT", "SKILL_ALIGN", cond=true},
   {"DECIDE_RETRY", "SKILL_ALIGN", cond="vars.tries <= MAX_TRIES"}, -- REPOSITION ??
   {"DECIDE_RETRY", "FAILED", cond="vars.tries > MAX_TRIES"}
}

function INIT:init()
   self.fsm.vars.tries = 0
end

function SKILL_ALIGN:init()
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.skills[1].x = ALIGN_POS.x
   self.skills[1].y = ALIGN_POS.y
   self.skills[1].ori = ALIGN_POS.ori
end

function SKILL_DETECT:init()
   self.skills[1].place = self.fsm.vars.place
end
