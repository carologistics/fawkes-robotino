
----------------------------------------------------------------------------
--  leave_area.lua - generic global goto
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License,  or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful, 
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

-- Initialize module
module(...,  skillenv.module_init)

-- Crucial skill information
name               = "determine_signal"
fsm                = SkillHSM:new{name=name,  start="INIT",  debug=false}
depends_skills     = { "motor_move" }
depends_interfaces = {
   { v="plugin", type="RobotinoLightInterface", id="Light_State" },
   { v="output", type="RobotinoLightInterface", id="determined", writing=true }   
}


documentation      = [==[
writes ampel data into the light interface 
]==]

-- Constants
local MIN_VIS_HIST = 4
local TIMEOUT = 10
local MOVES = { 0.08, -0.15, 0.32, -0.5 }

-- Initialize as skill module
skillenv.skill_module(_M)

function plugin_sure()
   return plugin:visibility_history() >= MIN_VIS_HIST
end

fsm:define_states{ export_to=_M,
   closure={plugin=plugin, MOVES=MOVES},
   {"INIT", JumpState},
   {"DETERMINE", JumpState},
   {"MOVE", SkillJumpState, skills={{motor_move}}, final_to="DETERMINE", fail_to="DETERMINE"}
}

fsm:add_transitions{
   {"INIT", "DETERMINE", timeout=1}, -- let vision settle
   {"DETERMINE", "FINAL", cond=plugin_sure},
   {"DETERMINE", "FAILED", cond="vars.move_idx > #MOVES"},
   {"DETERMINE", "MOVE", timeout=TIMEOUT}
}

function INIT:init()
   self.fsm.vars.move_idx = 1
end

function MOVE:init()
   self.skills[1].y = MOVES[self.fsm.vars.move_idx]
   self.fsm.vars.move_idx = self.fsm.vars.move_idx + 1
end

function FINAL:init()
   output:set_red(plugin:red())
   output:set_yellow(plugin:yellow())
   output:set_green(plugin:green())
   self.fsm.determine_done = true
end

