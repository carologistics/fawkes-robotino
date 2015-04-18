----------------------------------------------------------------------------
--  goto_place.lua
--
--  Created: Sat Jun 14 15:13:19 2014
--  Copyright  2015 Tobias Neumann
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
name               = "goto_place"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"goto"}
depends_interfaces = { }

documentation      = [==[Drives into field after given offset

Parameters:
   place: navgraph point to drive to (without pathplaner)
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"GOTO", SkillJumpState, skills={{goto}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",   "FAILED", cond="not navgraph", desc="no navgraph"},
   {"INIT",   "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT",   "GOTO",   cond=true},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function GOTO:init()
   self.skills[1].x     = self.fsm.vars.node:x()
   self.skills[1].y     = self.fsm.vars.node:y()
   if self.fsm.vars.node:has_property("orientation") then
      self.skills[1].ori   = self.fsm.vars.node:property_as_float("orientation");
   end
end
