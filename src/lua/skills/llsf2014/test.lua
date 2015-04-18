
----------------------------------------------------------------------------
--  goto.lua - 
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré
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
name               = "test"
fsm                = SkillHSM:new{name=name, start="GO"}
depends_skills     = { "goto" }

documentation      = [==[MWE timeout bug]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"GO", SkillJumpState, skills={{goto}}, final_to="FINAL", fail_to="FAILED"},
}


fsm:add_transitions{
   {"GO", "FINAL", timeout=0.5}
}

function GO:init()
   self.skills[1].x = 4
   self.skills[1].y = 4
end

