
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
fsm                = SkillHSM:new{name=name, start="GOTO_IS", debug=false}
depends_skills     = {"goto","fetch_puck","leave_IS"}
depends_interfaces = {
   
}

documentation      = [==[test der bisherigen skills später eigener skill mit puck_aufname_location und puck_abgabe_location]==]
--constants
local start = "Is"
-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"GOTO_IS", SkillJumpState, skills=goto, final_to="SKILL_FETCH_PUCK", fail_to="FAILED"},
   {"SKILL_FETCH_PUCK", SkillJumpState, skills=fetch_puck, final_to="SKILL_LEAVE_AREA",
      fail_to="FAILED"},
   {"SKILL_LEAVE_AREA", SkillJumpState, skills=leave_IS, final_to="FINAL", fail_to="FAILED"}
}

function GOTO_IS:init()
   self.args = {goto_name=start}
end

