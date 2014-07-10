
----------------------------------------------------------------------------
--  get_rid_of_puck.lua - escapes an accidentally picked up puck
--
--  Created: Thu 10. Jul 12:45:34 CEST 2014
--  Copyright  2014 Matthias Loebach
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
name               = "get_rid_of_puck"
fsm                = SkillHSM:new{name=name, start="GET_RID_OF_PUCK", debug=false}
depends_skills     = {""}
depends_interfaces = nil


documentation      = [==[
escape puck we don't want to have
]==]
-- Constants


-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"GET_RID_OF_PUCK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

function GET_RID_OF_PUCK:init()
   self.skills[1].x = -0.2
   self.skills[1].ori = 0.5
   self.skills[1].vel_trans = 0.8
end


