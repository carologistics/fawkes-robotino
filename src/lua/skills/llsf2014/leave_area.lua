
----------------------------------------------------------------------------
--  leave_area.lua - generic global goto
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
name               = "leave_area"
fsm                = SkillHSM:new{name=name, start="LEAVE_AREA", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = nil


documentation      = [==[
leave area without puck
]==]
-- Constants


-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"LEAVE_AREA", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

function LEAVE_AREA:init()
   self.skills[1].x=-0.2 
   self.skills[1].y=0 
   self.skills[1].ori=0
end



