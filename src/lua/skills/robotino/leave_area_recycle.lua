
----------------------------------------------------------------------------
--  leave_area_recycle.lua
--
--  Created: Fri Jun 21 13:16:41 2013
--  Copyright  2013 Frederik Zwilling
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
name               = "leave_area_recycle"
fsm                = SkillHSM:new{name=name, start="ROTATE_FIRST", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = nil

documentation      = [==[Leave area of reccle machine with puck by driving left and rotating]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"ROTATE_FIRST", SkillJumpState, skills={{motor_move}}, final_to="DRIVE_LEFT", fail_to="FAILED"},
   {"DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="U_TURN", fail_to="FAILED"},
   {"U_TURN", SkillJumpState, skills={{motor_move}}, final_to="DRIVE_FORWARD", fail_to="FAILED"},
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

function ROTATE_FIRST:init()
   self.skills[1].x=0 
   self.skills[1].y=0 
   self.skills[1].ori=math.pi/4
end
function DRIVE_LEFT:init()
   self.skills[1].x=0 
   self.skills[1].y=0.2 
   self.skills[1].ori=0
end
function U_TURN:init()
   self.skills[1].x=0 
   self.skills[1].y=0 
   self.skills[1].ori=math.pi
end
function DRIVE_FORWARD:init()
   self.skills[1].x=0.1 
   self.skills[1].y=0 
   self.skills[1].ori=0
end
