
----------------------------------------------------------------------------
--  leave_area_with_puck.lua
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
name               = "leave_area_with_puck"
fsm                = SkillHSM:new{name=name, start="DRIVE_LEFT", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {

}

documentation      = [==[Leaves area with puck by driving left and rotating]==]

-- Initialize as skill module
skillenv.skill_module(...)


fsm:add_transitions{
	closure={motor=motor},
	{"DRIVE_LEFT", "ROTATE", skill=motor_move, fail_to="FAILED"},
	{"ROTATE", "FINAL", skill=motor_move, fail_to="FAILED"}
}

function DRIVE_LEFT:init()
	self.args = {x=0,y=0.2,ori=0}
end
function ROTATE:init()
	self.args = {x=0, y=0, ori=math.pi}
end

