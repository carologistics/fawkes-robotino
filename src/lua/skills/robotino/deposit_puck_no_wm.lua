
----------------------------------------------------------------------------
--  goto.lua - generic global goto
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
name               = "deposit_puck_no_wm"
fsm                = SkillHSM:new{name=name, start="DESC_DIRECTION", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
}

documentation      = [==[
			 deposits the used puck at the side of the traffic light
			 Needed Variable is called "machine, and must be a number
			 between 1 and 10, where the number is the machine you're
			 standing in front of
		     ]==]

-- Initialize as skill module
skillenv.skill_module(...)


fsm:add_transitions{
   closure={motor=motor},	
   {"SKILL_DRIVE_LEFT", "SKILL_DRIVE_FORWARD", skill=motor_move, fail_to="FAILED"},
   {"SKILL_DRIVE_FORWARD", "SKILL_DRIVE_BACKWARD", skill=motor_move, fail_to="FAILED"},
   {"SKILL_DRIVE_BACKWARD", "FINAL", skill=motor_move, fail_to="FAILED"},
}

function SKILL_DRIVE_LEFT:init()
	self.args = {x=0,y=0.28,ori=0}
end

function SKILL_DRIVE_FORWARD:init()
	self.args = {x=0.05,y=0,ori=0}
end

function SKILL_DRIVE_BACKWARD:init()
	self.args = {x=-0.2,y=0,ori=0}
end

