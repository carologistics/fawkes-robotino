
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
name               = "deposit_puck"
fsm                = SkillHSM:new{name=name, start="DESC_DIRECTION", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
   -- TODO {v = "worldModel", type="WorldModel"}
}

documentation      = [==[deposits the used puck at the side of the traffic light]==]

-- Initialize as skill module
skillenv.skill_module(...)

function no_puck()
	--return worldModel:numberOfPucks=0
	return false
end


function one_left()
	--pucksLeft = worldModel:getNumberOfPucks(right,machine)
	--return (pucksLeft == 1)
	return true
end

fsm:add_transitions{
   closure={motor=motor},
   {"DESC_DIRECTION", "SKILL_DRIVE_LEFT", cond=no_puck, desc="There is no Puck in the storage"},
   {"DESC_DIRECTION", "SKILL_DRIVE_RIGHT", cond=one_left},
   {"SKILL_DRIVE_LEFT", "SKILL_DRIVE_FORWARD", skill=motor_move, fail_to="FAILED"},
   {"SKILL_DRIVE_RIGHT", "SKILL_DRIVE_FORWARD", skill=motor_move, fail_to="FAILED"},
   {"SKILL_DRIVE_FORWARD", "SKILL_DRIVE_BACKWARD", skill=motor_move, fail_to="FAILED"},
   {"SKILL_DRIVE_BACKWARD", "FINAL", skill=motor_move, fail_to="FAILED"}
}

function DESC_DIRECTION:init()
	
end

function SKILL_DRIVE_LEFT:init()
	self.args = {x=0,y=0.18,ori=0}
end

function SKILL_DRIVE_RIGHT:init()
	self.args = {x=0,y=-0.18,ori=0}
end

function SKILL_DRIVE_FORWARD:init()
	self.args = {x=0.2,y=0,ori=0}
end

function SKILL_DRIVE_BACKWARD:init()
	self.args = {x=-0.2,y=0,ori=0}
 --TODO UPDATE WORLD MODEL WENN FINAL
end

