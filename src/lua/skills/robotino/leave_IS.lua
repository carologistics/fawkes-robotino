
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
name               = "leave_IS"
fsm                = SkillHSM:new{name=name, start="CHECK_TURN", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
	{v="pose", type="Position3DInterface",id = "Pose"}
}

documentation      = [==[Leaves area with puck by driving left and rotating]==]

-- Initialize as skill module
skillenv.skill_module(...)

local m_pos = require("machine_pos_module")

function get_ori_diff()
	local ori = 2*math.acos(pose:rotation(3)) 
	local is_ori = m_pos.delivery_goto.Is.ori
	local diff = 0
	if ori > is_ori then 
      		if ori - is_ori < math.pi then
			diff =  ori - is_ori 
		else
			diff =  - 2.0 * math.pi + ori - is_ori
		end
  	else
		if is_ori - ori < math.pi then
			diff = ori - is_ori 
		else
			diff = 2.0 * math.pi - is_ori + ori;
    		end
	end
	return diff
end
function oriented_left()
	if get_ori_diff() >= 0 then
		return true
	end
end
function oriented_right()
	if get_ori_diff() < 0 then
		return true
	end
end
function needs_turn()
	if math.abs(get_ori_diff()) < 1.05 then
		return true
	end
end
function needs_no_turn()
	return not needs_turn()
end

fsm:add_transitions{
	closure={motor=motor},
	{"CHECK_TURN", "TURN", cond = needs_turn},
	{"CHECK_TURN", "SKILL_MOTOR_MOVE", cond = needs_no_turn},
	{"TURN", "SKILL_MOTOR_MOVE",skill=motor_move, fail_to="FAILED"},
	{"SKILL_MOTOR_MOVE", "FINAL", skill=motor_move, fail_to="FAILED"}
}

function CHECK_TURN:init()
end
function TURN:init()
	--1.05 = 60°
	if oriented_left() then
		self.args = {x=0, y=0, ori=1.05-math.abs(get_ori_diff())}
	else
		self.args = {x=0, y=0, ori=-1.05+math.abs(get_ori_diff())}
	end
	print(get_ori_diff())
end
function SKILL_MOTOR_MOVE:init()
	if oriented_left() then
		self.args = {x=0, y=0.5, ori=0}
	else
		self.args = {x=0, y=-0.5, ori=0}
	end
	
end

