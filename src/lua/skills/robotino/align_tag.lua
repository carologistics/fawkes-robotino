----------------------------------------------------------------------------
--  align_tag.lua - stupidly move to some odometry position
--
--  Copyright  2013 The Carologistics Team
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
name               = "align_tag"
fsm                = SkillHSM:new{name=name, start="DRIVE"}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "tag_0", type = "Position3DInterface"}
}

documentation      = [==[Moves the robot that the tag 0 is seen at the given point.
@param x The X distance to the tag
@param y The Y distance to the tag
@param ori The orientation to the tag
]==]

-- Constants
local min_distance = 0.1
local desired_position_margin = 0.02

-- Condition Functions
-- Check, weather the final position is reached
function tag_reached(self)
	return (math.abs(tag_0:translation(0)-self.fsm.vars.x) < desired_position_margin)
	   and (math.abs(tag_0:translation(1)-self.fsm.vars.y) < desired_position_margin)
end

-- Check if one tag is visible
function tag_not_visible(self)
	return (tag_0:visibility_history() == 0)
end

-- Check if input is not valid
function input_invalid(self)
	return self.fsm.vars.x < min_distance
end

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
	{"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="DRIVE", fail_to="FAILED"},
	{"ORIENTATE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
	{"DRIVE", "FAILED", cond=tag_not_visible, desc="No tag visible"},
	{"DRIVE", "FAILED", cond=input_invalid, desc="Distance to tag is garbage, sould be > than " .. min_distance},
	{"DRIVE", "ORIENTATE", cond=tag_reached, desc="Tag Reached orientate"}
}

-- Drive to tag
function DRIVE:init()
   --handle nil values
   if(self.fsm.vars.x == nil) then
      slef.fsm.vars.x = 0.1
   end
   if(self.fsm.vars.y == nil) then
      self.fsm.vars.y = 0
   end

	-- get distance and rotation from tag vision
	local found_x = tag_0:translation(0)
	local found_y = tag_0:translation(1)
	local found_ori = -tag_0:rotation(0)
	-- calculate transition -> to approach the tag
	local delta_x = found_x - (self.fsm.vars.x * math.cos(found_ori) + self.fsm.vars.y * (-1 * math.sin(found_ori)))
	local delta_y = found_y - (self.fsm.vars.x * math.sin(found_ori) + self.fsm.vars.y * math.cos(found_ori))
	-- enlarge found values to activate the motor
	if math.abs(delta_x) < 0.04 and math.abs(delta_y) < 0.04 then
		delta_x = delta_x * 2 
		delta_y = delta_y * 2 
		--print("enlarging")
	end 
	-- debug printing
	--print("found_x = " .. found_x)
	--print("delta_x = " .. delta_x)
	--print("found_y = " .. found_y)
	--print("delta_y = " .. delta_y)
	--print("ori  = " .. found_ori)
	
	-- move to tag alignment -> call motor_move
	self.skills[1].x = delta_x
	self.skills[1].y = delta_y
	self.skills[1].ori = found_ori
end

function ORIENTATE:init()
	self.skills[1].ori = self.fsm.vars.ori
end
