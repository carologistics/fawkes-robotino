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
local max_velocity = { x = 0.4, y = 0.4, ori = 1.8 } -- maximum motor can do
local min_velocity = { x = 0.035, y = 0.035, ori = 0.15 } --minimum to start motor
local acceleration = { x = 0.05, y = 0.05, ori = o.10 } --accelleration per loop
local deceleration_distance = { x = 0.07, y = 0.07, ori = 0.2 } --decelerate when distance is closer
local cycle = 0

-- Variables
local target = { x = 0 , y = 0 , ori = 0}

--moving funtions
function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end


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

-- check for motor writer
function no_motor_writer(self)
	return not motor:has_writer()
end

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
	{"DRIVE", JumpState},
	{"ORIENTATE", SkillJumpState, skills={{motor_move}}, final_to="FINISHED", fail_to="FAILED"},
}

fsm:add_transitions{
	{"DRIVE", "FAILED", cond=no_motor_writer, desc="No writer for the motor"},
	{"DRIVE", "FAILED", cond=tag_not_visible, desc="No tag visible"},
	{"DRIVE", "FAILED", cond=input_invalid, desc="Distance to tag is garbage, sould be > than " .. min_distance},
	{"DRIVE", "ORIENTATE", cond=tag_reached, desc="Tag Reached orientate"},
}

-- Drive to tag
function DRIVE:init()
	--handle nil values
	local x = self.fsm.vars.x or 0.1
	local y = self.fsm.vars.y or 0

	-- get distance and rotation from tag vision
	local found_x = tag_0:translation(0)
	local found_y = tag_0:translation(1)
	local found_ori = -tag_0:rotation(0)
	-- calculate transition -> to approach the tag
	local delta_x = found_x - (x * math.cos(found_ori) + y * (-1 * math.sin(found_ori)))
	local delta_y = found_y - (x * math.sin(found_ori) + y * math.cos(found_ori))
	-- move to tag alignment -> call motor_move
	target.x = delta_x
	target.y = delta_y
	target.ori = found_ori
	cycle = 0
end


function DRIVE:loop()
	--get the distance to drive
	distance ={ x = tag_0:translation(0) - (self.fsm.vars.x or 0.1),
				y = tag_0:translation(1) - (self.fsm.vars.y or 0.0),
				ori = tag_0:rotations(0)}
	--get a good velocity
	velocity = { x = 1, y = 1, ori = 1}
	accelleration = { x = 0, y = 0, ori = 0}
	-- for x y ori in distance to target
	for key,value in pairs(distance) do
		if deceleration_distance[k] > 0 then accelleration[k] = max_velocity[k] / deceleration_distance[k] end
		--get acceleration for current cycle
		accel_veloc = cycle * accelleration[k]
		--get how much to deceletate
		decel_veloc = a[k]/5 * math.abs(distance[k])
		--decide what to send, max acceleration or deceleration
		velocity[k] = math.min( max_velocity[k],
			math.max(min_velocity[k], math.min(max_velocity[k], accel_veloc, decel,velocity))
		)
	end
	cycle = cycle + 1
	send_transrot(velocity.x, velocity.y, velocity.ori)
	--send motor message
end

function ORIENTATE:init()
	self.skills[1].ori = self.fsm.vars.ori
end
