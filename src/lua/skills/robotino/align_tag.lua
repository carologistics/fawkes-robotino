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
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "tag_0", type = "Position3DInterface"},
   {v = "tag_1", type = "Position3DInterface"},
   {v = "tag_2", type = "Position3DInterface"},
   {v = "tag_3", type = "Position3DInterface"},
   {v = "tag_4", type = "Position3DInterface"},
}

documentation      = [==[Moves the robot that the tag 0 is seen at the given point.
@param x The X distance to the tag
@param y The Y distance to the tag
@param ori The orientation to the tag
]==]

-- Constants
local min_distance = 0.1
local desired_position_margin = {x=0.005, y=0.005, ori=0.01}
local min_velocity = { x = 0.015, y = 0.015, ori = 0.05 } --minimum to start motor
local max_velocity = { x = 0.4, y = 0.4 , ori = 0.4} -- maximum, full motor

-- Variables
local target = { x = 0 , y = 0 , ori = 0}
local tries

--moving funtions
function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function get_tag_distance(tag)
   return math.sqrt((tag:translation(0)*tag:translation(0)) + (tag:translation(1)*tag:translation(1)))
end

function get_tag_visible(tag)
   return tag:visibility_history() > 0
end


function get_closest_tag()
   -- begin with tag 0
   local retval = tag_0
   local closest_distance = get_tag_distance(tag_0)
   --print("tag_0 at distance " .. tostring(closest_distance))
   -- tag 1
   if(get_tag_visible(tag_1) and get_tag_distance(tag_1) < closest_distance) then
      retval = tag_1
	  closest_distance = get_tag_distance(tag_1)
      --print("tag_1 visible and closer at " .. tostring(closest_distance))
   end
   -- tag 2
   if(get_tag_visible(tag_2) and get_tag_distance(tag_2) < closest_distance) then
      retval = tag_2
	  closest_distance = get_tag_distance(tag_2)
      -print("tag_2 visible and closer at " .. tostring(closest_distance))
   end
   -- tag 3
   if(get_tag_visible(tag_3) and get_tag_distance(tag_3) < closest_distance) then
      retval = tag_3
	  closest_distance = get_tag_distance(tag_3)
      --print("tag_3 visible and closer at " .. tostring(closest_distance))
   end
   -- tag 4
   if(get_tag_visible(tag_4) and get_tag_distance(tag_4) < closest_distance) then
      retval = tag_4
	  closest_distance = get_tag_distance(tag_4)
     -- print("tag_4 visible and closer at " .. tostring(closest_distance))
   end

   --print("closest distance is " .. tostring(closest_distance))

   return retval
end

-- Condition Functions
-- Check, weather the final position is reached
function tag_reached(self)
	local tag = get_closest_tag()
	return (math.abs(tag:translation(0)-self.fsm.vars.x) < desired_position_margin.x)
	   and (math.abs(tag:translation(1)-self.fsm.vars.y) < desired_position_margin.y)
end

-- Check if one tag is visible
function tag_not_visible(self)
	local tag = get_closest_tag()
	return (tag:visibility_history() == 0)
end

-- Check if input is not valid
function input_invalid(self)
	return self.fsm.vars.x < min_distance
end

-- check for motor writer
function no_motor_writer(self)
	return not motor:has_writer()
end

function too_many_tries()
   return tries > 9
end

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
	{"DRIVE", JumpState},
   {"NO_TAG", JumpState},
	{"ORIENTATE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
	{"INIT", "FAILED", cond=no_motor_writer, desc="No writer for the motor"},
	{"DRIVE", "NO_TAG", cond=tag_not_visible, desc="No tag visible"},
   {"NO_TAG", "DRIVE", timeout=0.5, desc="try again"},
   {"DRIVE", "FAILED", cond=too_many_tries, desc="tag really not seen"},
	{"INIT", "FAILED", cond=input_invalid, desc="Distance to tag is garbage, sould be > than " .. min_distance},
   {"INIT", "DRIVE", cond=true, desc="start"},
	{"DRIVE", "ORIENTATE", cond=tag_reached, desc="Tag Reached orientate"},
}

function INIT:init()
   self.fsm.vars.x = self.fsm.vars.x or 0.1
   self.fsm.vars.y = self.fsm.vars.y or 0.0
   self.fsm.vars.ori = self.fsm.vars.ori or 0.0
   tries = 0
end

-- Drive to tag
function DRIVE:init()
	--handle nil values
	local x = self.fsm.vars.x
	local y = self.fsm.vars.y

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

function NO_TAG:init()
   tires = tries + 1
end

function printtable(table)
   for k,v in pairs(table) do
      print(k .. " = " .. v)
   end
end

local old_speed={x=0,y=0,ori=0}

function DRIVE:loop()
   local tag = get_closest_tag()
   local q = fawkes.tf.Quaternion:new(tag:rotation(0), tag:roatation(1), tag,rotation(2), tag:rotation(3))
   --skip on empty values
   if(tag:translation(0) == 0 and tag:translation(1) == 0 and fawkes.tf.get_yaw(q) == 0) then
--      send_transrot(0,0,0)
      return
   end
	--get the distance to drive
	distance = { x = tag:translation(0) ,--- self.fsm.vars.x,
				y = tag:translation(1) ,--- self.fsm.vars.y,
                ori = -fawkes.tf.get_yaw(q)}
   distance.x = distance.x - (self.fsm.vars.x * math.cos(distance.ori) + self.fsm.vars.y * (-1 * math.sin(distance.ori)))
   distance.y = distance.y - (self.fsm.vars.x * math.sin(distance.ori) + self.fsm.vars.y * math.cos(distance.ori))
   --print("current distance")
   --printtable(distance)
	--get a good velocity
	local velocity = {x = 0, y = 0, ori = 0}
	for key,value in pairs(distance) do
      velocity[key] = math.sqrt(math.abs(distance[key]/2))*max_velocity[key]  --+min_velocity[key]
      if distance[key] < 0 then velocity[key] = velocity[key] *-1 end
      --average
      velocity[key] = (velocity[key]+old_speed[key])/2
      old_speed[key] = velocity[key]
  	end
   --print("velocity:")
   --printtable(velocity)
	--send motor message
	send_transrot(velocity.x, velocity.y, velocity.ori)
end

function ORIENTATE:init()
	self.skills[1].ori = self.fsm.vars.ori
end
