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
   {v = "tag_5", type = "Position3DInterface"},
   {v = "tag_6", type = "Position3DInterface"},
   {v = "tag_7", type = "Position3DInterface"},
   {v = "tag_8", type = "Position3DInterface"},
   {v = "tag_9", type = "Position3DInterface"},
   {v = "tag_10", type = "Position3DInterface"},
   {v = "tag_11", type = "Position3DInterface"},
   {v = "tag_12", type = "Position3DInterface"},
   {v = "tag_13", type = "Position3DInterface"},
   {v = "tag_14", type = "Position3DInterface"},
   {v = "tag_15", type = "Position3DInterface"},
   {v = "tag_info", type = "TagVisionInterface"},
}

documentation      = [==[Moves the robot that the tag 0 is seen at the given point.
@param tag_id REQUIRED the id of the tag to align to
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

--moving funtions
function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function get_tag_distance(tag)
   -- in the camera frame of reference the forward distance is z(2) and the lateral distance is x(0)
   return math.sqrt((tag:translation(2)*tag:translation(2)) + (tag:translation(0)*tag:translation(0)))
end

function get_tag_visible(tag)
   return tag:visibility_history() > 0
end

function get_tag_with_id(wanted_id)
   --print("wanted_id: " .. tostring(wanted_id))
   local i=1
   -- get the correct id
   for j=0,15 do
      id = tag_info:tag_id(j)
      --print("j: " .. tostring(j) .. " id: " .. tostring(id))
      -- stop when id is found
      if id == wanted_id then
         break
      end
      i = i+1
   end
   local tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }
   --print("i: " .. tostring(i) .. " tag_" .. tostring(i-1))
   return tags[i]
end

-- Condition Functions
-- Check, weather the final position is reached
function tag_reached(self)
   local tag = get_tag_with_id(self.fsm.vars.tag_id)
   -- the forward distance is the z trnaslation in the camera frame of reference
   forward_distance = tag:translation(2)
   -- the lateral distance is the x translation in the camera frame of reference
   lateral_distance = -tag:translation(0)

   return (math.abs(forward_distance-self.fsm.vars.x) < desired_position_margin.x)
      and (math.abs(lateral_distance-self.fsm.vars.y) < desired_position_margin.y)
end

-- Check if one tag is visible
function tag_not_visible(self)
    local tag = get_tag_with_id(self.fsm.vars.tag_id)
    return (tag:visibility_history() <= 0)
end

-- Check if input is not valid
function input_invalid(self)
	return self.fsm.vars.x < min_distance
end

-- check for motor writer
function no_motor_writer(self)
	return not motor:has_writer()
end

-- check weather the wanted tag is availabel
function id_not_found(self)
   local found = false
   local wanted_id = self.fsm.vars.tag_id
   for i=0,15 do
      id=tag_info:tag_id(i)
      --print("tag nr: " .. i .. " id: " .. id .. " wanted id: " .. wanted_id)
      if id == wanted_id then
         found = true
         break
      end
   end
   --print("found: " ..tostring(found))
   return not found
end

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"DRIVE", JumpState},
   {"ORIENTATE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=id_not_found, desc="Tag with the wanted id is not visible"},
   {"INIT", "FAILED", cond=no_motor_writer, desc="No writer for the motor"},
   {"DRIVE", "FAILED", cond=tag_not_visible, desc="Tag not visible"},
   {"INIT", "FAILED", cond=input_invalid, desc="Distance to tag is garbage, sould be > than " .. min_distance},
   {"INIT", "DRIVE", cond=true, desc="start"},
   {"DRIVE", "ORIENTATE", cond=tag_reached, desc="Tag Reached orientate"},
}

local old_speed={x=0,y=0,ori=0}

function INIT:init()
   self.fsm.vars.x = self.fsm.vars.x or 0.1
   self.fsm.vars.y = self.fsm.vars.y or 0.0
   self.fsm.vars.ori = self.fsm.vars.ori or 0.0
   self.fsm.vars.tag_id = self.fsm.vars.tag_id or -1
   old_speed={x=0,y=0,ori=0}
end

-- Drive to tag
function DRIVE:init()
end

function printtable(table)
   for k,v in pairs(table) do
      print(k .. "\t\t= " .. v)
      if(k == "ori") then
         print("degree\t= " .. v*180/math.pi)
      end
   end
end

function get_yaw(qx, qy, qz, qw)
   return math.atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz)
end

function DRIVE:loop()
   local tag = get_tag_with_id(self.fsm.vars.tag_id)

   local yaw = get_yaw(tag:rotation(0), tag:rotation(1), tag:rotation(2), tag:rotation(3))
   -- correct rotation
   yaw = yaw - (math.pi/2)
   -- the forward distance is the z trnaslation in the camera frame of reference
   forward_distance = tag:translation(2)
   -- the lateral distance is the x translation in the camera frame of reference
   lateral_distance = -tag:translation(0)

   --skip on empty values
   if(forward_distance == 0 and lateral_distance == 0 and yaw == 0) then
--      send_transrot(0,0,0)
      return
   end
	--get the distance to drive
    distance = { x = forward_distance ,--- self.fsm.vars.x,
                y = lateral_distance ,--- self.fsm.vars.y,
                ori = -yaw}
   --print("original distance")
   --printtable(distance)
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
