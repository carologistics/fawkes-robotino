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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "tag_0", type = "Position3DInterface", id="/tag-vision/0"},
   {v = "tag_1", type = "Position3DInterface", id="/tag-vision/1"},
   {v = "tag_2", type = "Position3DInterface", id="/tag-vision/2"},
   {v = "tag_3", type = "Position3DInterface", id="/tag-vision/3"},
   {v = "tag_4", type = "Position3DInterface", id="/tag-vision/4"},
   {v = "tag_5", type = "Position3DInterface", id="/tag-vision/5"},
   {v = "tag_6", type = "Position3DInterface", id="/tag-vision/6"},
   {v = "tag_7", type = "Position3DInterface", id="/tag-vision/7"},
   {v = "tag_8", type = "Position3DInterface", id="/tag-vision/8"},
   {v = "tag_9", type = "Position3DInterface", id="/tag-vision/9"},
   {v = "tag_10", type = "Position3DInterface", id="/tag-vision/10"},
   {v = "tag_11", type = "Position3DInterface", id="/tag-vision/11"},
   {v = "tag_12", type = "Position3DInterface", id="/tag-vision/12"},
   {v = "tag_13", type = "Position3DInterface", id="/tag-vision/13"},
   {v = "tag_14", type = "Position3DInterface", id="/tag-vision/14"},
   {v = "tag_15", type = "Position3DInterface", id="/tag-vision/15"},
   {v = "tag_info", type = "TagVisionInterface", id="/tag-vision/info"},
}

documentation      = [==[Moves the robot that the tag 0 is seen at the given point.
@param tag_id the id of the tag to align to, if empty the closest tag is used to align to
@param x The X distance to the tag in the tag frame
@param y The Y distance to the tag in the tag frame, (negative in base_link frame)
@param ori The orientation to the tag in the base link frame
]==]

-- Constants
local min_distance = 0.1
--local desired_position_margin = {x=0.01, y=0.01, ori=0.01}
--local min_velocity = { x = 0.015, y = 0.015, ori = 0.05 } --minimum to start motor
--local max_velocity = { x = 0.4, y = 0.4 , ori = 0.4} -- maximum, full motor
local desired_position_margin = {x=0.05, y=0.03, ori=0.1}
local min_velocity = { x = 0.025, y = 0.025, ori = 0.1 } --minimum to start motor
local max_velocity = { x = 0.6, y = 0.6 , ori = 0.6} -- maximum, full motor
local tfm = require("tf_module")

local TIMEOUT=2

-- Variables
local target = { x = 0 , y = 0 , ori = 0}


function printtable(table)
   for k,v in pairs(table) do
      print(k .. "\t\t= " .. v)
      if(k == "ori") then
         print("degree\t= " .. v*180/math.pi)
      end
   end
end

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

-- Condition Functions
-- Check, weather the final position is reached
function tag_reached(self)
   local distance = tfm.transform({x = self.fsm.vars.x, y = self.fsm.vars.y, ori = math.pi}, fsm.vars.transform_name, "/base_link")
   return distance
      and (math.abs(distance.x) < desired_position_margin.x)
      and (math.abs(distance.y) < desired_position_margin.y)
end

-- Check if one tag is visible
function tag_not_visible()
    local tag = fsm.vars.iface_name
    return (tag and tag:visibility_history() <= 0)
end

-- Check if input is not valid
function input_ok()
	return fsm.vars.x >= min_distance
end

-- check for motor writer
function no_motor_writer(self)
	return not motor:has_writer()
end

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"FIND_TAG", JumpState},
   {"DRIVE", JumpState},
   {"ORIENTATE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   closure={input_ok=input_ok},
   {"INIT", "FIND_TAG", cond=input_ok},
   {"FIND_TAG", "DRIVE", cond="vars.iface_name"},
   {"FIND_TAG", "FAILED", timeout=TIMEOUT},
   {"INIT", "FAILED", cond="not input_ok()"},
   {"INIT", "FAILED", cond=tag_not_visible, desc="Tag not visible"},
   {"INIT", "FAILED", cond=no_motor_writer, desc="No writer for the motor"},
   {"DRIVE", "FAILED", cond=tag_not_visible, desc="Tag not visible"},
   {"DRIVE", "ORIENTATE", cond=tag_reached, desc="Tag Reached orientate"},
}

local old_speed={x=0,y=0,ori=0}

function INIT:init()
   self.fsm.vars.x = self.fsm.vars.x or 0.1
   self.fsm.vars.y = self.fsm.vars.y or 0.0
   self.fsm.vars.ori = self.fsm.vars.ori or 0.0
   old_speed={x=0,y=0,ori=0}
end

function FIND_TAG:loop()
   local i=1
   local tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }
   if not self.fsm.vars.tag_id then
      -- no tag ID argument given
      local closest_distance=9999
      for iter=1,16 do
         local my_tag = tags[iter]
         if (get_tag_distance(my_tag) < closest_distance) and (my_tag:visibility_history() > 0 )then
            closest_distance = get_tag_distance(my_tag)
            self.fsm.vars.transform_name = "/tag_" .. tostring(iter-1)
            self.fsm.vars.iface_name = my_tag
         end
      end
   else
      -- find the blackboard interface that has the ID given by the tag_id arg
      for j=0,15 do
         id = tag_info:tag_id(j)
         --print("j: " .. tostring(j) .. " id: " .. tostring(id))
         -- stop when id is found
         if id == fsm.vars.tag_id then
            self.fsm.vars.transform_name = "/tag_" .. tostring(j)
            self.fsm.vars.iface_name = tags[i]
            break
         end
         i = i+1
      end
      --print("i: " .. tostring(i) .. " tag_" .. tostring(i-1))
   end
end


function DRIVE:loop()

   local tag = self.fsm.vars.iface_name

   local distance = tfm.transform({x = self.fsm.vars.x, y = self.fsm.vars.y, ori = math.pi}, self.fsm.vars.transform_name, "/base_link")
   --print("self.fsm.vars.transform_name: " .. self.fsm.vars.transform_name)

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
   self.args["motor_move"] = {ori = self.fsm.vars.ori}
end

function FAILED:init()
   send_transrot(0,0,0)
end
