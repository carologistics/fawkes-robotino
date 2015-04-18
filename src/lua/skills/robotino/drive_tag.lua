----------------------------------------------------------------------------
--  drive_tag.lua - moves to a tag using colli for longer distance
--
--  Copyright  2015 The Carologistics Team
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
name               = "drive_tag"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {"relgoto", "align_tag"}
depends_interfaces = {
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
   {v = "tag_info", type = "TagVisionInterface", id="/tag-vision/info"},
}

documentation      = [==[Drives the robot that the tag with the given id. Uses relgoto for collision detection on longer distances
@param id The id of the tag to drive to
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constant
-- use drive when father than this distnance in m
local drive_bound = 0.7
-- align tag with these parameters
local align_tag_param = { x = 0.4, y = 0, ori = 0 }

function printtable(table)
   for k,v in pairs(table) do
      print(k .. "\t\t= " .. v)
      if(k == "ori") then
         print("degree\t= " .. v*180/math.pi)
      end
   end
end

function get_tag_with_id(wanted_id)
   local i=1
   -- get the correct id
   for j=0,15 do
      id = tag_info:tag_id(j)
      -- stop when id is found
      if id == wanted_id then
         break
      end
      i = i+1
   end
   local tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }
   return tags[i]
end

function get_tag_distance(tag)
   -- in the camera frame of reference the forward distance is z(2) and the lateral distance is x(0)
   return math.sqrt((tag:translation(2)*tag:translation(2)) + (tag:translation(0)*tag:translation(0)))
end

function id_not_found(self)
   local found = false
   for i=1,15 do
      id=tag_info:tag_id(i)
      if id == wanted_id then
         found = true
         break
      end
   end
   return found
end

function tag_distance_long(self)
   tag = get_tag_with_id(self.fsm.vars.id)
   distance = get_tag_distance(tag)
   --print("distance: " .. distance)
   return distance > drive_bound
end

function tag_distance_short(self)
   return not tag_distance_long(self)
end

fsm:define_states { export_to=_M,
   {"INIT", JumpState},
   {"DRIVE", SkillJumpState, skills={{relgoto}}, final_to="ALIGN", fail_to="FAILED"},
   {"ALIGN", SkillJumpState, skills={{align_tag}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions {
   {"INIT", "FAILED", cond=id_not_found, desc="Tag with the given id not found"},
   {"INIT", "DRIVE", cond=tag_distance_long, desc="Far away tag, use collision detection drive"},
   --{"DRIVE", "ALIGN", cond=tag_distance_short, desc="Close enough to the tag, aligning is enough"},
   {"INIT", "ALIGN", cond=tag_distance_short, desc="Near tag, align"},
}

function INIT:init()
   self.fsm.vars.id = self.fsm.vars.id or 0
end

function get_yaw(qx, qy, qz, qw)
   return math.atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz)
end


function DRIVE:init()
   -- relgoto
   tag = get_tag_with_id(self.fsm.vars.id)
   local yaw = get_yaw(tag:rotation(0), tag:rotation(1), tag:rotation(2), tag:rotation(3))
   -- correct rotation
   yaw = yaw - (math.pi/2)
   -- the forward distance is the z trnaslation in the camera frame of reference
   forward_distance = tag:translation(2)
   -- the lateral distance is the x translation in the camera frame of reference
   lateral_distance = tag:translation(0)
   --get the distance to drive
   distance = { x = forward_distance ,--- self.fsm.vars.x,
               y = -lateral_distance ,--- self.fsm.vars.y,
               ori = -yaw}
   --print("original distance")
   --printtable(distance)
   distance.x = distance.x - ((align_tag_param.x+0.3) * math.cos(distance.ori) + align_tag_param.y * (-1 * math.sin(distance.ori)))
   distance.y = distance.y - ((align_tag_param.x+0.3) * math.sin(distance.ori) + align_tag_param.y * math.cos(distance.ori))

   print("x: " .. distance.x)
   print("y: " .. distance.y)
   self.skills[1].x = distance.x
   self.skills[1].y = distance.y
   self.skills[1].ori = distance.ori
end

function ALIGN:init()
   self.skills[1].x = align_tag_param.x
   self.skills[1].y = align_tag_param.y
   self.skills[1].ori = align_tag_param.ori
   self.skills[1].tag_id = self.fsm.vars.id

end
