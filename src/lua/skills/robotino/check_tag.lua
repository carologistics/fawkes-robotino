----------------------------------------------------------------------------
--  check_tag.lua -check if tag in front is the given tag_id 
--
--  Copyright 2015 The Carologistics Team
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
name               = "check_tag"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {}
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
   {v = "tag_info", type = "TagVisionInterface"},
}

documentation      = [==[
@param tag_id the id of the tag to check for 
]==]
function no_tag_vision()
   return not tag_info:has_writer()
end

function get_tag_visible(tag)
   return tag:visibility_history() > 0
end

-- Check if one tag is visible
function tag_visible()
    local tag = fsm.vars.iface_name
    return (tag and tag:visibility_history() > 0)
end

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={tag_visible=tag_visible},
   {"INIT", JumpState},
   {"CHECK_TAG", JumpState},
}

fsm:add_transitions{
   closure={input_ok=input_ok},
   {"INIT", "FAILED", cond="not vars.tag_id", desc="No tag_id given!"},
   {"INIT", "FAILED", cond=no_tag_vision, desc="Tag vision disabled"},
   {"INIT", "CHECK_TAG", cond=true},
   {"CHECK_TAG", "FAILED", cond="not tag_visible()", desc="The given tag_id is not visible"},
   {"CHECK_TAG", "FINAL", cond=tag_visible, desc="The given tag_id is visible"},
}

function CHECK_TAG:init()
   local i=1
   local tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7, tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }
   -- find the blackboard interface that has the ID given by the tag_id arg
   for j=0,15 do
      id = tag_info:tag_id(j)
      print("j: " .. tostring(j) .. " id: " .. tostring(id))
      -- stop when id is found
      if id == fsm.vars.tag_id then
         self.fsm.vars.transform_name = "/tag_" .. tostring(j)
         self.fsm.vars.iface_name = tags[i]
         break
      end
      i = i+1
   end
   print("i: " .. tostring(i) .. " tag_" .. tostring(i-1))
end
