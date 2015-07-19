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

-- Check if one tag is visible
function tag_visible(self)
  for k,v in pairs(self.fsm.vars.tags) do
    if v:visibility_history() > 0 then
      if self.fsm.vars.tag_id then -- can I see the searched tag
        id = tag_info:tag_id(k)
        if id == self.fsm.vars.tag_id then
          return true
        end
      else -- can I see any tag
        return true
      end
    end
  end
  return false
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
   --{"INIT", "FAILED", cond="not vars.tag_id", desc="No tag_id given!"},
   {"INIT", "FAILED", cond=no_tag_vision, desc="Tag vision disabled"},
   {"INIT", "CHECK_TAG", cond=true},
   {"CHECK_TAG", "FINAL", cond=tag_visible, desc="The given tag_id is visible"},
   {"CHECK_TAG", "FAILED", timeout=0.5, desc="The given tag_id is not visible"},
}

function INIT:init()
  self.fsm.vars.tags = {}
  self.fsm.vars.tags[0] = tag_0
  self.fsm.vars.tags[1] = tag_1
  self.fsm.vars.tags[2] = tag_2
  self.fsm.vars.tags[3] = tag_3
  self.fsm.vars.tags[4] = tag_4
  self.fsm.vars.tags[5] = tag_5
  self.fsm.vars.tags[6] = tag_6
  self.fsm.vars.tags[7] = tag_7
  self.fsm.vars.tags[8] = tag_8
  self.fsm.vars.tags[9] = tag_9
  self.fsm.vars.tags[10] = tag_10
  self.fsm.vars.tags[11] = tag_11
  self.fsm.vars.tags[12] = tag_12
  self.fsm.vars.tags[13] = tag_13
  self.fsm.vars.tags[14] = tag_14
  self.fsm.vars.tags[15] = tag_15
end

