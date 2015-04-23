
----------------------------------------------------------------------------
--  get_product_from.lua
--
--  Created: Fri Apr 17
--  Copyright  2015  Johannes Rothe
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
name               = "get_product_from"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"mps_align", "product_pick", "drive_to","shelf_pick", "mps_detect_signal"}
depends_interfaces = {
}

documentation      = [==[ 
aligns to a machine and picks a product from the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    the side of the mps (default is output give "input" to get from input)
      @param shelf   Position on shelf: ( LEFT | MIDDLE | RIGHT )
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"MPS_DETECT_SIGNAL", SkillJumpState, skills={{mps_detect_signal}}, final_to="SKILL_PRODUCT_PICK", fail_to="FAILED"},
   {"SKILL_PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PICK", cond="vars.shelf", desc="Pick from shelf"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PICK", cond="vars.side", desc="Pick from conveyor and don't watch the light"},
   {"DECIDE_ENDSKILL", "MPS_DETECT_SIGNAL", cond=true, desc="Pick from conveyor and watch light"},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO:init()
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      self.skills[1].place = self.fsm.vars.place .. "-I"
   else --if no side is given drive to output
      self.skills[1].place = self.fsm.vars.place .. "-O"
   end
end

function MPS_ALIGN:init()
   -- align in front of the conveyor belt
   self.skills[1].x = navgraph:node(self.fsm.vars.place):property_as_float("align_distance")
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      if navgraph:node(self.fsm.vars.place):has_property("input_offset_y") then
         self.skills[1].y = navgraph:node(self.fsm.vars.place):property_as_float("input_offset_y")
      else
         self.skills[1].y = 0
      end
      self.skills[1].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
   else --if no side is given get from output
      if navgraph:node(self.fsm.vars.place):has_property("output_offset_y") then
         self.skills[1].y = navgraph:node(self.fsm.vars.place):property_as_float("output_offset_y")
      else
         self.skills[1].y = 0
      end
      self.skills[1].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
   end
   self.skills[1].ori = 0
end

function MPS_DETECT_SIGNAL:init()
   self.skills[1].wait_for = "GREEN"
   self.skills[1].place = self.fsm.vars.place
end

function SKILL_PRODUCT_PICK:init()
   self.skills[1].place = self.fsm.vars.place
end

function SKILL_SHELF_PICK:init()
   self.skills[1].slot = self.fsm.vars.shelf
end
