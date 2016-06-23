
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_align", "product_pick", "drive_to","shelf_pick", "conveyor_align"}
depends_interfaces = {
}

documentation      = [==[ 
aligns to a machine and picks a product from the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    optional the side of the mps (default is output give "input" to get from input)
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param atmps   optional if already standing at the mps: ( CONVEYOR )
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "MPS_ALIGN", cond=already_at_conveyor, desc="Already in front of the mps, align"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"},
   {"DECIDE_ENDSKILL", "MPS_ALIGN", cond="vars.counter <= 1", desc="Pick from shelf"},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PICK", cond="vars.shelf", desc="Pick from shelf"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PICK", cond="true", desc="Pick from conveyor"},
}

function INIT:init()
   self.fsm.vars.counter = 0
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO:init()
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      self.args["drive_to"] = {place = self.fsm.vars.place .. "-I"}
   else --if no side is given drive to output
      self.args["drive_to"] = {place = self.fsm.vars.place .. "-O"}
   end
end

function MPS_ALIGN:init()
   self.fsm.vars.counter = self.fsm.vars.counter + 1
   -- align in front of the conveyor belt
   self.args["mps_align"].x = navgraph:node(self.fsm.vars.place):property_as_float("align_distance")
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      if navgraph:node(self.fsm.vars.place):has_property("input_offset_y") then
         self.args["mps_align"].y = navgraph:node(self.fsm.vars.place):property_as_float("input_offset_y")
      else
         self.args["mps_align"].y = 0
      end
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
   else --if no side is given get from output
      if navgraph:node(self.fsm.vars.place):has_property("output_offset_y") then
         self.args["mps_align"].y = navgraph:node(self.fsm.vars.place):property_as_float("output_offset_y")
      else
         self.args["mps_align"].y = 0
      end
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
   end
   self.args["mps_align"].ori = 0
end

function CONVEYOR_ALIGN:init()
   self.args["conveyor_align"].product_present = true
end

function SKILL_PRODUCT_PICK:init()
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      if navgraph:node(self.fsm.vars.place):has_property("input_offset_x") then
         self.args["product_pick"].offset_x = navgraph:node(self.fsm.vars.place):property_as_float("input_offset_x")
      else
         self.args["product_pick"].offset_x = 0 
      end 
   else --if no side is given get from output
      if navgraph:node(self.fsm.vars.place):has_property("output_offset_x") then
         self.args["product_pick"].offset_x = navgraph:node(self.fsm.vars.place):property_as_float("output_offset_x")
      else
         self.args["product_pick"].offset_x = 0 
      end 
   end
end

function SKILL_SHELF_PICK:init()
   self.args["shelf_pick"] = {slot = self.fsm.vars.shelf}
end
