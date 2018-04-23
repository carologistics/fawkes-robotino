
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
depends_skills     = {"mps_align", "product_pick", "drive_to_local","shelf_pick", "conveyor_align"}
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
if config:exists("/navgraph-generator-mps/left-shelf-distance") then
   left_shelf_distance = config:get_float("/navgraph-generator-mps/left-shelf-distance")
end
if config:exists("/navgraph-generator-mps/middle-shelf-distance") then
   middle_shelf_distance = config:get_float("/navgraph-generator-mps/middle-shelf-distance")
end
if config:exists("/navgraph-generator-mps/right-shelf-distance") then
   right_shelf_distance = config:get_float("/navgraph-generator-mps/right-shelf-distance")
end


function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to_local}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="SKILL_PRODUCT_PICK", fail_to="FAILED"},
   -- {"DECIDE_ENDSKILL", JumpState},
   -- {"SKILL_SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "MPS_ALIGN", cond=already_at_conveyor, desc="Already in front of the mps, align"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"},
   -- {"DECIDE_ENDSKILL", "SKILL_SHELF_PICK", cond="vars.shelf", desc="Pick from shelf"},
   -- {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PICK", cond="true", desc="Pick from conveyor"},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO:init()
   if self.fsm.vars.shelf ~= nil then
      -- navgraph point name for shelf is e.g. C-CS1-R last letter (R, M, L) for right, middle, left
      self.args["drive_to_local"] = {place = self.fsm.vars.place .. "-" .. string.sub(self.fsm.vars.shelf, 1, 1)}
   elseif self.fsm.vars.side == "input" then
      self.args["drive_to_local"] = {place = self.fsm.vars.place .. "-I"}
   else --if no side is given drive to output
      self.args["drive_to_local"] = {place = self.fsm.vars.place .. "-O"}
   end
end

function MPS_ALIGN:init()
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
   else --if no side is given get from output
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
   end

   self.args["mps_align"].x = 0.2

   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      self.args["mps_align"].y = 0.03
      if self.fsm.vars.shelf == "LEFT" then
          self.args["mps_align"].y += left_shelf_distance
      elseif self.fsm.vars.shelf == "MIDDLE" then
          self.args["mps_align"].y += middle_shelf_distance
      elseif self.fsm.vars.shelf == "RIGHT" then
          self.args["mps_align"].y += right_shelf_distance
      end
   else
      self.args["mps_align"].y = -0.03
   end
end

function CONVEYOR_ALIGN:init()
   self.args["conveyor_align"].mps = place

   if (self.fsm.vars.shelf ~= nil) then
       self.args["conveyor_align"].target_on_mps = self.fsm.vars.shelf .. "_S"
   else
       self.args["conveyor_align"].target_on_mps = string.upper(self.fsm.vars.side) .. "_C"
       self.args["conveyor_align"].disable_realsense_afterwards = false
   end
end


function SKILL_PRODUCT_PICK:init()
   self.args["product_pick"].offset_x = 0 
end

-- function SKILL_SHELF_PICK:init()
--    self.args["shelf_pick"] = {slot = self.fsm.vars.shelf}
-- end
