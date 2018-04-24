
----------------------------------------------------------------------------
--  get_product_from_new.lua
--
--  Created: Fri Apr 17
--  Copyright  2018 Carsten Stoffels
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
name               = "get_product_from_new"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_align", "product_pick_new", "drive_to"}
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
--
-- Tunables
local x_Distance = 0.4
local y_Input = -0.3
local y_Output = 0.3
local y_Shelf_Left = 0.7
local y_Shelf_Middle = 0.8
local y_Shelf_Right = 0.9

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"PRODUCT_PICK_NEW", SkillJumpState, skills={{product_pick_new}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "MPS_ALIGN", cond=already_at_conveyor, desc="Already in front of the mps, align"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"},
}

function INIT:init()
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
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
   else --if no side is given get from output
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
   end

   self.args["mps_align"].x = x_Distance

   if self.fsm.vars.side == "input" then
      self.args["mps_align"].y = y_Input
   else if self.fsm.vars.side == "output" then
      self.args["mps_align"].y = y_Output
   else if self.fsm.vars.shelf == "LEFT" then
      self.args["mps_align"].y = y_Shelf_Left
   else if self.fsm.vars.shelf == "MIDDLE" then
      self.args["mps_align"].y = y_Shelf_Middle
   else if self.fsm.vars.shelf == "RIGHT" then
      self.args["mps_align"].y = y_Shelf_Right
   end
end

function CONVEYOR_ALIGN:init()
   if (self.fsm.vars.shelf == nil) then
     self.args["conveyor_align"].disable_realsense_afterwards = false
   end
end
