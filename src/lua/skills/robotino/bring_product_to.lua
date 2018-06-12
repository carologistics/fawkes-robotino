
----------------------------------------------------------------------------
--  bring_product_to.lua
--
--  Created: Sat Apr 18
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
name               = "bring_product_to"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"mps_align", "product_put", "drive_to_machine_point","shelf_put","slide_put","conveyor_align","motor_move"}
depends_interfaces = {
  {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"},
  {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
}

documentation      = [==[ 
aligns to a machine and puts a product on the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    optional the side of the mps, default is input (give "output" to bring to output)
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param slide   optional true if you want to put it on the slide
      @param atmps   optional position at mps shelf, default NO (not at mps at all) : ( NO | LEFT | MIDDLE | RIGHT | CONVEYOR )
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local X_AT_MPS = 0.4

function already_at_mps(self)
   return not (self.fsm.vars.atmps=="NO" or self.fsm.vars.atmps==nil)
end

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph,gripper_if=gripper_if},
   {"INIT", JumpState},
   {"DRIVE_TO_MACHINE_POINT", SkillJumpState, skills={{drive_to_machine_point}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="PRODUCT_PUT", fail_to="FAILED"},
   {"PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "CONVEYOR_ALIGN", cond=already_at_conveyor, desc="At mps, skip drive_to_local"},
   {"INIT", "DRIVE_TO_MACHINE_POINT", cond=true, desc="Everything OK"},
   --{"DRIVE_TO_MACHINE_POINT", "FAILED", cond="not gripper_if:is_holds_puck()", desc="Abort if base is lost"},
   --{"CONVEYOR_ALIGN", "FAILED", cond="not gripper_if:is_holds_puck()", desc="Abort if base is lost"},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO_MACHINE_POINT:init()
   local option = "CONVEYOR"

   if self.fsm.vars.slide then
      option = "SLIDE"
   elseif self.fsm.vars.shelf then
     option = "SHELF_" .. self.fsm.vars.shelf
   end

   if self.fsm.vars.side == "input" or self.fsm.vars.shelf or self.fsm.vars.slide then
      self.fsm.vars.tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
      self.args["drive_to_machine_point"] = {place = self.fsm.vars.place .. "-I", option = option, x_at_mps=X_AT_MPS, tag_id=self.fsm.vars.tag_id}
   else --if no side is given drive to output
      self.fsm.vars.tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
      self.args["drive_to_machine_point"] = {place = self.fsm.vars.place .. "-O", option = option, x_at_mps=X_AT_MPS, tag_id=self.fsm.vars.tag_id}
   end
end

function CONVEYOR_ALIGN:init()
    if (self.fsm.vars.slide == nil or self.fsm.vars.shelf == nil) then
      self.args["conveyor_align"].disable_realsense_afterwards = false
    end

    self.args["conveyor_align"].side = self.fsm.vars.side
    self.args["conveyor_align"].place = self.fsm.vars.place
    self.args["conveyor_align"].slide = self.fsm.vars.slide
    self.args["conveyor_align"].shelf = self.fsm.vars.shelf
end


function PRODUCT_PUT:init()
  self.args["product_put"].place = self.fsm.vars.place
  self.args["product_put"].slide = self.fsm.vars.slide
  self.args["product_put"].shelf = self.fsm.shelf
  self.args["product_put"].side = self.fsm.vars.side
end
