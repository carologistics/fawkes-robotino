
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
depends_skills     = {"shelf_pick", "product_pick", "drive_to_machine_point", "conveyor_align", "gripper_commands_new", "motor_move"}
depends_interfaces = {
  {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
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
local pam = require("parse_module")
-- Constants
local X_AT_MPS = 0.4

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end


fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO_MACHINE_POINT", SkillJumpState, skills={{drive_to_machine_point}}, final_to="OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="CONVEYOR_ALIGN", fail_to="CONVEYOR_ALIGN"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="MOVE_TO_CENTER_MACHINE", fail_to="FAILED"},
   {"SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="MOVE_TO_CENTER_MACHINE", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"MOVE_TO_CENTER_MACHINE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "CONVEYOR_ALIGN", cond=already_at_conveyor, desc="Already in front of the mps, align"},
   {"INIT", "DRIVE_TO_MACHINE_POINT", cond=true, desc="Everything OK"},
   {"DECIDE_ENDSKILL", "PRODUCT_PICK", cond="not vars.shelf", desc="Product pick selected"},
   {"DECIDE_ENDSKILL", "SHELF_PICK", cond="vars.shelf", desc="Shelf pick selected"},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO_MACHINE_POINT:init()
   local option = "CONVEYOR"
   if self.fsm.vars.shelf then
      self.fsm.vars.side = "input"
   end
   if self.fsm.vars.side == "input" or self.fsm.vars.shelf then
      self.fsm.vars.tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
      self.args["drive_to_machine_point"] = {place = self.fsm.vars.place .. "-I", option = option, x_at_mps=X_AT_MPS, tag_id=self.fsm.vars.tag_id}
   else --if no side is given drive to output
      self.fsm.vars.tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
      self.args["drive_to_machine_point"] = {place = self.fsm.vars.place .. "-O", option = option, x_at_mps=X_AT_MPS, tag_id=self.fsm.vars.tag_id}
   end
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands_new"].command = "OPEN"
  self.args["gripper_commands_new"].wait = false
end

function SHELF_PICK:init()
   self.args["shelf_pick"] = {slot = self.fsm.vars.shelf}
end

function PRODUCT_PICK:init()
  self.args["product_pick"].place = self.fsm.vars.place
  self.args["product_pick"].side = self.fsm.vars.side
  self.args["product_pick"].slide = self.fsm.vars.slide
  self.args["product_pick"].shelf = self.fsm.vars.shelf
end

function CONVEYOR_ALIGN:init()
   if (self.fsm.vars.shelf == nil) then
     self.args["conveyor_align"].disable_realsense_afterwards = false
   end
   self.args["conveyor_align"].place = self.fsm.vars.place
   self.args["conveyor_align"].side = self.fsm.vars.side
end

function MOVE_TO_CENTER_MACHINE:init()
  local move_y = 0
  if self.fsm.vars.shelf then
    move_y = 0.3
  elseif self.fsm.vars.slide then
    move_y = 0.3
  end
  self.args["motor_move"].y = move_y
end
