
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
depends_skills     = {"product_pick", "drive_to_machine_point", "conveyor_align", "shelf_pick", "gripper_commands"}
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
local pam = require("parse_module")

-- Constants

local get_z_clear = 0.05

-- If this matches the desired x distance of conveyor align, conveyor align has the chance
-- of not needing to move at all.
-- x distance to laserline
local X_AT_MPS = 0.28


function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

function shelf_set(self)
    return self.fsm.vars.shelf ~= nil
end


fsm:define_states{ export_to=_M, closure={navgraph=navgraph,shelf_set=shelf_set},
   {"INIT", JumpState},
   {"DRIVE_TO_MACHINE_POINT", SkillJumpState, skills={{drive_to_machine_point}}, final_to="CONVEYOR_ALIGN", fail_to="PRE_PRE_FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_ENDSKILL", fail_to="PRE_PRE_FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="PRE_PRE_FAILED"},
   {"SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="FINAL", fail_to="PRE_PRE_FAILED"},
   {"PRE_PRE_FAILED", SkillJumpState, skills={{gripper_commands}}, final_to="PRE_FAILED", fail_to="PRE_FAILED"},
   {"PRE_FAILED", SkillJumpState, skills={{gripper_commands}}, final_to="FAILED", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "CONVEYOR_ALIGN", cond=already_at_conveyor, desc="Already in front of the mps, align"},
   {"INIT", "DRIVE_TO_MACHINE_POINT", cond=true, desc="Everything OK"},
   {"DECIDE_ENDSKILL","SHELF_PICK", cond=shelf_set},
   {"DECIDE_ENDSKILL","PRODUCT_PICK", cond=true},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
   if self.fsm.vars.side == nil then
     self.fsm.vars.side = "output"
   end
end

function DRIVE_TO_MACHINE_POINT:init()
   local option = "CONVEYOR"
   if self.fsm.vars.shelf ~= nil then
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

function DRIVE_TO_MACHINE_POINT:exit()
  local dtmp_fsm = skillenv.get_skill_fsm("drive_to_machine_point")
  if dtmp_fsm.current == dtmp_fsm.states[dtmp_fsm.fail_state] then
    self.fsm:set_error("Drive To Machine Point Failed")
  end
end

function PRODUCT_PICK:init()
  self.args["product_pick"].place = self.fsm.vars.place
  self.args["product_pick"].side = self.fsm.vars.side
  self.args["product_pick"].slide = self.fsm.vars.slide
  self.args["product_pick"].shelf = self.fsm.vars.shelf
end

function PRODUCT_PICK:exit()
  local pp_fsm = skillenv.get_skill_fsm("product_pick")
  if pp_fsm.current == pp_fsm.states[pp_fsm.fail_state] then
    self.fsm:set_error("Product Pick Failed")
  end
end

function SHELF_PICK:init()
  self.args["shelf_pick"].slot = self.fsm.vars.shelf
end

function SHELF_PICK:exit()
  local sp_fsm = skillenv.get_skill_fsm("shelf_pick")
  if sp_fsm.current == sp_fsm.states[sp_fsm.fail_state] then
    self.fsm:set_error("Shelf Pick Failed")
  end
end

function CONVEYOR_ALIGN:init()
   if (self.fsm.vars.shelf == nil) then
     self.args["conveyor_align"].disable_realsense_afterwards = false
   end

   if self.fsm.vars.shelf ~= nil then
     self.args["conveyor_align"].side = "input"
     self.args["conveyor_align"].place = self.fsm.vars.place
     self.args["conveyor_align"].slide = self.fsm.vars.slide
   else
     self.args["conveyor_align"].side = self.fsm.vars.side
     self.args["conveyor_align"].place = self.fsm.vars.place
     self.args["conveyor_align"].slide = self.fsm.vars.slide
   end

end

function CONVEYOR_ALIGN:exit()
  local cv_fsm = skillenv.get_skill_fsm("conveyor_align")
  if cv_fsm.current == cv_fsm.states[cv_fsm.fail_state] then
    self.fsm:set_error("Conveyor Align Failed")
  end
end

function PRE_PRE_FAILED:init()
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = get_z_clear
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = true
end

function PRE_FAILED:init()
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = false
end

