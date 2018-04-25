
----------------------------------------------------------------------------
--  bring_product_to_new.lua
--
--  Created: Sat Apr 18
--  Copyright  2015  Johannes Rothe
--  Copyright  2018  Carsten Stoffels
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_align", "product_put_new", "drive_to_local","shelf_put_new","slide_put_new","conveyor_align","motor_move"}
depends_interfaces = {
  {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
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
skillenv.skill_module(_M)
-- Constants
-- Tunables
local x_distance = 0.4
local move_back_distance = -0.2
local y_input = 0
local y_slide = 0

function already_at_mps(self)
   return not (self.fsm.vars.atmps=="NO" or self.fsm.vars.atmps==nil)
end

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph,gripper_if=gripper_if},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"PRODUCT_PUT_NEW", SkillJumpState, skills={{product_put_new}}, final_to="FINAL", fail_to="FAILED"},
   {"SHELF_PUT_NEW", SkillJumpState, skills={{shelf_put_new}}, final_to="MOVE_BACK", fail_to="FAIELD"},
   {"SLIDE_PUT_NEW", SkillJumpState, skills={{slide_put_new}}, final_to="MOVE_BACK", fail_to="FAILED"},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "MPS_ALIGN", cond=already_at_conveyor, desc="At mps, skip drive_to"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"},
   {"DRIVE_TO", "FAILED", cond="not gripper_if:is_holds_puck()", desc="Abort if base is lost"},
   {"MPS_ALIGN", "FAILED", cond="not gripper_if:is_holds_puck()", desc="Abort if base is lost"},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PUT_NEW", cond="vars.shelf", desc="Put on shelf using new skill"},
   {"DECIDE_ENDSKILL", "SKILL_SLIDE_PUT_NEW", cond="vars.slide", desc="Put on slide using new skill"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PUT_NEW", cond=true, desc="Put on conveyor using new skill"},
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO:init()
   if self.fsm.vars.side == "output" then
      self.args["drive_to_local"] = {place = self.fsm.vars.place .. "-O"}
   else
      self.args["drive_to_local"] = {place = self.fsm.vars.place .. "-I"}
   end
end

function MPS_ALIGN:init()
   if self.fsm.vars.side == "output" then
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
   else
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
   end

   self.args["mps_align"].x = x_distance

  -- TODO needed ?
  -- if self.fsm.vars.side == "output" then
  --    self.args["mps_align"].y = -0.03
  -- else
  --    self.args["mps_align"].y = 0.03
  -- end
end

function CONVEYOR_ALIGN:init()
    if (self.fsm.vars.slide == nil or self.fsm.vars.shelf == nil) then
      self.args["conveyor_align"].disable_realsense_afterwards = false
    end
end

function SKILL_PRODUCT_PUT:init()
   self.args["product_put_new"].offset_x = 0
end

function SKILL_SHELF_PUT:init()
   -- Just hand through the Shelf position
   self.args["shelf_put_new"].slot = self.fsm.vars.shelf
end

function SKILL_SLIDE_PUT:init()
end

function MOVE_BACK:init()
  self.args["motor_move"].x = -move_back_distance
end
