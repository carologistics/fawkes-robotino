
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
name               = "tagless_bring_product"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"tagless_mps_align", "product_put","shelf_put","slide_put","conveyor_align","motor_move"}
depends_interfaces = {
  {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
}

documentation      = [==[ 
aligns to a machine and puts a product on the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param zone    the name of the MPS (see navgraph)
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param slide   optional true if you want to put it on the slide
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants



fsm:define_states{ export_to=_M, closure={gripper_if=gripper_if},
   {"INIT", JumpState},
   {"MPS_ALIGN", SkillJumpState, skills={{tagless_mps_align}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_SHELF_PUT", SkillJumpState, skills={{shelf_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_SLIDE_PUT", SkillJumpState, skills={{slide_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "MPS_ALIGN", cond=true},
   {"MPS_ALIGN", "FAILED", cond="not gripper_if:is_holds_puck()", desc="Abort if base is lost"},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PUT", cond="vars.shelf", desc="Put on shelf"},
   {"DECIDE_ENDSKILL", "SKILL_SLIDE_PUT", cond="vars.slide", desc="Put on slide"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PUT", cond=true, desc="Put on conveyor"}
}

function INIT:init()
  self.fsm.vars.xZone = tonumber(string.sub(self.fsm.vars.zone, 4, 4)) - 0.5
  self.fsm.vars.yZone = tonumber(string.sub(self.fsm.vars.zone, 5, 5)) - 0.5
  if string.sub(self.fsm.vars.zone, 1, 1) == "M" then
   self.fsm.vars.xZone = 0 - self.fsm.vars.xZone
  end
 
  self.fsm.vars.alignX1 = self.fsm.vars.xZone - 0.5
  self.fsm.vars.alignY1 = self.fsm.vars.yZone - 0.5
  self.fsm.vars.alignX2 = self.fsm.vars.xZone + 0.5
  self.fsm.vars.alignY2 = self.fsm.vars.yZone + 0.5


end

function MPS_ALIGN:init()
  self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
  self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
  self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
  self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2


end

function CONVEYOR_ALIGN:init()
    if (self.fsm.vars.slide == nil or self.fsm.vars.shelf == nil) then
      self.args["conveyor_align"].disable_realsense_afterwards = false
    end
end

function SKILL_PRODUCT_PUT:init()
   self.args["product_put"].offset_x = 0 
end

function SKILL_SHELF_PUT:init()
   -- Just hand through the Shelf position
   self.args["shelf_put"].slot = self.fsm.vars.shelf
end
