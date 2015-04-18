
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
depends_skills     = {"mps_align", "product_put", "drive_to"}
depends_interfaces = {
}

documentation      = [==[ 
aligns to a machine and picks a product from the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    the side of the mps (either "input" or "output")
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="PRODUCT_PUT", fail_to="FAILED"},
   {"PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "FAILED", cond="not self.fsm.vars.side", desc="no side given"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"}
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function DRIVE_TO:init()
   if self.fsm.vars.side == "input" then
      self.skills[1].place = self.fsm.vars.place .. "-I"
   elseif self.fsm.vars.side == "output" then
      self.skills[1].place = self.fsm.vars.place .. "-O"
   end
end

function MPS_ALIGN:init()
   -- align in front of the conveyor belt
   self.skills[1].x = navgraph:node(self.fsm.vars.place):property_as_float("align_distance")
   if navgraph:node(self.fsm.vars.place):has_property("input_offset_y") then
      self.skills[1].y = navgraph:node(self.fsm.vars.place):property_as_float("input_offset_y")
   else
      self.skills[1].y = 0
   end
   self.skills[1].ori = 0
end

function PRODUCT_PUT:init()
   self.skills[1].place = self.fsm.vars.place
end
