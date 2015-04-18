
----------------------------------------------------------------------------
--  bring_product_to.lua
--
--  Created: Thu Apr 16
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
fsm                = SkillHSM:new{name=name, start="MPS_ALIGN", debug=true}
depends_skills     = {"mps_align", "product_put"}
depends_interfaces = {
}

documentation      = [==[ 
aligns to a machine a puts a product on the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

fsm:define_states{ export_to=_M,
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="PRODUCT_PUT", fail_to="FAILED"},
   {"PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{}

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
