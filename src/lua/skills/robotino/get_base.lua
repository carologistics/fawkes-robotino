
----------------------------------------------------------------------------
--  get_base.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2015  Randolph Maaßen
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
name               = "get_base"
fsm                = SkillHSM:new{name=name, start="MPS_ALIGN", debug=true}
depends_skills     = {"mps_align", "product_pick"}
depends_interfaces = {
}

documentation      = [==[ get_base
alignes to a machine and picks up a base element

Parameters:
      @param tag_id    int   the tag_id
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

fsm:define_states{ export_to=_M,
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="PRODUCT_PICK", fail_to="FAILED"},
   {"PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{}

function MPS_ALIGN:init()
   -- align in front of the conveyor belt
   self.skills[1].tag_id = self.fsm.vars.tag_id
   -- TODO config value
   self.skills[1].x = 0.415
   self.skills[1].y = 0
   self.skills[1].ori = 0
end

function PRODUCT_PICK:init()
   -- nothing to do here, the skill does not get any parameter
end
