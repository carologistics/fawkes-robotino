
----------------------------------------------------------------------------
--  approach_test.lua
--
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
name               = "approach_test"
fsm                = SkillHSM:new{name=name, start="MPS_ALIGN", debug=true}
depends_skills     = {"mps_align","product_put","product_pick","shelf_put","shelf_pick","slide_put","conveyor_align"}
depends_interfaces = {
}

documentation      = [==[ 
aligns to a machine and puts a product on the conveyor.
It meant to be used to test the approach process when
there is no navgraph available

Parameters:
      @param option  whether to pick or put, default is input ("put" or "pick")
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param slide   optional true if you want to put it on the slide
      @param tag_id
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_ENDSKILL", fail_to="DECIDE_ENDSKILL"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_SHELF_PUT", SkillJumpState, skills={{shelf_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_SLIDE_PUT", SkillJumpState, skills={{slide_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PUT", cond="vars.shelf and vars.option=='put'", desc="Put on shelf"},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PICK", cond="vars.shelf and vars.option=='pick'", desc="Pick from shelf"},
   {"DECIDE_ENDSKILL", "SKILL_SLIDE_PUT", cond="vars.slide", desc="Put on slide"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PUT", cond="vars.option=='put'", desc="Put on conveyor"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PICK", cond="vars.option=='pick'", desc="Pick from conveyor"}
}

function MPS_ALIGN:init()
   -- align in front of the conveyor belt
   self.skills[1].x = 0.6
   self.skills[1].tag_id = self.fsm.vars.tag_id
end

function SKILL_SHELF_PUT:init()
   -- Just hand through the Shelf position
   self.skills[1].slot = self.fsm.vars.shelf
end

function SKILL_SHELF_PICK:init()
   -- Just hand through the Shelf position
   self.skills[1].slot = self.fsm.vars.shelf
end
