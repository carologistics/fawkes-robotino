
----------------------------------------------------------------------------
--  approach_test.lua
--
--  Copyright  2015  Johannes Rothe
--  Copyright  2019  Morian Sonnet
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_align","product_put","product_pick","shelf_pick","conveyor_align"}
depends_interfaces = {
}

documentation      = [==[ 
Aligns to an machine and interacts with the MPS
It is meant to test interactions with MPS, when a navgraph is not available
You are responsible to enter values which make sense, no sanity check is executed.

Parameters:
      @param place   description of the MPS, e.g. M-CRS1

      @param action  optional whether to pick or put ( PUT | PICK )
                     default is PICK
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
                     when any value defined -> shelf pick
      @param slide   optional true if you want to put it on the slide
      @param side    optional which side of the MPS should be considered ( input | output )
                     default is input for action=="PUT"
                     and output for action=="PICK"
      @param wait    optional true if the robot should wait for two seconds after each step
      for example: approach_test{place="M-CRS1",action="PUT", side="output"}
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants

-- If this matches the desired x distance of conveyor align, conveyor align has the chance
-- of not needing to move at all.
-- x distance to laserline
local X_AT_MPS = config:get_float_or_default("/skills/approach_test/X_AT_MPS", 0.28)

-- offset of the conveyor relative to the middlepoint of the laser line
local CONVEYOR_IN_OUT_OFFSET = config:get_float_or_default("/skills/approach_test/CONVEYOR_IN_OUT_OFFSET", 0.03)
-- offset of the slide relative to the middle point of the laser liner
local OFFSET_SLIDE = config:get_float_or_default("skills/approach_test/OFFSET_SLIDE", -0.27) 

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="DECIDE_WAIT_MPS_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_WAIT_CONVEYOR_ALIGN", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_SHELF_PICK", SkillJumpState, skills={{shelf_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"WAIT_MPS_ALIGN", JumpState},
   {"WAIT_CONVEYOR_ALIGN", JumpState},
   {"DECIDE_WAIT_MPS_ALIGN", JumpState},
   {"DECIDE_WAIT_CONVEYOR_ALIGN", JumpState}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="vars.error"}, -- not yet used
   {"INIT", "MPS_ALIGN", cond="true"},
   {"DECIDE_WAIT_MPS_ALIGN", "WAIT_MPS_ALIGN", cond="vars.wait"},
   {"DECIDE_WAIT_MPS_ALIGN", "CONVEYOR_ALIGN", cond="true"},
   {"WAIT_MPS_ALIGN", "CONVEYOR_ALIGN", timeout=2},
   {"DECIDE_WAIT_CONVEYOR_ALIGN", "WAIT_CONVEYOR_ALIGN", cond="vars.wait"},
   {"DECIDE_WAIT_CONVEYOR_ALIGN", "DECIDE_ENDSKILL", cond="true"},
   {"WAIT_CONVEYOR_ALIGN", "DECIDE_ENDSKILL", timeout=2},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PICK", cond="vars.shelf", desc="Pick from shelf"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PUT", cond="vars.action=='PUT'", desc="Put on conveyor or slide"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PICK", cond="vars.action=='PICK'", desc="Pick from conveyor"},
   {"DECIDE_ENDSKILL", "FAILED", cond="true", desc="action was not correct"}
}

function INIT:init()
-- make everything explicit
   if self.fsm.vars.shelf ~= nil then
     self.fsm.vars.action = "PICK"
   end
   if not self.fsm.vars.slide then
     self.fsm.vars.slide = false
   else
     self.fsm.vars.action = "PUT"
   end
   if self.fsm.vars.action == "PUT" then
     if self.fsm.vars.side == nil then
       if self.fsm.vars.slide then
         self.fsm.vars.side = "input"
       else
         self.fsm.vars.side = "input"
       end
     end
   else
     self.fsm.vars.action = "PICK"
     if self.fsm.vars.side == nil then
       if self.fsm.vars.shelf == nil then
         self.fsm.vars.side = "output"
       else
         self.fsm.vars.side = "input"
       end
     end
   end
end

function MPS_ALIGN:init()
   self.args["mps_align"].x = X_AT_MPS
   if not self.fsm.vars.slide then
     if self.fsm.vars.side == "input" then -- input side
       self.args["mps_align"].y = CONVEYOR_IN_OUT_OFFSET
     else -- output side
       self.args["mps_align"].y = -CONVEYOR_IN_OUT_OFFSET
     end
   else -- the slide is considered
     self.args["mps_align"].y = OFFSET_SLIDE
   end
end

function CONVEYOR_ALIGN:init()
  self.args["conveyor_align"].place = self.fsm.vars.place
  self.args["conveyor_align"].slide = self.fsm.vars.slide
  self.args["conveyor_align"].side = self.fsm.vars.side
end

function SKILL_SHELF_PICK:init()
   -- Just hand through the Shelf position
   self.args["shelf_pick"].slot = self.fsm.vars.shelf
end

function SKILL_PRODUCT_PUT:init()
   self.args["product_put"].slide = self.fsm.vars.slide
end
