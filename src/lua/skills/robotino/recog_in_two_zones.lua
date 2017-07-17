----------------------------------------------------------------------------
--  recog_in_two_zones.lua - drives to two zones and recognizes the mps located there 
--
--  Copyright 2017 The Carologistics Team
--
--  Author : Carsten Stoffels
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
name               = "recog_in_two_zones"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"zone_recog"}
depends_interfaces = {}

documentation      = [==[ recog_in_two_zones
	This skill does: Drives to two zones and calls zone_recog to recognize the mps located there
	@param zone1 First zone to drive to
	@param zone2 Second zone to drive to
]==]


-- Initialize as skill module
skillenv.skill_module(_M)


fsm:define_states{ export_to=_M,
   closure = {is_not_finished=is_not_finished,},
   {"INIT", JumpState},
   {"FAILED_TO_RECOGNIZE_MPS_1",JumpState}, 
   {"FAILED_TO_RECOGNIZE_MPS_2",JumpState}, 
   {"RECOGNIZE_MPS_1", SkillJumpState, skills={{zone_recog}}, final_to="RECOGNIZE_MPS_2", fail_to="FAILED_TO_RECOGNIZE_MPS_1"},
   {"RECOGNIZE_MPS_2", SkillJumpState, skills={{zone_recog}}, final_to="FINAL", fail_to="FAILED_TO_RECOGNIZE_MPS_2"},
}

fsm:add_transitions{
   {"INIT", "RECOGNIZE_MPS_1", cond=true},
   {"FAILED_TO_RECOGNIZE_MPS_1", "RECOGNIZE_MPS_2", cond=true,desc="Failed to recognize MPS_1"},
   {"FAILED_TO_RECOGNIZE_MPS_2", "FAILED", cond=true, desc="Failed to recognize MPS_2"}, 
}


function INIT:init()
end

function RECOGNIZE_MPS_1:init()
	 self.args["zone_recog"].zone  = self.fsm.vars.zone1	
end


function RECOGNIZE_MPS_2:init() 
	self.args["zone_recog"].zone = self.fsm.vars.zone2

end
