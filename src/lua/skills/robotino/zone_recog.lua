
----------------------------------------------------------------------------
--  drive_to_zones.lua
--
--  Created: Sat Jul 01 13:27:47 2017
--  Copyright 2017 Carsten Stoffels
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
name               = "zone_recog"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_recog", "goto"}
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"}
}

documentation      = [==[ drive_to_zone

                          This skill does:
                                drives to the corners of the zone and tries to recognize the mps                  

]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
MPS_LENGTH = 0.7
MPS_WIDTH = 0.35
BOT_RADIUS = 0.46/2
START_DIST_TO_MPS = 0.15+BOT_RADIUS

START_POS={-MPS_WIDTH/2-START_DIST_TO_MPS,0.,0.}



function calc_xy_coordinates(self)
 
 
  -- zone argument is of the form  M-Z21
  self.fsm.vars.xZone = tonumber(string.sub(self.fsm.vars.zone, 4, 4)) - 0.5
  self.fsm.vars.yZone = tonumber(string.sub(self.fsm.vars.zone, 5, 5)) - 0.5
  if string.sub(self.fsm.vars.zone, 1, 1) == "M" then
   self.fsm.vars.xZone = 0 - self.fsm.vars.xZone
  end
 
  self.fsm.vars.xLeft = self.fsm.vars.xZone-1
  self.fsm.vars.xRight = self.fsm.vars.xZone+1

  self.fsm.vars.yUp = self.fsm.vars.yZone+1 
  self.fsm.vars.yDown = self.fsm.vars.yZone-1 

  return true

end


fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"DRIVE_TO_START", SkillJumpState, skills={{goto}}, final_to="RECOGNIZE_MPS_START", fail_to="FAILED"},
   {"DRIVE_TO_SECOND", SkillJumpState, skills={{goto}}, final_to="RECOGNIZE_MPS_SECOND", fail_to="FAILED"},
   {"DRIVE_TO_THIRD", SkillJumpState, skills={{goto}}, final_to="RECOGNIZE_MPS_THIRD", fail_to="FAILED"},
   {"DRIVE_TO_FINAL", SkillJumpState, skills={{goto}}, final_to="RECOGNIZE_MPS_FINAL", fail_to="FAILED"},
   {"RECOGNIZE_MPS_START", SkillJumpState, skills={{mps_recog}}, final_to="DRIVE_TO_SECOND", fail_to="FAILED"},
   {"RECOGNIZE_MPS_SECOND", SkillJumpState, skills={{mps_recog}}, final_to="DRIVE_TO_THIRD", fail_to="FAILED"},
   {"RECOGNIZE_MPS_THIRD", SkillJumpState, skills={{mps_recog}}, final_to="DRIVE_TO_FINAL", fail_to="FAILED"},
   {"RECOGNIZE_MPS_FINAL", SkillJumpState, skills={{mps_recog}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "DRIVE_TO_START", cond=calc_xy_coordinates},
   {"INIT", "FAILED", cond=true, desc="zoneID is not between 1 and 106"},
}



function DRIVE_TO_START:init()
   self.args["goto"].x = self.fsm.vars.xLeft 
   self.args["goto"].y = self.fsm.vars.yZone
   --self.args["goto"].region_trans = 2
   self.args["goto"].ori = 0
end

function DRIVE_TO_SECOND:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yDown
   --self.args["goto"].region_trans = 2
   self.args["goto"].ori = 1.57 
end
function DRIVE_TO_THIRD:init()
   self.args["goto"].x = self.fsm.vars.xRight
   self.args["goto"].y = self.fsm.vars.yZone
   --self.args["goto"].region_trans = 2
   self.args["goto"].ori = 3.1415
end

function DRIVE_TO_FINAL:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yUp
   --self.args["goto"].region_trans = 2
   self.args["goto"].ori = 4.71
end



