
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
name               = "zone_recog_improved"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_recog", "drive_to","tagless_mps_align"}
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"},
   {v = "mps_recognition_if", type = "MPSRecognitionInterface" ,id="/MarkerlessRecognition"},

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

ROTATION_STEPS = 2*math.pi/8
INIT_ROTATION = 2*math.pi/8 

START_POS={-MPS_WIDTH/2-START_DIST_TO_MPS,0.,0.}

-- Constants
MPS_TYPES = {
'Base Station',
'Cap Station',
'Delivery Station',
'Ring Station',
'Storage Station',
}


function speak(...)
   speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(string.format(unpack(arg))))
   printf(unpack(arg))
end


function calc_result(self) 
	return true; 
end 

function calc_xy_init(self)

  -- zone argument is of the form  M-Z21
  self.fsm.vars.xStart = tonumber(string.sub(self.fsm.vars.zone, 4, 4)) - 0.5
  self.fsm.vars.yStart = tonumber(string.sub(self.fsm.vars.zone, 5, 5)) - 0.5
 
  if string.sub(self.fsm.vars.zone, 1, 1) == "M" then
   self.fsm.vars.xStart = 0 - self.fsm.vars.xStart; 
  end


  self.fsm.vars.x1 = self.fsm.vars.xStart-1;
  self.fsm.vars.x2 = self.fsm.vars.xStart;
  self.fsm.vars.x3 = self.fsm.vars.xStart+1;

  self.fsm.vars.y1 = self.fsm.vars.yStart-1;
  self.fsm.vars.y2 = self.fsm.vars.yStart;
  self.fsm.vars.y3 = self.fsm.vars.yStart+1;

  self.fsm.vars.driveCounter = 1;

  self.fsm.vars.xCurrent = self.fsm.vars.x1; 
  self.fsm.vars.yCurrent = self.fsm.vars.y1;

  self.fsm.vars.ori = INIT_ROTATION;  

  self.fsm.vars.secondAlign = false; 
  return true

end


function calc_next_drive(self) 


	if self.fsm.vars.x1 == self.fsm.vars.xCurrent and self.fsm.vars.yCurrent ~= self.fsm.vars.yStart then
	
		self.fsm.vars.xCurrent = self.fsm.vars.x2 

	elseif self.fsm.vars.x2 == self.fsm.vars.xCurrent or self.fsm.vars.yCurrent == self.fsm.vars.yStart then 
		
		self.fsm.vars.xCurrent = self.fsm.vars.x3

	elseif self.fsm.vars.x3 == self.fsm.vars.xCurrent then 

		self.fsm.vars.xCurrent = self.fsm.vars.x1
		self.fsm.vars.yCurrent = self.fsm.vars.yCurrent + 1; 
	end


	self.fsm.vars.ori = ROTATION_STEPS * self.fsm.vars.driveCounter  + INIT_ROTATION; 
	self.fsm.vars.driveCounter = self.fsm.vars.driveCounter + 1; 

	return true; 

end


function calc_other_side(self) 

	if self.fsm.vars.secondAlign then 
		return false; 
	end


	if self.fsm.vars.x1 == self.fsm.vars.xCurrent then 

                self.fsm.vars.xCurrent = self.fsm.vars.x3

        elseif self.fsm.vars.x2 == self.fsm.vars.xCurrent then

                self.fsm.vars.xCurrent = self.fsm.vars.x2

        elseif self.fsm.vars.x3 == self.fsm.vars.xCurrent then

                self.fsm.vars.xCurrent = self.fsm.vars.x1
        end


 
        if self.fsm.vars.y1 == self.fsm.vars.yCurrent then

                self.fsm.vars.yCurrent = self.fsm.vars.y3

        elseif self.fsm.vars.y2 == self.fsm.vars.yCurrent then

                self.fsm.vars.yCurrent = self.fsm.vars.y2

        elseif self.fsm.vars.y3 == self.fsm.vars.yCurrent then

                self.fsm.vars.xCurrent = self.fsm.vars.y1
        end

	self.fsm.vars.ori = self.fsm.vars.ori + math.pi 

	self.fsm.vars.secondAlign = true; 

        return true;


end

function recognize_mps(self) 
	return false; 

end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"CALCULATE_RESULT",JumpState}, 
   {"CALCULATE_NEXT_ALIGN",JumpState},
   {"CALCULATE_OTHER_SIDE",JumpState}, 
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="TRY_ALIGN", fail_to="FAILED"},
   {"TRY_ALIGN", SkillJumpState, skills={{tagless_mps_align}}, final_to="CALCULATE_OTHER_SIDE",fail_to="CALCULATE_NEXT_ALIGN"},

}

fsm:add_transitions{
   {"INIT", "DRIVE_TO", cond=calc_xy_init},
   {"INIT", "FAILED",cond=true },
   {"CALCULATE_NEXT_ALIGN","DRIVE_TO",cond=calc_next_drive}, 
   {"CALCULATE_RESULT","FINAL", cond=true},
   {"CALCULATE_OTHER_SIDE","DRIVE_TO",cond=calc_other_side}, 
   {"CALCULATE_OTHER_SIDE","CALCULATE_RESULT",cond=true}, 
}






function DRIVE_TO:init()
   self.args["drive_to"].x = self.fsm.vars.xCurrent 
   self.args["drive_to"].y = self.fsm.vars.yCurrent
   self.args["drive_to"].ori = self.fsm.vars.ori;
end


function TRY_ALIGN:init() 

end

