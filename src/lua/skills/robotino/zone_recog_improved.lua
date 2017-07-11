
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
depends_skills     = {"mps_recog", "drive_to"}
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

RADIANT_STEPS = 1.0   
RADIANT_INIT = 4.71

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

function calc_final_result(self) 

    result = {}
    for i=0,5 do
      result[i] = 0
    end

    result[self.fsm.vars.resultStart] = result[self.fsm.vars.resultStart] + 1
    result[self.fsm.vars.resultSecond] = result[self.fsm.vars.resultSecond] + 1
    result[self.fsm.vars.resultThird] = result[self.fsm.vars.resultThird] + 1
    result[self.fsm.vars.resultFinal] = result[self.fsm.vars.resultFinal] + 1
   
    max = 1 

    for j=0,4 do 
        if result[j+1] > result[j] then
	   max = j+1
 	end 
    end
    
    recognition_result = MPS_TYPES[max]
    printf("The result of resultStart is %s",self.fsm.vars.resultStart)
    printf("The result of resultSecond is %s",self.fsm.vars.resultSecond)
    printf("The result of resultThird is %s",self.fsm.vars.resultThird)
    printf("The result of resultFinal is %s",self.fsm.vars.resultFinal)
    printf("The result of max is %s",max)
    printf("The result array entry 0 is %s", result[1]) 
    printf("The result of zone_recog is %s",recognition_result)


    return true

end

function calc_result(self) 
	return true; 
end 

function calc_xy_init(self)

  -- zone argument is of the form  M-Z21
  self.fsm.vars.xZone = tonumber(string.sub(self.fsm.vars.zone, 4, 4)) - 0.5
  self.fsm.vars.yZone = tonumber(string.sub(self.fsm.vars.zone, 5, 5)) - 0.5
  if string.sub(self.fsm.vars.zone, 1, 1) == "M" then
   self.fsm.vars.xZone = 0 - self.fsm.vars.xZone; 
  end
 
  self.fsm.vars.yStart = self.fsm.vars.yZone-1;
  self.fsm.vars.xStart = self.fsm.vars.xZone; 
  self.fsm.vars.ori = 1.57;

  self.fsm.vars.xRotate = self.fsm.vars.xStart; 
  self.fsm.vars.yRotate = self.fsm.vars.yStart;

 printf("xRotate is  %s",self.fsm.vars.xRotate)
 printf("yRotate is  %s",self.fsm.vars.yRotate)
  self.fsm.vars.currentRotationStep = 1;


  return true

end

function calc_xy_rotation(self) 
	

 self.fsm.vars.xRotate = self.fsm.vars.xStart + math.cos(RADIANT_INIT + self.fsm.vars.currentRotationStep * RADIANT_STEPS); 
 self.fsm.vars.yRotate = self.fsm.vars.yStart + math.sin(RADIANT_INIT + self.fsm.vars.currentRotationStep * RADIANT_STEPS);

 printf("xRotate is %s",self.fsm.vars.xRotate)
 printf("yRotate is %s",self.fsm.vars.yRotate)
 self.fsm.vars.currentRotationStep = self.fsm.vars.currentRotationStep + 1;

 printf("currentRotationStep is%s",self.fsm.vars.currentRotationStep) 

 return true;

end


function recognize_mps(self) 
	return false; 

end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"CALCULATE_ROTATION",JumpState},
   {"CALCULATE_RESULT",JumpState}, 
   {"DRIVE_TO_START", SkillJumpState, skills={{drive_to}}, final_to="CALCULATE_ROTATION", fail_to="FAILED"},
   {"DRIVE_CIRCLE", SkillJumpState, skills={{drive_to}}, final_to="CALCULATE_ROTATION",fail_to="FAILED"}, 
}

fsm:add_transitions{
   {"INIT", "DRIVE_TO_START", cond=calc_xy_init},
   {"INIT", "FAILED",cond=true },
   {"CALCULATE_ROTATION","DRIVE_CIRCLE",cond=calc_xy_rotation}, 
   {"CALCULATE_ROTATION","FINAL", cond=true},
}




function INIT:init() 

end

function DRIVE_TO_START:init()
   self.args["drive_to"].x = self.fsm.vars.xStart 
   self.args["drive_to"].y = self.fsm.vars.yStart
   --self.args["drive_to"].region_trans = 2
   self.args["drive_to"].ori = self.fsm.vars.ori;
end

function DRIVE_CIRCLE:init()

  self.args["drive_to"].x = self.fsm.vars.xRotate; 
  self.args["drive_to"].y = self.fsm.vars.yRotate;
	
end
