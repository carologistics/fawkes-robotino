
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
name               = "recog_from_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_recog","motor_move"}
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
START_DIST_TO_MPS = 0.35+BOT_RADIUS 
ROTATION_ANGLE = 10 

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


function calc_init(self) 

	return true; 
end

function calc_angle(self) 


        if 2*self.fsm.vars.k-1 <= self.fsm.vars.iteration -1 then 
		return false; 
	end

	self.fsm.vars.angle = (2*self.fsm.vars.k-1 / self.fsm.vars.level - 0.5) * math.pi 

	self.fsm.vars.rotateX = START_DIST_TO_MPS * math.sin(self.fsm.vars.angle)
   	self.fsm.vars.rotateY = START_DIST_TO_MPS - math.cos(self.fsm.vars.angle)  * START_DIST_TO_MPS


	self.fsm.vars.k = self.fsm.vars.k + 1 
	return true;
end 


function take_result(self) 

	self.fsm.vars.result[self.fsm.vars.resultCounter] = mps_recognition_if_:mpstype() 
	self.fsm.vars.resultCounter = self.fsm.vars.resultCounter + 1
end

function calc_result(self) 
	
    local  result = {}
    
    for i=0,5 do
      result[i] = 0
    end


    for i=0, self.fsm.vars.level do 
 	result[self.fsm.vars.result[i]] = result[self.fsm.vars.result[i]] + 1 

    end


    max = 0

    for j=0,5 do
        if result[j] > result[max] then
           max = j
        end
    end

    mps_recognition_if_:set_mpstype(max) 

    return true  
end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"CALC_ANGLE",JumpState}, 
   {"CALC_RESULT",JumpState}, 
   {"TAKE_RESULT",JumpState}, 
   {"MOVE_ROTATE", SkillJumpState,skills={{motor_move}}, final_to="MPS_RECOG",fail_to="FAILED"}, 
   {"MPS_RECOG",SkillJumpState,skills={{mps_recog}}, final_to="TAKE_RESULT",fail_to="FAILED"}, 
}

fsm:add_transitions{
   {"INIT", "CALC_ANGLE", cond=calc_init},
   {"INIT", "FAILED",cond=true },
   {"CALC_ANGLE", "MOVE_ROTATE", cond=calc_angle},
   {"CALC_ANGLE", "CALC_RESULT",cond=true}, 
   {"TAKE_RESULT", "CALC_ANGLE", cond=take_result}, 

}


function INIT:init() 

	self.fsm.vars.results = {} 

	for i =1, self.fsm.vars.level do 
		results[i] = 0 
	end 

	self.fsm.vars.resultCounter = 0

end

function MOVE_ROTATE:init() 

  self.args["motor_move"] = { x = self.fsm.vars.rotateX, y = self.fsm.vars.rotateY, ori = self.fsm.vars.angle, 
  				vel_trans = 0.2, 
				tolerance = { x=0.002,y=0.002,ori=0.01}} 
end

