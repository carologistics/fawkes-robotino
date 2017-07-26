
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
depends_skills     = {"mps_recog","motor_move","tagless_mps_align"}
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"},
   {v = "mps_recognition_if", type = "MPSRecognitionInterface" ,id="/MarkerlessRecognition"},

}

documentation      = [==[ recog_from_align 

                          This skill does:
				Recognition of station from previous alignment 

			@param x1    : zone parameter for tagless_mps_align 
			@param y1    : zone parameter for tagless_mps_align  
			@param x2    : zone parameter for tagless_mps_align 
			@param y2    : zone parameter for tagless_mps_align 

			@param level : level of the iteration ( increased value means more iteration steps )  

]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
MPS_LENGTH = 0.7
MPS_WIDTH = 0.35
BOT_RADIUS = 0.46/2
START_DIST_TO_MPS = 0.35+BOT_RADIUS 
ROTATION_ANGLE = 10 
FACTOR = 0.8
START_POS={-MPS_WIDTH/2-START_DIST_TO_MPS,0.,0.}

-- Constants
MPS_TYPES = {
'No Station',
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


  if 2*self.fsm.vars.k-1 > self.fsm.vars.level -1 then 
		return false; 
	end

	local ang = ((((2*self.fsm.vars.k)-1 )/ self.fsm.vars.level) - 0.5) * math.pi
   

	self.fsm.vars.rotateY = START_DIST_TO_MPS * math.sin(ang)
   	self.fsm.vars.rotateX = START_DIST_TO_MPS -( math.cos(ang)  * START_DIST_TO_MPS)

   self.fsm.vars.angle = ang
   printf("ActAngle: %f",self.fsm.vars.angle) 
	self.fsm.vars.k = self.fsm.vars.k + 1 
	return true;
end 


function take_result(self) 
  return true
end

function calc_result(self) 
	

    return true  
end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"CALC_ANGLE",JumpState}, 
   {"CALC_RESULT",JumpState}, 
   {"TAKE_RESULT",JumpState}, 
   {"ALIGN",SkillJumpState,skills={{tagless_mps_align}}, final_to="MOVE_ROTATE", fail_to="FAILED"},
   {"MOVE_ROTATE", SkillJumpState,skills={{motor_move}}, final_to="MPS_RECOG",fail_to="FAILED"}, 
   {"MPS_RECOG",SkillJumpState,skills={{mps_recog}}, final_to="TAKE_RESULT",fail_to="FAILED"}, 
}

fsm:add_transitions{
   {"INIT", "CALC_ANGLE", cond=calc_init},
   {"INIT", "FAILED",cond=true },
   {"CALC_ANGLE", "ALIGN", cond=calc_angle},
   {"CALC_ANGLE", "CALC_RESULT",cond=true}, 
   {"TAKE_RESULT", "CALC_ANGLE", cond=take_result}, 
   {"CALC_RESULT","FINAL",cond=true}
}


function INIT:init() 


	 self.fsm.vars.resultCounter = 0
         self.fsm.vars.angle = 0
         self.fsm.vars.rotateX = 0
         self.fsm.vars.rotateY=0
         self.fsm.vars.k = 1
end

function MOVE_ROTATE:init() 

  self.args["motor_move"] = { x = self.fsm.vars.rotateX*FACTOR, y = self.fsm.vars.rotateY*FACTOR, ori = self.fsm.vars.angle*(-1)*FACTOR, 
  				vel_trans = 0.2, 
				tolerance = { x=0.002,y=0.002,ori=0.01}} 
end

function ALIGN:init() 

	self.args["tagless_mps_align"] = { x1  = self.fsm.vars.alignX1 , y1 = self.fsm.vars.alignY1 , x2  = self.fsm.vars.alignX2 , y2 = self.fsm.vars.alignY2 } 

end 
