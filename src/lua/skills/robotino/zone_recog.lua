
----------------------------------------------------------------------------
--  zone_recog.lua
--
--  Created: Sat Jul 01 13:27:47 2017
--  Copyright 2017 Carsten Stoffels, Daniel Habering
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
depends_skills     = {"recog_from_align", "goto","tagless_mps_align"}
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"},
   {v = "mps_recognition_if", type = "MPSRecognitionInterface" ,id="/MarkerlessRecognition"},

}

documentation      = [==[ zone_recog

			This skill does: Circles a given zone in which an mps should be located. Searches for the long sides of the mps. Aligns there and calls recog_from_align.
			If the first run does not successfull recognizes the mps, it is redone by taking more picture at each side.

			@param zone Zone in which a MPS should be located	
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
MPS_COUNT = 5
MPS_WIDTH = 0.35
BOT_RADIUS = 0.46/2
START_DIST_TO_MPS = 0.15+BOT_RADIUS

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

function calc_final_result(self) 
    if max~=j and self.fsm.vars.i*2>=8 then
      self.fsm.vars.i = self.fsm.vars.i * 2
      return false
    end
    return true

end


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

  self.fsm.vars.alignX1 = self.fsm.vars.xZone - 0.5
  self.fsm.vars.alignY1 = self.fsm.vars.yZone - 0.5
  self.fsm.vars.alignX2 = self.fsm.vars.xZone + 0.5
  self.fsm.vars.alignY2 = self.fsm.vars.yZone + 0.5



  return true

end


fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"DRIVE1", SkillJumpState, skills={{goto}}, final_to="ALIGN", fail_to="MOVE_FAILED"},
   {"DRIVE2", SkillJumpState, skills={{goto}}, final_to="ALIGN", fail_to="MOVE_FAILED"},
   {"DRIVE3", SkillJumpState, skills={{goto}}, final_to="ALIGN", fail_to="MOVE_FAILED"},
   {"DRIVE4", SkillJumpState, skills={{goto}}, final_to="ALIGN", fail_to="MOVE_FAILED"},
   {"CALCULATE_RESULT",JumpState},
   {"EXPLORE",SkillJumpState, skills={{recog_from_align}}, final_to="CHOOSE_NEXT", fail_to="CHOOSE_NEXT_FAILED"},
   {"CHOOSE_NEXT_FAILED",JumpState}, 
   {"CHOOSE_NEXT",JumpState},
   {"MOVE_FAILED", JumpState},
   {"ALIGN",SkillJumpState, skills={{tagless_mps_align}}, final_to="EXPLORE", fail_to="MOVE_FAILED"},
}
fsm:add_transitions{
   {"INIT", "DRIVE1", cond=calc_xy_coordinates},
   {"INIT", "FAILED", cond=true},

   {"MOVE_FAILED", "DRIVE2", cond="vars.last ==1"},
    {"MOVE_FAILED", "DRIVE3", cond="vars.last ==2"},
    {"MOVE_FAILED", "DRIVE4", cond="vars.last ==3"},
    {"MOVE_FAILED", "CALCULATE_RESULT", cond="vars.last ==4"},
 {"MOVE_FAILED", "FAILED", cond="vars.last ==4 and vars.last == nil"},
   {"CHOOSE_NEXT", "DRIVE3", cond="vars.last == 1"},
   {"CHOOSE_NEXT", "DRIVE4", cond="vars.last == 2"},
   {"CHOOSE_NEXT", "CALCULATE_RESULT", cond="vars.last == 3 or vars.last == 4"},
   {"CHOOSE_NEXT_FAILED", "DRIVE3", cond="vars.last == 1"},
   {"CHOOSE_NEXT_FAILED", "DRIVE4", cond="vars.last == 2"},
   {"CHOOSE_NEXT_FAILED", "CALCULATE_RESULT", cond="vars.last == 3 or vars.last == 4"},
   {"CALCULATE_RESULT","FINAL", cond=calc_final_result},
   {"CALCULATE_RESULT","DRIVE1", cond="vars.i <= 8"},
  {"CALCULATE_RESULT","FAILED",cond=true}
}

function INIT:init()
	self.fsm.vars.i = 1
	self.fsm.vars.count = 0
end

function EXPLORE:init()
	self.args["recog_from_align"].level = self.fsm.vars.i
	self.args["recog_from_align"].alignX1 = self.fsm.vars.alignX1
	self.args["recog_from_align"].alignY1 = self.fsm.vars.alignY1
	self.args["recog_from_align"].alignX2 = self.fsm.vars.alignX2
	self.args["recog_from_align"].alignY2 = self.fsm.vars.alignY2
end
function ALIGN:init()
  self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
  self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
  self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
  self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2

end


function CHOOSE_NEXT:init()
  
end

function DRIVE1:init()
     self.args["goto"].x = self.fsm.vars.xLeft 
   self.args["goto"].y = self.fsm.vars.yZone
   self.args["goto"].ori = 0
   self.fsm.vars.last = 1
end

function DRIVE2:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yDown
   self.args["goto"].ori = 1.57
   self.fsm.vars.last = 2
end

function DRIVE3:init()
   self.args["goto"].x = self.fsm.vars.xRight
   self.args["goto"].y = self.fsm.vars.yZone
   self.args["goto"].ori = 3.1415	

   self.fsm.vars.last = 3

end

function DRIVE4:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yUp
   self.args["goto"].ori = 4.71
   self.fsm.vars.last = 4
end



