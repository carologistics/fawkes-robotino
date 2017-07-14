
----------------------------------------------------------------------------
--  goto_zones.lua
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
depends_skills     = {"recog_from_align", "goto","tagless_mps_align"}
depends_interfaces = {
  -- {v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"},
   {v = "mps_recognition_if", type = "MPSRecognitionInterface" ,id="/MarkerlessRecognition"},

}

documentation      = [==[ goto_zone

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
  -- speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(string.format(unpack(arg))))
   printf(unpack(arg))
end

function calc_final_result(self) 
    count = 0
    result = {}
    for i=0,5 do
      result[i] = 0
    end
    
    local l = self.fsm.vars.results
    
    while l do 
	    result[l.value] = result[l.value]+1
	    l = l.next
    end
    
    max = 0

    for j=0,5 do 
        if result[j] > result[max] then
	         printf("Result %d is %d ",j,result[j])
           max = j
 	end 
    end
   
    if max == 0 then
    	return false
    end

    for j=0,5 do
	    if result[j] == result[max] then
		    if j~=max then
			    return false
		    end
	    end
    end

    recognition_result = MPS_TYPES[max+1]
    printf("The result of zone_recog is %s",recognition_result)


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

  return true

end


fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"DRIVE1", SkillJumpState, skills={{goto}}, final_to="ALIGN1", fail_to="FAILED"},
   {"DRIVE2", SkillJumpState, skills={{goto}}, final_to="ALIGN2", fail_to="FAILED"},
   {"DRIVE3", SkillJumpState, skills={{goto}}, final_to="ALIGN3", fail_to="FAILED"},
   {"DRIVE4", SkillJumpState, skills={{goto}}, final_to="ALIGN4", fail_to="FAILED"},
   {"CALCULATE_RESULT",JumpState},
   {"EXPLORE",SkillJumpState, skills={{recog_from_align}}, final_to="CHOOSE_NEXT", fail_to="CHOOSE_NEXT"},
   {"CHOOSE_NEXT",JumpState},
   {"ALIGN1",SkillJumpState, skills={{tagless_mps_align}}, final_to="EXPLORE", fail_to="DRIVE2"},
   {"ALIGN2",SkillJumpState, skills={{tagless_mps_align}}, final_to="EXPLORE", fail_to="FAILED"},
   {"ALIGN3",SkillJumpState, skills={{tagless_mps_align}}, final_to="EXPLORE", fail_to="FAILED"},
   {"ALIGN4",SkillJumpState, skills={{tagless_mps_align}}, final_to="EXPLORE", fail_to="FAILED"},
}
fsm:add_transitions{
   {"INIT", "DRIVE1", cond=calc_xy_coordinates},
   {"INIT", "FAILED", cond=true},
   {"CHOOSE_NEXT", "DRIVE3", cond="vars.last == 1"},
   {"CHOOSE_NEXT", "DRIVE4", cond="vars.last == 2"},
   {"CHOOSE_NEXT", "CALCULATE_RESULT", cond="vars.last == 3 or vars.last == 4"},
   {"CALCULATE_RESULT","FINAL", cond=calc_final_result},
   {"CALCULATE_RESULT","DRIVE1", cond="2*vars.i <= 8"},
  {"CALCULATE_RESULT","FAILED",cond=true}
}

function INIT:init()
	self.fsm.vars.i = 1
	self.fsm.vars.resultCount = 0
	self.fsm.vars.results = nil
self.fsm.vars.count = 0
end

function EXPLORE:init()
	self.args["recog_from_align"].level = self.fsm.vars.i
end

function ALIGN1:init()
  self.args["tagless_mps_align"].x1 = 2
  self.args["tagless_mps_align"].y1 = 2
  self.args["tagless_mps_align"].x2 = 3
  self.args["tagless_mps_align"].y2 = 3
  self.args["tagless_mps_align"].ori = 0
  self.fsm.vars.last = 1
end

function ALIGN2:init()
  self.args["tagless_mps_align"].x1 = 2
  self.args["tagless_mps_align"].y1 = 2
  self.args["tagless_mps_align"].x2 = 3
  self.args["tagless_mps_align"].y2 = 3
  self.fsm.vars.last = 2


end

function ALIGN3:init()
  self.fsm.vars.results = {next = self.fsm.vars.results, value = mps_recognition_if:mpstype()};
  self.fsm.vars.count = self.fsm.vars.count + 1
  self.args["tagless_mps_align"].y1 = 2
  self.args["tagless_mps_align"].x2 = 3
  self.args["tagless_mps_align"].y2 = 3
 self.fsm.vars.last = 3

end


function ALIGN4:init()
    self.fsm.vars.results = {next = self.fsm.vars.results, value = mps_recognition_if:mpstype()};
  self.fsm.vars.count = self.fsm.vars.count + 1
  self.args["tagless_mps_align"].x1 = 2
  self.args["tagless_mps_align"].y1 = 2
  self.args["tagless_mps_align"].x2 = 3
  self.args["tagless_mps_align"].y2 = 3
  self.fsm.vars.last = 4

end

function CALCULATE_RESULT:init()
    self.fsm.vars.results = {next = self.fsm.vars.results, value = mps_recognition_if:mpstype()};
  self.fsm.vars.count = self.fsm.vars.count + 1
 
end

function DRIVE1:init()
   self.fsm.vars.i = self.fsm.vars.i * 2
   self.args["goto"].x = self.fsm.vars.xLeft 
   self.args["goto"].y = self.fsm.vars.yZone
   self.args["goto"].ori = 0
end

function DRIVE2:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yDown
   self.args["goto"].ori = 1.57
end

function DRIVE3:init()
   self.args["goto"].x = self.fsm.vars.xRight
   self.args["goto"].y = self.fsm.vars.yZone
   self.args["goto"].ori = 3.1415	
end

function DRIVE4:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yUp
   self.args["goto"].ori = 4.71

end



