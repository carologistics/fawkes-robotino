----------------------------------------------------------------------------
--  check_tag.lua -check if tag in front is the given tag_id 
--
--  Copyright 2015 The Carologistics Team
--
--  Author : Carsten Stoffels, Daniel Habering,Sebastian Schönitz
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

-- Initialize modul
module(..., skillenv.module_init)

-- Crucial skill information
name               = "mps_recog"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {}
depends_interfaces = {
	{v = "mps_recognition_if", type = "MPSRecognitionInterface" ,id="/MarkerlessRecognition"},
--	{v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"},

}

documentation      = [==[
This skill should check if the current picture of the RealSense can be identified by a machine using the mps_recog plugin 

]==]


-- Initialize as skill module
skillenv.skill_module(_M)


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
   --speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(string.format(unpack(arg))))
    printf(unpack(arg))
end

function send_takedata(self)
   local msg = mps_recognition_if.ComputeMessage:new()
   mps_recognition_if:msgq_enqueue_copy(msg)
end

function send_cleardata()
    -- mps_reocognition_if:msgq_enqueue_copy(mps_recognition_if.ClearMessage:new())
end

function mpsRecogPlugin_ready()
  return mps_recognition_if:is_final()
  --return true;
end

function data_evaluated(self)
   if not mpsRecogPlugin_ready() then
      return false
   end
   return true
end

function recognition_result()
   --recognition_result=mps_recognition_if:toString_MPSType(mps_recognition_if:mpstype());
   --recognition_result="testStation"; 
   recognition_result = MPS_TYPES[mps_recognition_if:mpstype()+1];
   speak("Recognition completed! Result: %s",MPS_TYPES[mps_recognition_if:mpstype()+1])

   printf("The result is %s",recognition_result);
  
   return true;
end


fsm:define_states{ export_to=_M,
   closure={mps_recognition_if=mps_recognition_if}, --,speechsynth=speechsynth},
   {"CHECK_INTERFACE", JumpState},
   {"INIT", JumpState},
   {"CLEAR", JumpState},
   {"TAKEDATA", JumpState},
   {"WAIT", JumpState},
   {"OUTPUT", JumpState},
}


fsm:add_transitions{
   {"CHECK_INTERFACE", "FAILED", cond="not mps_recognition_if:has_writer()", desc="no writer for recognition interface"},
   {"CHECK_INTERFACE", "INIT", cond="true"},
   {"INIT", "CLEAR", cond="self.fsm.vars.clear"},
   {"CLEAR", "TAKEDATA", cond=true},
   {"INIT", "TAKEDATA", cond=mpsRecogPlugin_ready},
   {"TAKEDATA", "WAIT", timeout=0.2},
   {"WAIT", "OUTPUT", cond=data_evaluated},
   {"WAIT", "WAIT", timeout=1},
   {"OUTPUT", "FINAL", cond=recognition_result},
   {"OUTPUT", "FAILED", cond=true},
}


function INIT:init()
   if self.fsm.vars.clear == nil then
      self.fsm.vars.clear = true
   end
end

function CLEAR:init()
   send_cleardata()
end

function TAKEDATA:init()
   send_takedata(self)
end
