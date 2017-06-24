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
	{v = "recognition-interface", type = "MPSRecognitionInterface" ,id="/mps-recognition"},
}

documentation      = [==[
This skill should check if the current picture of the RealSense can be identified by a machine using the mps_recog plugin 

]==]


-- Initialize as skill module
skillenv.skill_module(_M)


-- Constants
MPS_TYPES = {
'No Station',
'Base Stationn',
'Cap Station',
'Delivery Station',
'Ring Station',
'Storage Station',
'Not Clear', 
'Ring or Cap',
}



function speak(...)
   speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(string.format(unpack(arg))))
   printf(unpack(arg))
end

function send_takedata(self)
   local msg = mps_recognition_if.TakeDataMessage:new()
   recognition-interface:msgq_enqueue_copy(msg)
end

function send_cleardata()
   recognition-interface:msgq_enqueue_copy(mps_recognition_if.ClearMessage:new())
end

function mpsRecogPlugin_ready()
   return recognition-interface:is_final()
end

function data_evaluated(self)
   if not mpsRecogPlugin_ready() then
      return false
   end
   return true
end

function recognition_result()
   string recognition_result =  recognition-interface:mpstype();
   speak("The result is %s",recognition_result);
   return true;
end


fsm:define_states{ export_to=_M,
   closure={mps_recognition_if=mps_recognition_if,speechsynth=speechsynth},
   {"CHECK_INTERFACE", JumpState},
   {"INIT", JumpState},
   {"CLEAR", JumpState},
   {"TAKEDATA", JumpState},
   {"WAIT", JumpState},
   {"DECIDE", JumpState},
}


fsm:add_transitions{
   {"CHECK_INTERFACE", "FAILED", cond="not mps_recognition_if:has_writer()", desc="no writer for recognition interface"},
   {"CHECK_INTERFACE", "INIT", cond="true"},
   {"INIT", "CLEAR", cond="self.fsm.vars.clear"},
   {"CLEAR", "TAKEDATA", cond=mpsRecogPlugin_ready},
   {"INIT", "TAKEDATA", cond=mpsRecogPlugin_ready},
   {"TAKEDATA", "WAIT", timeout=0.2},
   {"WAIT", "DECIDE", cond=data_evaluated},
   {"WAIT", "WAIT", timeout=1},
   {"DECIDE", "FINAL", cond=recognition_result},
   {"DECIDE", "FAILED", cond=true},
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
