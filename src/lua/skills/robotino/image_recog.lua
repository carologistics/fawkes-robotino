----------------------------------------------------------------------------
--  image_recog.lua -check if tag in front is the given tag_id 
--
--  Copyright 2015 The Carologistics Team
--
--  Author : Carsten Stoffels, Daniel Habering
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
name               = "image_recog"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {}
depends_interfaces = {
	{v = "im_recognition_if", type = "ImageRecognitionInterface" ,id="/ImageRecognition"},

}

documentation      = [==[
		
		   	This skill should check if the current picture of the RealSense can be identified by using the image_recognition 
	@param clear Clears the data, if true
]==]


-- Initialize as skill module
skillenv.skill_module(_M)


-- Constants
MPS_TYPES = {
'No Station',
'Base Station',
'Cap S	tation',
'Delivery Station',
'Ring Station',
'Storage Station',
}




function send_takedata(self)
   local msg = im_recognition_if.RecognitionMessage:new()
   im_recognition_if:msgq_enqueue_copy(msg)
end

function send_cleardata()
    -- mps_reocognition_if:msgq_enqueue_copy(mps_recognition_if.ClearMessage:new())
end

function mpsRecogPlugin_ready()
  return im_recognition_if:is_final()
end


function recognition_result(self)
   recognition_result = self.fsm.vars.classes[im_recognition_if:recclass()+1];
   printf("Recognition Result: %s",recognition_result);
   return true;
end

function fileExists(file)
	f = io.open(file,"rb")
	if f then f:close() end
	return f ~= nil
end


function getClasses(self)
	file = os.getenv("HOME")
	count = 2
	file = file .. "/fawkes-robotino/cfg/conf.d/image_recognition.yaml"
	if fileExists(file) then
		lines = {}
		for line in io.lines(file) do
			lines[#lines+1] = line
		end	

		for k,v in pairs(lines) do
			index = string.find(v,"labels")
			if index ~= nil then
				start = string.find(v,"\"")
				fin = string.find(v,"\"",start+1)
				labels = string.sub(v,start+1,fin-1) 
				labels = os.getenv("HOME") .. labels
			end
		end
		
		if fileExists(labels) then
			for line in io.lines(labels) do
				self.fsm.vars.classes[count]= line
				printf("%s",self.fsm.vars.classes[count])
				count = count +1	

			end
			return true
		else
			printf(file .." missing")
			return false
		end

	else
		printf(file .." missing")
		return false
	end
end

function hasWriter(self)
	return im_recognition_if:has_writer()
end

fsm:define_states{ export_to=_M,
   closure={mps_recognition_if=mps_recognition_if}, --,speechsynth=speechsynth},
   {"CHECK_INTERFACE", JumpState},
   {"INIT", JumpState},
   {"CLEAR", JumpState},
   {"TAKEDATA", JumpState},
   {"OUTPUT", JumpState},
}

fsm:add_transitions{
   {"CHECK_INTERFACE", "FAILED", cond=hasWriter, desc="no writer for recognition interface"},
   {"CHECK_INTERFACE", "INIT", cond="true"},
   {"INIT", "CLEAR", cond="self.fsm.vars.clear"},
   {"CLEAR", "FINAL", cond=true},
   {"INIT", "TAKEDATA", cond=mpsRecogPlugin_ready and getClasses},
   {"INIT", "FAILED", cond=true},
   {"TAKEDATA", "OUTPUT", cond=mpsRecogPlugin_ready},
   {"TAKEDATA", "FAILED", timeout=3},
   {"OUTPUT", "FINAL", cond=recognition_result},
   {"OUTPUT", "FAILED", cond=true},
}

function INIT:init()
	self.fsm.vars.classes = {}
	self.fsm.vars.classes[1] = "unknown"
end

function CLEAR:init()
   send_cleardata()
end

function TAKEDATA:init()
   send_takedata(self)
end
