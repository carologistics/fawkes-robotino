
----------------------------------------------------------------------------
--  detect_object.lua
--
--  Created: Wed Apr 21
--  Copyright  2021 Sebastian Eltester
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
name               = "detect_object"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {}
depends_interfaces = {
   {v = "yolo-interface", type = "YoloOpenCVInterface", id="YoloOpenCV"},
}

documentation      = [==[
sends an image file path to the YoloOpenCV plugin to get the bounding boxes of detected objects

Parameters:
      @param imgpath path to the image to detect objects in
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants

fsm:define_states{ export_to=_M, closure={},
   {"INIT", JumpState},
   {"DETECT", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not yolo-interface:has_writer()", desc="YoloOpenCV has no writer"},
   {"INIT", "DETECT", cond=true, desc="Detect objects"},
   {"DETECT", "FAILED", cond="not vars.detection_successful", desc="vars.feedback_error_msg"},
   {"DETECT", "FINAL", cond="vars.detection_successful"},
}

function INIT:init()
   local detect_object_request = yolo-interface.DetectObjectMessage:new()
   detect_object_request:set_path_to_picture(self.fsm.vars.imgpath)
   self.fsm.vars.request_id = yolo-interface:msgq_enqueue(detect_object_request)
end

function DETECT:init()
   local detect_feedback = nil
   while (not yolo-interface:empty()) do
      detect_feedback = yolo-interface.msgq_first()
      self.fsm.vars.feedback_error_msg = detect_feedback:error_message()
      self.fsm.vars.detection_successful = detect_feedback:is_detection_successful()
   end
end


