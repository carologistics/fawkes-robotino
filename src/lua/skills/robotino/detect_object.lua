
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
   {v = "yolo_interface_write", type = "YoloOpenCVInterface", id="YoloOpenCVwrite"},
}

documentation      = [==[
sends an image file path to the YoloOpenCV plugin to get the bounding boxes of detected objects

Parameters:
      @param imgpath path to the image to detect objects in
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants

function has_writer()
   return yolo_interface_write:has_writer()
end

function checkid(self)
   return yolo_interface_write:msgid() == self.fsm.vars.msgid
end

fsm:define_states{ export_to=_M, closure={has_writer=has_writer},
   {"INIT", JumpState},
   {"DETECTED", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not has_writer()", desc="YoloOpenCV has no writer"},
   {"INIT", "DETECTED", cond=checkid, desc="Detected objects"},
   {"DETECTED", "FAILED", cond="not vars.detection_successful", desc="vars.feedback_error_msg"},
   {"DETECTED", "FINAL", cond="vars.detection_successful"},
}

function INIT:init()
   local detect_object_request = yolo_interface_write.DetectObjectMessage:new()
   detect_object_request:set_path_to_picture(self.fsm.vars.imgpath)
   self.fsm.vars.request_id = yolo_interface_write:msgq_enqueue(detect_object_request)
   self.fsm.vars.msgid = detect_object_request:id()
end

function DETECTED:init()
   local detect_feedback = nil
   local object_nr = 1
   local index = 1024
   self.fsm.vars.objects = {}
   while (yolo_interface_write:classId(1024 - index) ~= 1024) do
      if yolo_interface_write:is_detection_successful() then
         self.fsm.vars.objects[object_nr] = {c_x = yolo_interface_write:centerX(1024 - index),
                                             c_y = yolo_interface_write:centerY(1024 - index),
                                             h = yolo_interface_write:height(1024 - index),
                                             w = yolo_interface_write:width(1024 - index),
                                             conf = yolo_interface_write:confidence(1024 - index),
                                             class_id = yolo_interface_write:classId(1024 - index)}
       
      else
         self.fsm.vars.detection_successful = false
         self.fsm.vars.feedback_error_msg = yolo_interface_write:error()
      end
      index = index - 1
   end
   self.fsm.vars.detection_successful = true
end

