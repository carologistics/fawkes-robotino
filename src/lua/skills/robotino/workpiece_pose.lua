----------------------------------------------------------------------------
--  workpiece_pose.lua
--
--  Created: Sun Jun 06
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


-- get_product_from gives shelf position left middle right
-- detect object gives all objects with classId, pixel pos and confidences
-- 
-- in workpiece_pose skill:
-- determine left, middle, right pixel pos (currently only for shelf) (known from argument)
-- then send to realsense plugin
-- realsense writes XYZ from pixel to Realsense2 interface translation
-- workpiece pose plugin gives tf frame from xyz
--

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "workpiece_pose"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"detect_object"}
depends_interfaces = {
   {v = "yolo_interface", type = "YoloOpenCVInterface", id = "YoloOpenCVwrite"},
   {v = "realsense_control", type = "CameraControlInterface", id = "realsense2_cam"},
   {v = "realsense_switch", type = "SwitchInterface", id = "realsense2"},
   {v = "realsense_boundingbox", type = "Realsense2Interface", id = "BoundingBoxInterface"},
   {v = "workpiece_pose", type = "WorkpiecePoseInterface", id = "WorkpiecePose"},
}

documentation      = [==[
Reads bounding box from yolo, gets XYZ from ROI in realsense depth frame, publishes workpiece frame
Parameters:
      @param location can be: (CONVEYOR | LEFT | MIDDLE | RIGHT) 
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local LEFT = {x = { min = 0, max = 0}, y = { min = 0, max = 0}}
local MIDDLE = {x = { min = 264, max = 371}, y = { min = 200, max = 280}}
local RIGHT = {x = { min = 0, max = 0}, y = { min = 0, max = 0}}

-- TBD
local CONVEYOR = {x = { min = 0, max = 0}, y = { min = 0, max = 0}}

local tolerances = {
         ["CONVEYOR"] = CONVEYOR,
         ["LEFT"] = LEFT,
         ["MIDDLE"] = MIDDLE,
         ["RIGHT"] = RIGHT,
         }

function has_writer()
   return (yolo_interface:has_writer() and
           realsense_control:has_writer() and
           realsense_switch:has_writer() and
           realsense_boundingbox:has_writer())
end

function check_bounding_box_if_msgid(self)
   return self.fsm.vars.rs_bb_if_msgid == realsense_boundingbox:msgid()
end

fsm:define_states{ export_to=_M, closure={has_writer=has_writer, 
                   check_bounding_box_if_msgid=check_bounding_box_if_msgid},
   {"INIT", JumpState},
   {"DETECT_WORKPIECE", SkillJumpState, skills={{"detect_object"}}, final_to="GET_XYZ", fail_to="FAILED"},
   {"GET_XYZ", JumpState},
   {"PUBLISH_POSE", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not has_writer()", desc="YoloOpenCV or Realsense Interface has no writer"},
   {"INIT", "DETECT_WORKPIECE", cond=true, desc="Detecting Workpiece"},
   {"GET_XYZ", "FAILED", cond="vars.low_conf_or_out_of_range", 
      desc="Confindences too low or Bounding box out of range"},
   {"GET_XYZ", "PUBLISH_POSE", cond="check_bounding_box_if_msgid(self)", desc="Deprojected pixel to point"},
   {"PUBLISH_POSE", "FAILED", cond="vars.zero_pose", desc="Can't get XYZ, deprojection returned 0,0,0"},
   {"PUBLISH_POSE", "FINAL", cond=true},
}

function INIT:init()
   self.fsm.vars.zero_pose = nil
   self.fsm.vars.got_xyz = nil
   self.fsm.vars.rs_bb_if_msgid = nil
   self.fsm.vars.low_conf_or_out_of_range = nil
   self.fsm.vars.classId = 0
   self.fsm.vars.objects = {}
   realsense_switch:msgq_enqueue(realsense_switch.EnableSwitchMessage:new())
end

function DETECT_WORKPIECE:init()
   self.args["detect_object"].classId = self.fsm.vars.classId
end

function GET_XYZ:init()
   local object_nr = 0
   local index = 1024
   local found_bb =  false
   while (yolo_interface:classId(1024 - index) ~= 1024) do
      if yolo_interface:classId(1024 - index) == self.fsm.vars.classId then
         if (tolerances[self.fsm.vars.location].x.min <= yolo_interface:centerX(1024 - index) and
             tolerances[self.fsm.vars.location].x.max >= yolo_interface:centerX(1024 - index) and
             tolerances[self.fsm.vars.location].y.min <= yolo_interface:centerY(1024 - index) and
             tolerances[self.fsm.vars.location].y.max >= yolo_interface:centerY(1024 - index) and
             yolo_interface:confidence(1024 - index) > 0.95) then
               bbox_msg = realsense_boundingbox.BoundingBoxMessage:new()
               bbox_msg:set_center_x(yolo_interface:centerX(1024 - index))
               bbox_msg:set_center_y(yolo_interface:centerY(1024 - index))
               bbox_msg:set_height(yolo_interface:height(1024 - index))
               bbox_msg:set_width(yolo_interface:width(1024 - index))
               self.fsm.vars.rs_bb_if_msgid = realsense_boundingbox:msgq_enqueue(bbox_msg)
               found_bb = true
         end
      end
      index = index - 1
   end
   if not found_bb then
      self.fsm.vars.low_conf_or_out_of_range = true
   end
end

function PUBLISH_POSE:init()
   local x = realsense_boundingbox:translation(0)
   local y = realsense_boundingbox:translation(1)
   local z = realsense_boundingbox:translation(2)
   if x == 0 and y == 0 and z == 0 then
      self.fsm.vars.zero_pose = true
   else
      wp_pose_msg = workpiece_pose.WPPoseMessage:new()
      wp_pose_msg:set_translation(0, x)
      wp_pose_msg:set_translation(1, y)
      wp_pose_msg:set_translation(2, z)
      workpiece_pose:msgq_enqueue(wp_pose_msg)
   end
end
