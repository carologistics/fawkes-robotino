----------------------------------------------------------------------------
--  save_realsense_image.lua - Save an image from the rgb camera using librealsense
--
--  Copyright  2020 The Carologistics Team
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
name               = "save_realsense_image"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {}
-- TODO:get interface names from congic
depends_interfaces = {
   {v = "realsense_switch", type = "SwitchInterface", id = "realsense2"},
   {v = "realsense_control", type = "CameraControlInterface", id = "realsense2_cam"},
}

documentation      = [==[Uses librealsense to save a color frame to a PNG image
Parameters:
       @param disable_realsense_afterwards bool  disable the realsense after saving the image
       @param object string object in the image
]==]

-- Initialize as skill module

skillenv.skill_module(_M)

function has_writer()
   return realsense_switch:has_writer() and CameraControlInterface:has_writer()
end

fsm:define_states{ export_to=_M,
   closure={ },
   {"INIT", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond = "vars.object != nil", desc = "Object name is required"},
   {"INIT", "FAILED", cond = not has_writer(), desc = "No Switch or Camera Control Writer"},
   {"INIT", "SAVE_IMAGE", cond = has_writer, desc = "Switch and CameraControl writer enabled" },
   {"SAVE_IMAGE", "FINAL", cond = true, desc = "Saving Image"},
}

function INIT:init()
   realsense_switch:msgq_enqueue(realsense_switch.EnableSwitchMessage:new())
   self.fsm.vars.disable_realsense_afterwards = false
end

function SAVE_IMAGE:init()
   realsense_control:msgq_enqueue(realsense_control.SaveImageMessage:new(self.fsm.vars.object))
end

function cleanup()
   if self.fsm.vars.disable_realsense_afterwards then
      realsense_switch:msgq_enqueue(realsense_switch.DisableSwitchMessage:new())
   end
end

function FAILED:init()
   cleanup()
end

function FINAL:init()
   cleanup()
end

