
----------------------------------------------------------------------------
--  gripper.lua - Skill to open or close Robotino AX12 gripper
--
--  Created: Sat Feb 28 10:46:33 2015
--  Copyright  2014  Sebastian Reuter
--             2014  Tim Niemueller
--             2015  Nicolas Limpert
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
name               = "ax12gripper"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=true}
depends_skills     = nil
depends_interfaces = {
   {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
}

documentation      = [==[Skill to open and close AX12 - gripper.
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- States
fsm:define_states{
   export_to=_M,
   closure={gripper_if=gripper_if, right_fully_loaded=right_fully_loaded, left_fully_loaded=left_fully_loaded},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
   {"CLOSE_GRIPPER_WAIT", JumpState},
}

function is_right_full_loaded()
   fsm.vars.right_fully_loaded = gripper_if:right_load() > 150
   print("load_right: " .. gripper_if:right_load())
   return fsm.vars.right_fully_loaded
end

function is_left_full_loaded()
   fsm.vars.left_fully_loaded = gripper_if:left_load() > 150
   print("load_left: " .. gripper_if:left_load())
   return fsm.vars.left_fully_loaded
end

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not gripper_if:has_writer()", desc="No writer for gripper"},
   {"CHECK_WRITER", "COMMAND", cond=true},
   {"COMMAND", "FINAL", cond="not vars.error"},
   {"COMMAND", "FAILED", cond="vars.error"},
   {"COMMAND", "CLOSE_GRIPPER_WAIT", cond="vars.close_load"},
}

function CLOSE_GRIPPER_WAIT:init()
   self.fsm.vars.right_fully_loaded = false
   self.fsm.vars.left_fully_loaded = false
   gripper_if:msgq_enqueue_copy(gripper_if.CloseLoadMessage:new())
   -- print("right: " .. self.fsm.vars.right_fully_loaded .. " left: " .. self.fsm.vars.left_fully_loaded)
end

function COMMAND:init()
   -- if self.fsm.vars.close then
   --    theCloseMessage = gripper_if.CloseMessage:new()
   --    theCloseMessage:set_offset(self.fsm.vars.offset)
   --    gripper_if:msgq_enqueue_copy(theCloseMessage)
   -- elseif self.fsm.vars.open then
   if self.fsm.vars.open then
--      gripper_if:msgq_enqueue_copy(gripper_if.OpenMessage:new())
      theOpenMessage = gripper_if.OpenMessage:new()
      theOpenMessage:set_offset(self.fsm.vars.offset)
      gripper_if:msgq_enqueue_copy(theOpenMessage)
   elseif self.fsm.vars.close then
      print("close")
      theCloseMessage = gripper_if.CloseMessage:new()
      theCloseMessage:set_offset(self.fsm.vars.offset)
      gripper_if:msgq_enqueue_copy(theCloseMessage)
   elseif self.fsm.vars.close_load then
      print("close load")
      theCloseLoadMessage = gripper_if.CloseLoadMessage:new()
      gripper_if:msgq_enqueue_copy(theCloseLoadMessage)
-- Set servo position by ID and desired angle
   elseif self.fsm.vars.id and self.fsm.vars.angle then
      local id = self.fsm.vars.id
      local angle = self.fsm.vars.angle
      print("Set servo " .. id .. " to " .. angle)
      theSetServoMessage = gripper_if.SetServoMessage:new()
      theSetServoMessage:set_servoID(id)
      theSetServoMessage:set_angle(angle)
      gripper_if:msgq_enqueue_copy(theSetServoMessage)
   elseif self.fsm.vars.enable then
      local enable = self.fsm.vars.enable
      local value = self.fsm.vars.value
      theSetEnabledMessage = gripper_if.SetEnabledMessage:new()
      theSetEnabledMessage:set_enabled(value)
      print("set enabled to " .. tostring(value))
      gripper_if:msgq_enqueue_copy(theSetEnabledMessage)
   else
      self.fsm:set_error("No known command")
      self.fsm.vars.error = true
   end
end
