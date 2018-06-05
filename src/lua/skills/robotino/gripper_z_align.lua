
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
name               = "gripper_z_align"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "gripper_arduino_if", type = "ArduinoInterface", id="Arduino"}
}

documentation      = [==[Skill to modify the z position of the AX12 gripper by using a stepper motor.
@param command    can be one of UP or DOWN (UP or DOWN require the desired number of millimeters)
@param num_mm     only used with the UP or DOWN-commands - the desired relative position in mm.
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local max_mm = 0.0

-- States
fsm:define_states{
   export_to=_M,
   closure={gripper_arduino_if=gripper_arduino_if},
   {"CHECK_WRITER", JumpState},
   {"WAIT_COMMAND", JumpState},
   {"COMMAND", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not gripper_arduino_if:has_writer()", desc="No writer for arduino"},
   {"CHECK_WRITER", "COMMAND", cond=true},
   {"COMMAND", "WAIT_COMMAND", timeout=1.0},
   {"WAIT_COMMAND", "FINAL", cond="vars.restore"},
   {"WAIT_COMMAND", "FINAL", cond="gripper_arduino_if:is_final()"},
   {"WAIT_COMMAND", "FAILED", cond="vars.error"},
}

function COMMAND:init()
   if config:exists("/arduino/z_max") then
      max_mm = 1000 * config:get_float("/arduino/z_max")
   end
   if self.fsm.vars.command == "UP" then
      if gripper_arduino_if:z_position() - self.fsm.vars.num_mm < 0 then
         self.fsm:set_error("desired position out of bounds: " .. gripper_arduino_if:z_position() - self.fsm.vars.num_mm .. " < 0")
	 self.fsm.vars.error = true
      else
         theMoveXYZRelMessage = gripper_arduino_if.MoveXYZRelMessage:new()
         theMoveXYZRelMessage:set_z(-self.fsm.vars.num_mm or 0)
         gripper_arduino_if:msgq_enqueue_copy(theMoveXYZRelMessage)
      end
   elseif self.fsm.vars.command == "DOWN" then
      if gripper_arduino_if:z_position() + self.fsm.vars.num_mm > max_mm then
         self.fsm:set_error("desired position out of bounds: " .. gripper_arduino_if:z_position() + self.fsm.vars.num_mm .. " > " .. max_mm)
	 self.fsm.vars.error = true
      else
         theMoveXYZRelMessage = gripper_arduino_if.MoveXYZRelMessage:new()
         theMoveXYZRelMessage:set_z(self.fsm.vars.num_mm or 0)
         gripper_arduino_if:msgq_enqueue_copy(theMoveXYZRelMessage)
      end
   elseif self.fsm.vars.command == "TO_UPPER_Z" then
      theToHomeMessage = gripper_arduino_if.ToHomeMessage:new()
      gripper_arduino_if:msgq_enqueue_copy(theToHomeMessage)
   elseif self.fsm.vars.command == "RESET_Z_POS" then
      self.fsm.vars.restore = true
      theCalibrateMessage = gripper_arduino_if.CalibrateMessage:new()
      gripper_arduino_if:msgq_enqueue_copy(theCalibrateMessage)
   else
      self.fsm:set_error("No known command")
      self.fsm.vars.error = true
   end
end
