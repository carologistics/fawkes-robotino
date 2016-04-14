
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

--function relgotoz_allowed(self)
--   local cur_z = gripper_arduino_if:z_position()
--   local desired_z = self.fsm.vars.num_steps or 0
--   local upper_bound = gripper_if:z_upper_bound()
--   local lower_bound = gripper_if:z_lower_bound()
--   return (cur_z + desired_z) <= upper_bound and (cur_z + desired_z) >= lower_bound
--end

-- States
fsm:define_states{
   export_to=_M,
   closure={gripper_arduino_if=gripper_arduino_if, right_fully_loaded=right_fully_loaded, left_fully_loaded=left_fully_loaded},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not gripper_arduino_if:has_writer()", desc="No writer for arduino"},
   {"CHECK_WRITER", "COMMAND", cond=true},
   {"COMMAND", "FINAL", cond="vars.moveup or vars.movedown or vars.movezero"},
   {"COMMAND", "FAILED", cond="vars.error"},
}

function COMMAND:init()
   if self.fsm.vars.command == "UP" then
      self.fsm.vars.moveup = true
      theUpMessage = gripper_arduino_if.MoveUpwardsMessage:new()
      theUpMessage:set_num_mm(self.fsm.vars.num_mm or 0)
      gripper_arduino_if:msgq_enqueue_copy(theUpMessage)
   elseif self.fsm.vars.command == "DOWN" then
      self.fsm.vars.movedown = true
      theDownMessage = gripper_arduino_if.MoveDownwardsMessage:new()
      theDownMessage:set_num_mm(self.fsm.vars.num_mm or 0)
      gripper_arduino_if:msgq_enqueue_copy(theDownMessage)
   else
      self.fsm:set_error("No known command")
      self.fsm.vars.error = true
   end
end
