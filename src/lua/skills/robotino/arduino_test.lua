
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
name               = "arduino_test"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "gripper_arduino_if", type = "ArduinoInterface", id="Arduino"}
}

documentation      = [==[Skill to test the new Grbl Arduino Combination
@param command    can be only ABSMOVE at the moment
@param x          The x position to go to
@param y          The y position to go to
@param z          The z position to go to
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- States
fsm:define_states{
   export_to=_M,
   closure={gripper_arduino_if=gripper_arduino_if},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not gripper_arduino_if:has_writer()", desc="No writer for gripper"},
   {"CHECK_WRITER", "COMMAND", cond=true},
   {"COMMAND", "FAILED", cond="vars.error"},
}

function COMMAND:init()

   if self.fsm.vars.command == "ABSMOVE" then
     theMoveXYZAbsMessage = gripper_arduino_if.MoveXYZAbsMessage:new()
     theMoveXYZAbsMessage:set_x(self.fsm.vars.x or 1.0)
     theMoveXYZAbsMessage:set_y(self.fsm.vars.y or 1.0)
     theMoveXYZAbsMessage:set_z(self.fsm.vars.z or 1.0)
     gripper_arduino_if:msgq_enqueue_copy(theMoveXYZAbsMessage)
   else
     self.fsm:set_error("No known command")
     self.fsm.vars.error = true
   end

end
