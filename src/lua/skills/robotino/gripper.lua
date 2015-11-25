
----------------------------------------------------------------------------
--  gripper.lua - Skill to open or close Robotino gripper
--
--  Created: Thu March 13 15:34:23 2014
--  Copyright  2014  Sebastian Reuter
--             2014  Tim Niemueller
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
name               = "gripper"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "gripper_if", type = "GripperInterface"}
}

documentation      = [==[Skill to open and close gripper.
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- States
fsm:define_states{
   export_to=_M,
   closure={gripper_if=gripper_if},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not gripper_if:has_writer()", desc="No writer for gripper"},
   {"CHECK_WRITER", "COMMAND", cond=true},
   {"COMMAND", "FINAL", cond="not vars.error"},
   {"COMMAND", "FAILED", cond="vars.error"},
}

function COMMAND:init()
   if self.fsm.vars.close then
      gripper_if:msgq_enqueue_copy(gripper_if.CloseGripperMessage:new())
   elseif self.fsm.vars.open then
      gripper_if:msgq_enqueue_copy(gripper_if.OpenGripperMessage:new())
   else
      self.fsm:set_error("No known command")
      self.fsm.vars.error = true
   end
end

