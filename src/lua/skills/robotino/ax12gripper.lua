
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
@param command    can be one of OPEN, CLOSE, CENTER or RELGOTOZ (RELGOTOZ requires the z_position parameter to be set)
@param z_position only used with the RELGOTOZ-command - the desired relative position in mm.
                  The skill fails when a desired relative z position is set that would lead out of the grippers z-bounds
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function relgotoz_allowed(self)
   local cur_z = gripper_if:z_position()
   local desired_z = self.fsm.vars.z_position
   local upper_bound = gripper_if:z_upper_bound()
   local lower_bound = gripper_if:z_lower_bound()
   return (cur_z + desired_z) <= upper_bound and (cur_z + desired_z) >= lower_bound
end

-- States
fsm:define_states{
   export_to=_M,
   closure={gripper_if=gripper_if, right_fully_loaded=right_fully_loaded, left_fully_loaded=left_fully_loaded},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
   {"CLOSE_GRIPPER_WAIT", JumpState},
   {"WAIT_FOR_GRAB", JumpState},
   {"CHECK_GRAB_SUCCESS", JumpState},
   {"WAIT_FOR_RELGOTOZ", JumpState},
   {"FINAL_AFTER_IF_FINAL", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not gripper_if:has_writer()", desc="No writer for gripper"},
   {"CHECK_WRITER", "COMMAND", cond=true},
   {"COMMAND", "FINAL", cond="vars.open or vars.center"},
   {"COMMAND", "FAILED", cond="vars.error"},
   {"COMMAND", "WAIT_FOR_GRAB", cond="vars.grab"},
   {"COMMAND", "WAIT_FOR_GRAB", cond="vars.grab"},
   {"COMMAND", "WAIT_FOR_RELGOTOZ", cond="vars.relgotoz"},
   {"WAIT_FOR_RELGOTOZ", "FINAL_AFTER_IF_FINAL", timeout=0.5},
   {"WAIT_FOR_GRAB", "CHECK_GRAB_SUCCESS", timeout=1.5},
   {"CHECK_GRAB_SUCCESS", "FINAL", cond="gripper_if:is_holds_puck()"},
   {"CHECK_GRAB_SUCCESS", "FAILED", cond="not gripper_if:is_holds_puck()", desc="Gripper doesn't hold a puck"},
   {"CHECK_GRAB_SUCCESS", "FAILED", timeout=5, desc="Gripper timeout"},
   {"COMMAND", "CLOSE_GRIPPER_WAIT", cond="vars.close"},
   {"CLOSE_GRIPPER_WAIT", "FINAL_AFTER_IF_FINAL", timeout=0.5},
   {"FINAL_AFTER_IF_FINAL", "FINAL", cond="gripper_if:is_final()"},
}

function COMMAND:init()
   if self.fsm.vars.command == "OPEN" then
      self.fsm.vars.open = true
      theOpenMessage = gripper_if.OpenMessage:new()
      theOpenMessage:set_offset(self.fsm.vars.offset or 0)
      gripper_if:msgq_enqueue_copy(theOpenMessage)
   elseif self.fsm.vars.command == "CENTER" then
      self.fsm.vars.center = true
      theCenterMessage = gripper_if.CenterMessage:new()
      gripper_if:msgq_enqueue_copy(theCenterMessage)
   elseif self.fsm.vars.command == "CLOSE" then
      self.fsm.vars.close = true
      theCloseMessage = gripper_if.CloseMessage:new()
      theCloseMessage:set_offset(self.fsm.vars.offset or 0)
      gripper_if:msgq_enqueue_copy(theCloseMessage)
   elseif self.fsm.vars.command == "GRAB" then
      self.fsm.vars.grab = true
      theCloseMessage = gripper_if.CloseMessage:new()
      theCloseMessage:set_offset(self.fsm.vars.offset or 0)
      gripper_if:msgq_enqueue_copy(theCloseMessage)
   elseif self.fsm.vars.command == "RELGOTOZ" then
      self.fsm.vars.relgotoz = true
      if relgotoz_allowed(self) then
         theRelGotoZMessage = gripper_if.RelGotoZMessage:new()
         theRelGotoZMessage:set_rel_z(self.fsm.vars.z_position or 0)
         gripper_if:msgq_enqueue_copy(theRelGotoZMessage)
      else
         self.fsm:set_error("Desired z value out of bounds")
         self.fsm.vars.error = true
      end
   else
      self.fsm:set_error("No known command")
      self.fsm.vars.error = true
   end
end
