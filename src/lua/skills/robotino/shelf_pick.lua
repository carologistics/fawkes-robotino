
----------------------------------------------------------------------------
--  shelf_pick.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
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
name               = "shelf_pick"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = {
   {v = "line1", type = "LaserLineInterface"},
}

documentation      = [==[ shelf_pick

                          This skill does:
                          - Picks of Shelf                    

                          @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT",       SkillJumpState, skills={{ax12gripper}}, final_to="GOTO_SHELF", fail_to="FAILED" },
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="WAIT_FOR_LASERLINE", fail_to="FAILED"},
   {"APPROACH_SHELF", SkillJumpState, skills={{motor_move}}, final_to="GRAB_PRODUCT", fail_to="FAILED"},
   {"GRAB_PRODUCT", SkillJumpState, skills={{ax12gripper}}, final_to="WAIT_AFTER_GRAB", fail_to="FAILED"},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="CENTER_PUCK", fail_to="FAILED"},
   {"CENTER_PUCK", SkillJumpState, skills={{ax12gripper}}, final_to="FINAL", fail_to="FAILED"},
   {"WAIT_AFTER_GRAB", JumpState},
   {"WAIT_FOR_LASERLINE", JumpState},
}

fsm:add_transitions{
   {"GOTO_SHELF", "FAILED", cond="vars.error"},
   {"WAIT_AFTER_GRAB", "LEAVE_SHELF", timeout=0.5},
   {"WAIT_FOR_LASERLINE", "APPROACH_SHELF", timeout=0.5}
}

function INIT:init()
   self.skills[1].command = "OPEN"
end

function GOTO_SHELF:init()
   local dest_x = 0.2
   local dest_y = line1:end_point_1(1) + 0.075
   local line_offset
   if line1:end_point_1(1) < line1:end_point_2(1) then
      line_offset = line1:end_point_1(1) + 0.075
   else
      line_offset = line1:end_point_2(1) + 0.075
   end
   if self.fsm.vars.slot == "LEFT" then
      dest_y = line_offset + 0.17
   elseif self.fsm.vars.slot == "MIDDLE" then
      dest_y = line_offset + 0.1
   elseif self.fsm.vars.slot == "RIGHT" then
      dest_y = line_offset
   else
      dest_x = 0
      dest_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end
   
   self.skills[1].x = dest_x
   self.skills[1].y = dest_y
--   self.skills[1].y = -0.02 -(self.fsm.vars.slot * 0.094)
end

function APPROACH_SHELF:init()
   self.skills[1].x = line1:point_on_line(0) - 0.08
end

function GRAB_PRODUCT:init()
   self.skills[1].command = "CLOSE"
end

function LEAVE_SHELF:init()
   self.skills[1].x = -0.1
end

function CENTER_PUCK:init()
   self.skills[1].command = "CENTER"
end
