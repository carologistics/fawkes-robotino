
----------------------------------------------------------------------------
--  shelf_pick.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--                   Nicolas Limpert
--                   Johannes Rothe
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
name               = "shelf_put"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move", "ax12gripper", "approach_mps"}
depends_interfaces = {}

documentation      = [==[ shelf_put

                          This skill does:
                          - Puts a product on the shelf                    
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_SHELF", fail_to="FAILED"},
   {"APPROACH_SHELF", SkillJumpState, skills={{approach_mps}}, final_to="STORE_PRODUCT", fail_to="FAILED"},
   {"STORE_PRODUCT", SkillJumpState, skills={{ax12gripper}}, final_to="WAIT_AFTER_GRAB", fail_to="FAILED"},
   {"WAIT_AFTER_GRAB", JumpState},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "GOTO_SHELF", cond=true},
   {"WAIT_AFTER_GRAB", "LEAVE_SHELF", timeout=0.5},
}


function GOTO_SHELF:init()
   local shelf_to_conveyor = 0.09 --TODO measure both values
   local shelf_distance = 0.09
   if self.fsm.vars.slot == "LEFT" then
      dest_y = shelf_to_conveyor
   elseif self.fsm.vars.slot == "MIDDLE" then
      dest_y = shelf_to_conveyor + shelf_distance
   elseif self.fsm.vars.slot == "RIGHT" then
      dest_y = shelf_to_conveyor + 2*shelf_distance
   else
      dest_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end
   
   self.skills[1].y = -dest_y --shelf is on the right side of the conveyor
   self.skills[1].vel_trans = 0.2
   self.skills[1].tolerance = { x=0.002, y=0.002, ori=0.01 }
end

function APPROACH_SHELF:init()
   self.skills[1].x = 0.065 --TODO measure this value
end

function STORE_PRODUCT:init()
   self.skills[1].command = "OPEN"
end

function LEAVE_SHELF:init()
   self.skills[1].x = -0.1
end
