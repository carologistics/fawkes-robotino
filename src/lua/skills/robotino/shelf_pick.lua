
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = {}

documentation      = [==[ shelf_pick

                          This skill does:
                          - Picks of Shelf                    

                          @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local x_distance = 0.07
if config:exists("/hardware/robotino/align_distance_conveyor/x") then
   local x_distance = config:get_float("/hardware/robotino/align_distance_conveyor/x")
end

fsm:define_states{ export_to=_M,
   {"INIT",       SkillJumpState, skills={{ax12gripper}}, final_to="GOTO_SHELF", fail_to="FAILED" },
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_SHELF", fail_to="FAILED"},
   {"APPROACH_SHELF", SkillJumpState, skills={{motor_move}}, final_to="GRAB_PRODUCT", fail_to="FAILED"},
   {"GRAB_PRODUCT", SkillJumpState, skills={{ax12gripper}}, final_to="WAIT_AFTER_GRAB", fail_to="FAIL_SAFE"},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="CENTER_PUCK", fail_to="FAILED"},
   {"CENTER_PUCK", SkillJumpState, skills={{ax12gripper}}, final_to="FINAL", fail_to="FAILED"},
   {"FAIL_SAFE", SkillJumpState, skills={{motor_move}}, final_to="FAILED", fail_to="FAILED"},
   {"WAIT_AFTER_GRAB", JumpState},
}

fsm:add_transitions{
   {"GOTO_SHELF", "FAILED", cond="vars.error"},
   {"WAIT_AFTER_GRAB", "LEAVE_SHELF", timeout=0.5},
}

function INIT:init()
   self.args["ax12gripper"].command = "OPEN"
end

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
   
   self.args["motor_move"] =
			{ y = -dest_y, --shelf is on the right side of the conveyor
				vel_trans = 0.2,
				tolerance = { x=0.002, y=0.002, ori=0.01 }
			}
end

function APPROACH_SHELF:init()
   self.args["motor_move"].x = x_distance
   self.args["motor_move"].vel_trans = 0.2
end

function GRAB_PRODUCT:init()
   self.args["ax12gripper"].command = "CLOSE"
end

function LEAVE_SHELF:init()
   self.args["motor_move"].x = -0.2
end

function CENTER_PUCK:init()
   self.args["ax12gripper"].command = "CENTER"
end

function FAIL_SAFE:init()
   self.args["motor_move"].x = -0.1
end
