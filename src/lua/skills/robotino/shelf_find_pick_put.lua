
----------------------------------------------------------------------------
--  shelf_find_pick_put.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--             2016  David Schmidt
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
name               = "shelf_find_pick_put"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "ax12gripper", "approach_mps"}
depends_interfaces = {
   {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
}

documentation      = [==[ shelf_pick

                          This skill does:
                          - Picks of Shelf
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
MPS_LENGTH = 0.7
MPS_WIDTH = 0.35
BOT_RADIUS = 0.46/2
START_DIST_TO_MPS = 0.15+BOT_RADIUS

Y_OFFSET_CONVEYOR = 0.025

D_SHELF_CONVEYOR = 0.1
D_SHELF_SPACE = 0.1

fsm:define_states{ export_to=_M,
   closure={gripper_if=gripper_if},
   {"INIT",       SkillJumpState, skills={{ax12gripper}}, final_to="GOTO_SHELF", fail_to="FAILED" },
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_SHELF", fail_to="FAILED"},
   {"APPROACH_SHELF", SkillJumpState, skills={{approach_mps}}, final_to="GRAB_PRODUCT", fail_to="FAILED"},
   {"GRAB_PRODUCT", SkillJumpState, skills={{ax12gripper}}, final_to="WAIT_AFTER_GRAB", fail_to="FAIL_SAFE"},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="CHECK_PUCK", fail_to="FAILED"},
   {"CENTER_PUCK", SkillJumpState, skills={{ax12gripper}}, final_to="MOVE_INFRONT_CONVEYOR", fail_to="FAILED"},
   {"MOVE_INFRONT_CONVEYOR", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"FAIL_SAFE", SkillJumpState, skills={{motor_move}}, final_to="FAILED", fail_to="FAILED"},
   {"WAIT_AFTER_GRAB", JumpState},
   {"CHECK_PUCK", JumpState},
}

fsm:add_transitions{
   {"GOTO_SHELF", "FAILED", cond="vars.error"},
   {"CHECK_PUCK", "GOTO_SHELF", cond="not gripper_if:is_holds_puck()", desc="Gripper doesn't hold a puck"},
   {"CHECK_PUCK", "CENTER_PUCK", cond="true"},
   {"WAIT_AFTER_GRAB", "LEAVE_SHELF", timeout=0.5},
}

function INIT:init()
   self.args["ax12gripper"].command = "OPEN"
   self.fsm.vars.slot = "LEFT"
end

function GOTO_SHELF:init()
   if self.fsm.vars.slot == "LEFT" then
      dest_y = -Y_OFFSET_CONVEYOR + D_SHELF_CONVEYOR -- offset with "-", because shelf is to the right
      self.fsm.vars.slot = "MIDDLE"
   elseif self.fsm.vars.slot == "MIDDLE" then
      dest_y = D_SHELF_SPACE
      self.fsm.vars.slot = "RIGHT"
   elseif self.fsm.vars.slot == "RIGHT" then
      dest_y = D_SHELF_SPACE
      self.fsm.vars.slot = "ERROR"
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
   self.args["approach_mps"].x = 0.07 --TODO measure this value
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

function MOVE_INFRONT_CONVEYOR:init()
   if self.fsm.vars.slot == "MIDDLE" then
      dest_y = D_SHELF_CONVEYOR
   elseif self.fsm.vars.slot == "RIGHT" then
      dest_y = D_SHELF_CONVEYOR + D_SHELF_SPACE
   elseif self.fsm.vars.slot == "ERROR" then
      dest_y = D_SHELF_CONVEYOR + 2*D_SHELF_SPACE
   else
      dest_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end

   self.args["motor_move"] =
			{ y = dest_y, --shelf is on the right side of the conveyor
				vel_trans = 0.2,
				tolerance = { x=0.002, y=0.002, ori=0.01 }
			}
end

function FAIL_SAFE:init()
   self.args["motor_move"].x = -0.1
end
