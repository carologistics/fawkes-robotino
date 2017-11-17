
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
depends_skills     = {"motor_move", "ax12gripper", "approach_mps"}
depends_interfaces = {
   {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
}

documentation      = [==[ shelf_pick

                          This skill does:
                          - Picks of Shelf                    

                          @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local x_distance = 0.27
if config:exists("/skills/approach_distance_laser/x") then
   x_distance = config:get_float("/skills/approach_distance_laser/x")
end

fsm:define_states{ export_to=_M, closure={gripper_if=gripper_if},
   {"INIT",       SkillJumpState, skills={{ax12gripper}}, final_to="GOTO_SHELF", fail_to="FAILED" },
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="ADJUST_HEIGHT", fail_to="FAILED"},
   {"ADJUST_HEIGHT",       SkillJumpState, skills={{ax12gripper}}, final_to="APPROACH_SHELF", fail_to="FAILED" },
   {"APPROACH_SHELF", SkillJumpState, skills={{approach_mps}}, final_to="GRAB_PRODUCT", fail_to="FAILED"},
   {"GRAB_PRODUCT", SkillJumpState, skills={{ax12gripper}}, final_to="WAIT_AFTER_GRAB", fail_to="FAIL_SAFE"},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="WAIT_FOR_INTERFACE", fail_to="FAILED"},
   {"CENTER_PUCK", SkillJumpState, skills={{ax12gripper}}, final_to="RESET_Z_POS", fail_to="FAILED"},
   {"RESET_Z_POS", SkillJumpState, skills={{ax12gripper}}, final_to="FINAL", fail_to="FAILED"},
   {"FAIL_SAFE", SkillJumpState, skills={{motor_move}}, final_to="FAILED", fail_to="FAILED"},
   {"WAIT_AFTER_GRAB", JumpState},
   {"WAIT_FOR_INTERFACE", JumpState},
   {"CHECK_PUCK", JumpState},
}

fsm:add_transitions{
   {"GOTO_SHELF", "FAILED", cond="vars.error"},
   {"WAIT_AFTER_GRAB", "LEAVE_SHELF", timeout=0.5},
   {"WAIT_FOR_INTERFACE", "CHECK_PUCK", timeout=2},
   {"CHECK_PUCK", "CENTER_PUCK", cond="gripper_if:is_holds_puck()", desc="Got a puck"},
   {"CHECK_PUCK", "FAILED", cond="not gripper_if:is_holds_puck()", desc="GOT NO PUCK!"},
}

function INIT:init()
   self.args["ax12gripper"].command = "OPEN"
end

function GOTO_SHELF:init()
   local shelf_to_conveyor = 0.0 --TODO measure both values
   local shelf_distance = 0.1
   if self.fsm.vars.slot == "LEFT" then
      dest_y = shelf_to_conveyor - shelf_distance
   elseif self.fsm.vars.slot == "MIDDLE" then
      dest_y = shelf_to_conveyor
   elseif self.fsm.vars.slot == "RIGHT" then
      dest_y = shelf_to_conveyor + shelf_distance
   else
      dest_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end
   
   self.args["motor_move"] =
			{ y = -dest_y, --shelf is on the right side of the conveyor
				tolerance = { x=0.002, y=0.002, ori=0.01 }
			}
end

function ADJUST_HEIGHT:init()
   self.args["ax12gripper"].command = "RELGOTOZ"
   self.args["ax12gripper"].z_position = -4
   printf("adjusting height")
end


function APPROACH_SHELF:init()
   self.args["approach_mps"].x = x_distance
   self.args["approach_mps"].use_conveyor = false
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

function RESET_Z_POS:init()
   self.args["ax12gripper"].command = "RESET_Z_POS"
end

function FAIL_SAFE:init()
   self.args["motor_move"].x = -0.1
end
