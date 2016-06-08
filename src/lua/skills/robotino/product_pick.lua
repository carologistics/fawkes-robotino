
----------------------------------------------------------------------------
--  product_pick.lua
--
--  Created Wed Apr 15
--  Copyright  2015  Johannes Rothe
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
name               = "product_pick"
fsm                = SkillHSM:new{name=name, start="OPEN_GRIPPER", debug=false}
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = {
   {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
}

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
@param offset_x the offset_x from the navgraph point
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local x_distance = 0.07
if config:exists("/hardware/robotino/align_distance_conveyor/x") then
   local x_distance = config:get_float("/hardware/robotino/align_distance_conveyor/x")
end

fsm:define_states{ export_to=_M, closure={gripper_if=gripper_if},
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="WAIT_OPEN", fail_to="FAILED"},
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}},
      final_to="CLOSE_GRIPPER", fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="MOVE_BACK", fail_to="FAIL_SAFE"},
   {"WAIT_OPEN", JumpState},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="CLOSE_GRIPPER_SECOND", fail_to="FAILED"},
   {"CLOSE_GRIPPER_SECOND", SkillJumpState, skills={{ax12gripper}},
      final_to="MOVE_BACK_SECOND", fail_to="FAIL_SAFE"},
   {"MOVE_BACK_SECOND", SkillJumpState, skills={{motor_move}},
      final_to="CHECK_PUCK", fail_to="FAILED"},
   {"CHECK_PUCK", JumpState},
   {"CENTER_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="FINAL", fail_to="FAILED"},
   {"FAIL_SAFE", SkillJumpState, skills={{motor_move}},
      final_to="FAILED", fail_to="FAILED"},
}

fsm:add_transitions{
   {"WAIT_OPEN", "DRIVE_FORWARD", timeout=1},
   {"CHECK_PUCK", "CENTER_GRIPPER", cond="gripper_if:is_holds_puck()", desc="Got a puck"},
   {"CHECK_PUCK", "FAILED", cond="not gripper_if:is_holds_puck()", desc="GOT NO PUCK!"},
}

function OPEN_GRIPPER:init()
   self.args["ax12gripper"].command = "OPEN"
   printf("open gripper")
end

function DRIVE_FORWARD:init()
   self.args["motor_move"].x = x_distance
   self.args["motor_move"].vel_trans = 0.2
end

function MOVE_BACK:init()
   self.args["motor_move"] = {x = -0.02, vel_trans = 0.13, tolerance = { x=0.001, y=0.002, ori=0.01 } }
end

function MOVE_BACK_SECOND:init()
   self.args["motor_move"] = {x = -0.195}
end

function CLOSE_GRIPPER:init()
   self.args["ax12gripper"].command = "CLOSE"
   printf("close gripper")
end

function CLOSE_GRIPPER_SECOND:init()
   self.args["ax12gripper"].command = "CLOSE"
   printf("close gripper")
end

function CENTER_GRIPPER:init()
   self.args["ax12gripper"].command = "CENTER"
   printf("center gripper")
end

function FAIL_SAFE:init()
   self.args["ax12gripper"].x = -0.1
end
