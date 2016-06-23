
----------------------------------------------------------------------------
--  product_put.lua
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
name               = "product_put"
fsm                = SkillHSM:new{name=name, start="DRIVE_FORWARD", debug=false}
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = { }

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
@param offset_x the offset_x from the navgraph point
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local x_distance = 0.071
if config:exists("/hardware/robotino/align_distance_conveyor/x") then
      x_distance = config:get_float("/hardware/robotino/align_distance_conveyor/x") + 0.01
end

fsm:define_states{ export_to=_M,
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}},
      final_to="OPEN_GRIPPER", fail_to="FAILED"},
   --{"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
   --   final_to="WAIT", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="WAIT", fail_to="WAIT"},
   {"WAIT", JumpState},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"WAIT", "MOVE_BACK", timeout=0.5, desc="wait for gripper to open"}
}

function DRIVE_FORWARD:init()
   self.args["motor_move"].x = x_distance
   self.args["motor_move"].vel_trans = 0.2
end

function OPEN_GRIPPER:init()
   self.args["ax12gripper"].command = "OPEN"
   printf("open gripper")
end

function MOVE_BACK:init()
   self.args["motor_move"].x = -0.2
end
