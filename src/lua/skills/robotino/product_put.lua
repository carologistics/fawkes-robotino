
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
fsm                = SkillHSM:new{name=name, start="APPROACH_MPS", debug=true}
depends_skills     = {"motor_move", "ax12gripper", "approach_mps"}
depends_interfaces = { }

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
@param place Navgraph place to get the align_distance
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")

fsm:define_states{ export_to=_M,
   {"APPROACH_MPS", SkillJumpState, skills={{approach_mps}},
      final_to="OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="WAIT", fail_to="FAILED"},
   {"WAIT", JumpState},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"WAIT", "MOVE_BACK", timeout=0.5, desc="wait for gripper to open"}
}

function OPEN_GRIPPER:init()
   self.skills[1].open = true
   self.skills[1].close = false
   printf("open gripper")
end

function MOVE_BACK:init()
   self.skills[1].x = -0.2
end
