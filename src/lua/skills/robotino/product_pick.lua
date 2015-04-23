
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
fsm                = SkillHSM:new{name=name, start="OPEN_GRIPPER", debug=true}
depends_skills     = {"motor_move", "ax12gripper", "approach_mps"}
depends_interfaces = { 
   {v = "sensor", type="RobotinoSensorInterface"}
}

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
@param place Navgraph place to get the align_distance
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")

fsm:define_states{ export_to=_M, closure={sensor=sensor},
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="APPROACH_MPS", fail_to="FAILED"},
   {"APPROACH_MPS", SkillJumpState, skills={{approach_mps}},
      final_to="CLOSE_GRIPPER", fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="WAIT", fail_to="FAILED"},
   {"WAIT", JumpState},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="CENTER_GRIPPER", fail_to="FAILED"},
   {"CENTER_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"WAIT", "MOVE_BACK", timeout=0.5},
}

function OPEN_GRIPPER:init()
   self.skills[1].command = "OPEN"
   printf("open gripper")
end

function MOVE_BACK:init()
   self.skills[1].x = -0.2
end

function CLOSE_GRIPPER:init()
   self.skills[1].command = "CLOSE"
   printf("close gripper")
end

function CENTER_GRIPPER:init()
   self.skills[1].command = "CENTER"
   printf("center gripper")
end
