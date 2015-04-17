
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
fsm                = SkillHSM:new{name=name, start="DRIVE_FORWARD", debug=true}
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = { }

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local ALIGN_DISTANCE = 0.4 --TODO config value because it depends on the align_tag distance

fsm:define_states{ export_to=_M,
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}},
      final_to="OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="MOVE_BACK", fail_to="FAILED"},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{}

function DRIVE_FORWARD:init()
   local gripper_transformed = tfm.transform({x=ALIGN_DISTANCE, y=0, ori=0}, "/base_link", "/gripper")
   self.skills[1].x = gripper_transformed.x
   self.skills[1].y = gripper_transformed.y --if the gripper has an y offset
   self.skills[1].ori = 0
end

function OPEN_GRIPPER:init()
   self.skills[1].open = true
   self.skills[1].close = false
   printf("open gripper")
end

function MOVE_BACK:init()
   self.skills[1].x = -0.2
end
