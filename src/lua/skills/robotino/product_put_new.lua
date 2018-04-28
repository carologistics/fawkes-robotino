
----------------------------------------------------------------------------
--  product_put_new.lua
--
--  Created Wed Apr 15
--  Copyright  2015  Johannes Rothe
--  Copyright  2018  Carsten Stoffels
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
name               = "product_put_new"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "gripper_commands_new"}
depends_interfaces = { }

documentation      = [==[

   TODO: make documentation
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

local x_move_back = -0.2
local gripper_x = 0
local gripper_y = 0
local gripper_z = 0

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"MOVE_GRIPPER_YZ", SkillJumpState, skills={{gripper_commands_new}},final_to="MOVE_GRIPPER_X",FAILED},
   {"MOVE_GRIPPER_X", SkillJumpState, skills={{gripper_commands_new}}, final_to="OPEN_GRIPPER","FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="WAIT", fail_to="FAILED"},
   {"WAIT", JumpState},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},final_to="CLOSE_GRIPPER", fail_to="FAILED"},
   {"CLOSE_GRIPPER",SkillJumpState,skills={{gripper_commands_new}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT","MOVE_GRIPPER_X", cond=true},
  {"WAIT", "MOVE_BACK", timeout=0.5, desc="wait for gripper to open, then move back"},
}

function MOVE_GRIPPER_YZ:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].y = gripper_y
  self.args["gripper_commands_new"].z = gripper_z
end

function MOVE_GRIPPER_X:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = gripper_x
end
function OPEN_GRIPPER:init()
   self.args["gripper_commands_new"].command = "OPEN"
end

function MOVE_BACK:init()
   self.args["motor_move"].x = x_move_back
end

function CLOSE_GRIPPER:init()
   self.args["gripper_commands_new"].command = "CLOSE"
end
