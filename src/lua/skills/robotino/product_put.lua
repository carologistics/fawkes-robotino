
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
depends_skills     = {"motor_move", "ax12gripper", "approach_mps"}
depends_interfaces = { }

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
@param offset_x the offset_x from the navgraph point, default in cfg/conf.d/skills.yaml:skills/product_put/offset_x
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local x_distance = 0.315
if config:exists("/skills/approach_distance_conveyor/x") then
      x_distance = config:get_float("/skills/approach_distance_conveyor/x")
end
x_distance= x_distance

fsm:define_states{ export_to=_M,
   {"DRIVE_FORWARD", SkillJumpState, skills={{approach_mps}},
      final_to="OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="WAIT", fail_to="MOVE_BACK_FAILED"},
   {"WAIT", JumpState},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="CLOSE_GRIPPER", fail_to="CLOSE_GRIPPER"},
   {"MOVE_BACK_FAILED", SkillJumpState, skills={{motor_move}},
      final_to="FAILED", fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="RESET_Z_POS", fail_to="RESET_Z_POS"},
   {"SLAP_LEFT", SkillJumpState, skills={{ax12gripper}},
      final_to="OPEN_FROM_SLAP_LEFT", fail_to="OPEN_FROM_SLAP_LEFT"},
   {"WAIT_SLAP_LEFT", JumpState},
   {"OPEN_FROM_SLAP_LEFT", SkillJumpState, skills={{ax12gripper}},
      final_to="SLAP_RIGHT", fail_to="SLAP_RIGHT"},
   {"SLAP_RIGHT", SkillJumpState, skills={{ax12gripper}},
      final_to="OPEN_FROM_SLAP_RIGHT", fail_to="OPEN_FROM_SLAP_RIGHT"},
   {"WAIT_SLAP_RIGHT", JumpState},
   {"OPEN_FROM_SLAP_RIGHT", SkillJumpState, skills={{ax12gripper}},
      final_to="WAIT_FOR_GRIPPER", fail_to="WAIT_FOR_GRIPPER"},
   {"WAIT_FOR_GRIPPER", JumpState},
   {"RESET_Z_POS", SkillJumpState, skills={{ax12gripper}},
      final_to="FINAL", fail_to="FINAL"},
}

fsm:add_transitions{
--   {"WAIT", "MOVE_BACK", timeout=0.5, desc="wait for gripper to open"}
   {"WAIT", "SLAP_LEFT", timeout=0.5, desc="wait for gripper to open, then slap left"},
   {"WAIT_FOR_GRIPPER", "MOVE_BACK", timeout=0.5}
}

function DRIVE_FORWARD:init()
   if self.fsm.vars.offset_x == nil then
      self.fsm.vars.offset_x = - 0.01
      if config:exists("/skills/product_put/offset_x") then
         self.fsm.vars.offset_x = config:get_float("/skills/product_put/offset_x")
      end
   end

   self.args["approach_mps"].x = x_distance - self.fsm.vars.offset_x
   self.args["approach_mps"].use_conveyor = true
end

function OPEN_GRIPPER:init()
   self.args["ax12gripper"].command = "OPEN"
   printf("open gripper")
end

function MOVE_BACK:init()
   self.args["motor_move"].x = -0.2
end

function MOVE_BACK_FAILED:init()
   self.args["motor_move"].x = -0.2
end

function CLOSE_GRIPPER:init()
   self.args["ax12gripper"].command = "CLOSE"
   printf("close gripper")
end

function RESET_Z_POS:init()
   self.args["ax12gripper"].command = "RESET_Z_POS"
end

function SLAP_LEFT:init()
   self.args["ax12gripper"].command = "SLAP_LEFT"
end

function SLAP_RIGHT:init()
   self.args["ax12gripper"].command = "SLAP_RIGHT"
end

function OPEN_FROM_SLAP_LEFT:init()
   self.args["ax12gripper"].command = "OPEN"
end

function OPEN_FROM_SLAP_RIGHT:init()
   self.args["ax12gripper"].command = "OPEN"
end
