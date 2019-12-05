
----------------------------------------------------------------------------
--  discard.lua
--
--  Created Sat 04 April
--  Copyright  2019  Daniel Habering
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
name               = "discard"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands", "reset_gripper"}
depends_interfaces = {
}

documentation      = [==[
Skill to safely discard a workpiece
]==]

local MOVE_X = 0.05

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={},
  {"INIT", JumpState},
  {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands}}, final_to="OPEN_GRIPPER",fail_to="FAILED"},
  {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="RESET_GRIPPER"},
  {"RESET_GRIPPER", SkillJumpState, skills={{reset_gripper}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT", "MOVE_GRIPPER_FORWARD", true},
}

function INIT:init()
  -- Override values if host specific config value is set
  if config:exists("/arduino/y_max") then
      self.fsm.vars.y_max = config:get_float("/arduino/y_max")/2
  else
      self.fsm.vars.y_max = 0.038
  end
  if config:exists("/arduino/max_z") then
      self.fsm.vars.z_max = config:get_float("/arduino/z_max")
  else
      self.fsm.vars.z_max = 0.057
  end
end

function MOVE_GRIPPER_FORWARD:init()

  self.args["gripper_commands"].x = MOVE_X 
  self.args["gripper_commands"].y = self.fsm.vars.y_max
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = "gripper_home"
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands"].command = "OPEN"
end

function RESET_GRIPPER:init()
  self.args["reset_gripper"].calibrate = false
end
