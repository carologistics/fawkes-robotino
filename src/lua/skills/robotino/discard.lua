
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands"}
depends_interfaces = {
}

documentation      = [==[
Skill to put a product onto the conveyor or the slide.

]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

fsm:define_states{ export_to=_M,
   closure={pose_not_exist=pose_not_exist,is_grabbed = is_grabbed},
  {"INIT", JumpState},
  {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands}}, final_to="OPEN_GRIPPER",fail_to="FAILED"},
  {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="CALIBRATE_GRIPPER"},
  {"CALIBRATE_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="GRIPPER_HOME", fail_to="FAILED"},
  {"GRIPPER_HOME", SkillJumpState, skills={{gripper_commands}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT", "MOVE_GRIPPER_FORWARD", true, desc="Start aligning"},
}

function INIT:init()
  -- Override values if host specific config value is set

  if config:exists("/arduino/x_max") then
      self.fsm.vars.x_max = config:get_float("/arduino/x_max")
  else
      self.fsm.vars.x_max = 0.114
  end
  if config:exists("/arduino/y_max") then
      self.fsm.vars.y_max = config:get_float("/arduino/y_max")
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

  self.args["gripper_commands"].x = self.fsm.vars.x_max
  self.args["gripper_commands"].y = self.fsm.vars.y_max
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = "gripper_home"
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands"].command = "OPEN"
end

function CALIBRATE_GRIPPER:init()
  self.args["gripper_commands"].command = "CALIBRATE"
  self.args["gripper_commands"].wait = false
end

function GRIPPER_HOME:init()
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].wait = false
  self.args["gripper_commands"].command = "MOVEABS"
end
