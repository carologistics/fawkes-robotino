----------------------------------------------------------------------------
--  reset_gripper.lua
--
--  Created Fri May 03
--  Copyright  2019  Morian Sonnet
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
--
--  This skill resets the gripper. The bool param calibrate decides whether a calibration is run,
--  or only the realsense is blocked

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "reset_gripper"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands"}
depends_interfaces = {
}

documentation      = [==[
Skill to reset the gripper after either an succesful or not succesful pick/put

Parameters:
    @param calibrate   Decide whether gripper should be calibrated or not
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")

-- Constant
local height_safe = 0.05  -- z coordinate at home

function should_calibrate(self)
  if self.fsm.vars.calibrate ~= nil then
    if self.fsm.vars.calibrate == true then
      return true
    end
  end
  return false
end


fsm:define_states{ export_to=_M, closure={should_calibrate=should_calibrate},
  {"INIT", JumpState},
  {"GO_UP", SkillJumpState, skills={{gripper_commands}}, final_to="GO_BACK", fail_to="FAILED"},
  {"GO_BACK", SkillJumpState, skills={{gripper_commands}}, final_to="DECIDE_CAL", fail_to="FAILED"},
  {"DECIDE_CAL", JumpState},
  {"CALIBRATE", SkillJumpState, skills={{gripper_commands}}, final_to="HOME", fail_to="FAILED"},
  {"HOME", SkillJumpState, skills={{gripper_commands}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "GO_UP", cond=true},
   {"DECIDE_CAL", "CALIBRATE", cond=should_calibrate},
   {"DECIDE_CAL", "HOME", cond=true},
}

function GO_UP:init()
  self.args["gripper_commands"].command="MOVEABS"
  self.args["gripper_commands"].x_rel = 0.0
  self.args["gripper_commands"].y_rel = 0.0
  self.args["gripper_commands"].z = height_safe
  self.args["gripper_commands"].wait = true
end

function GO_BACK:init()
  self.args["gripper_commands"].command="MOVEABS"
  self.args["gripper_commands"].y_rel = 0.0
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].z = height_safe
  self.args["gripper_commands"].wait = false
end

function HOME:init()
  self.args["gripper_commands"].command="MOVEABS"
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].wait = false
end

function CALIBRATE:init()
  self.args["gripper_commands"].command="CALIBRATE"
  self.args["gripper_commands"].wait="CALIBRATE"
end
