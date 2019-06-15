
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
depends_skills     = {"gripper_commands","motor_move"}
depends_interfaces = {
    {v = "robotino_sensor", type = "RobotinoSensorInterface", id="Robotino"} -- Interface to read I/O ports
}

documentation      = [==[
Skill to put a product onto the conveyor or the slide.

Parameters:
      @param slide   optional true if you want to put it on the slide

]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

-- Constants
local gripper_pose_offset_x = 0.02  -- conveyor pose offset in x direction
local gripper_pose_offset_y = 0.00  -- conveyor_pose offset in y direction
local gripper_pose_offset_z = 0.03  -- conveyor_pose offset in z direction

local conveyor_gripper_forward_x = 0.07 -- distance to move gripper forward after align
local conveyor_gripper_down_z = -0.015  -- distance to move gripper down after driving over conveyor

local conveyor_gripper_back_x = -0.02 -- distance to move gripper back after opening gripper
local conveyor_gripper_up_z = 0.03    -- distance to move gripper up after opening the gripper

local slide_gripper_forward_x = 0.04  -- distance to move gripper forward after align if the target is slide
local slide_gripper_down_z = -0.04    -- distance to move gripper down after driving over slide

local slide_gripper_back_x = -0.01 -- distance to move gripper back after opening the gripper if the target is slide
local slide_gripper_up_z = 0.01    --distance to move gripper up after opening the gripper if the target is slide

local drive_back_x = -0.1      -- distance to drive back after closing the gripper

-- function to evaluate sensor data
function is_grabbed()
 if not robotino_sensor:has_writer() then
   print_warn("No robotino sensor writer to check sensor")
   return true
 end
 if robotino_sensor:is_digital_in(0) == false and robotino_sensor:is_digital_in(1) == true then -- white cable on DI1 and black on DI2
    return true
 else
    return false
 end
end


function pose_not_exist()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", "gripper")
   if transformed_pos == nil then
     return true
   end
   return false
end

fsm:define_states{ export_to=_M,
   closure={pose_not_exist=pose_not_exist,is_grabbed = is_grabbed},
  {"INIT", JumpState},
  {"GRIPPER_ALIGN", SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER_FORWARD",fail_to="FAILED"},
  {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands}}, final_to="OPEN_GRIPPER",fail_to="FAILED"},
  {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER_BACK", fail_to="FAILED"},
  {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands}}, final_to = "CALIBRATE_GRIPPER", fail_to="FAILED"},
  {"CALIBRATE_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="GRIPPER_HOME", fail_to="FAILED"},
  {"GRIPPER_HOME", SkillJumpState, skills={{gripper_commands}}, final_to="DRIVE_BACK", fail_to="FAILED"},
  {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
  {"INIT", "FAILED", cond="pose_not_exist()"},
  {"INIT", "FAILED", cond="not is_grabbed()", desc="Lost product"},
  {"INIT", "GRIPPER_ALIGN", true, desc="Start aligning"},
  {"MOVE_GRIPPER_FORWARD", "FAILED", cond="not is_grabbed()", desc="Lost product while aligning"},
}

function GRIPPER_ALIGN:init()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x = 0, y = 0, z = 0, w = 0}
  }

  self.args["gripper_commands"].command = "MOVEREL"
  self.args["gripper_commands"].x = target_pos.x
  self.args["gripper_commands"].y = target_pos.y
  self.args["gripper_commands"].z = target_pos.z
end

function MOVE_GRIPPER_FORWARD:init()
  if self.fsm.vars.slide ~= nil then
    self.args["gripper_commands"].x = slide_gripper_forward_x
    self.args["gripper_commands"].z = slide_gripper_down_z
  else
    self.args["gripper_commands"].x = conveyor_gripper_forward_x
    self.args["gripper_commands"].z = conveyor_gripper_down_z
  end

  self.args["gripper_commands"].y = 0.0
  self.args["gripper_commands"].command = "MOVEREL"
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands"].command = "OPEN"
end

function MOVE_GRIPPER_BACK:init()
  if self.fsm.vars.slide ~= nil then
    self.args["gripper_commands"].x = slide_gripper_back_x
    self.args["gripper_commands"].z = slide_gripper_up_z
  else
    self.args["gripper_commands"].x = conveyor_gripper_back_x
    self.args["gripper_commands"].z = conveyor_gripper_up_z
  end
  self.args["gripper_commands"].y = 0.0
  self.args["gripper_commands"].command = "MOVEREL"
end

function CALIBRATE_GRIPPER:init()
  self.args["gripper_commands"].command = "CALIBRATE"
  self.args["gripper_commands"].wait = false
end

function GRIPPER_HOME:init()
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = 0.03
  self.args["gripper_commands"].wait = false
  self.args["gripper_commands"].command = "MOVEABS"
end


function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end
