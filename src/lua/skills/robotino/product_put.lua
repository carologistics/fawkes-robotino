
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
depends_skills     = {"gripper_commands","motor_move", "reset_gripper", "check_wp"}
depends_interfaces = {}

documentation      = [==[
Skill to put a product onto the conveyor or the slide.

Parameters:
      @param check_wp (optional, default=true) whether you want to check for a wp or not
      @param slide   optional true if you want to put it on the slide

]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

-- Constants
local gripper_pose_offset_x = 0.005  -- conveyor pose offset in x direction
local gripper_pose_offset_y = 0.00  -- conveyor_pose offset in y direction
local gripper_pose_offset_z = 0.03  -- conveyor_pose offset in z direction

local conveyor_gripper_forward_x = 0.018 -- distance to move gripper forward after align
local conveyor_gripper_down_z = -0.015  -- distance to move gripper down after driving over conveyor

local conveyor_gripper_back_x = -0.02 -- distance to move gripper back after opening gripper
local conveyor_gripper_up_z = 0.03    -- distance to move gripper up after opening the gripper

local slide_gripper_forward_x = 0.035  -- distance to move gripper forward after align if the target is slide
local slide_gripper_down_z = -0.04    -- distance to move gripper down after driving over slide

local slide_gripper_back_x = -0.01 -- distance to move gripper back after opening the gripper if the target is slide
local slide_gripper_up_z = 0.01    --distance to move gripper up after opening the gripper if the target is slide

local drive_back_x = -0.1      -- distance to drive back after closing the gripper

function pose_not_exist()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", "odom")
   if transformed_pos == nil then
     return true
   end
   return false
end

function pose_gripper_offset(x,y,z)
  local target_pos = { x = x,
                       y = y,
                       z = z,
                       ori = { x = 0, y = 0, z = 0, w = 0}

   }
   local tmp = { x = 0,
                 y = 0,
                 z = 0,
                 ori = { x = 0, y = 0, z = 0, w = 0}
   }

   -- Get offset from gripper axis (middle of z sledge) to gripper finger
   local gripper_rel = tfm.transform6D(tmp,"gripper","gripper_z_dyn")

   -- Shift target point to gripper axis frame
   gripper_rel.x = target_pos.x - gripper_rel.x
   gripper_rel.y = target_pos.y - gripper_rel.y
   gripper_rel.z = target_pos.z - gripper_rel.z

   -- Transform target to gripper home frame = absolut coordinates of the axis
   local gripper_home_rel = tfm.transform6D(gripper_rel,"gripper","gripper_home")

   -- Clip to axis limits
   return { x = math.max(0,math.min(gripper_home_rel.x,fsm.vars.x_max)),
            y = math.max(-fsm.vars.y_max/2,math.min(gripper_home_rel.y,fsm.vars.y_max/2)),
            z = math.max(0,math.min(gripper_home_rel.z,fsm.vars.z_max))}
end

fsm:define_states{ export_to=_M,
   closure={pose_not_exist=pose_not_exist},
  {"INIT", JumpState},
  {"CHECK_WP", SkillJumpstate, skills={{check_wp}}, final_to="GRIPPER_ALIGN", fail_to="FAILED"},
  {"GRIPPER_ALIGN", SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER_FORWARD",fail_to="FAILED"},
  {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands}}, final_to="OPEN_GRIPPER",fail_to="FAILED"},
  {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="RESET_GRIPPER", fail_to="FAILED"},
  {"RESET_GRIPPER", SkillJumpState, skills={{reset_gripper}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT", "FAILED", cond="pose_not_exist()"},
  {"INIT", "CHECK_WP", cond=true, desc="Start aligning"},
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

function GRIPPER_ALIGN:init()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x = 0, y = 0, z = 0, w = 0}
  }

  local grip_pos_odom = tfm.transform6D(target_pos, "conveyor_pose", "odom")
  local grip_pos = tfm.transform6D(grip_pos_odom, "odom", "gripper")

  local pose =  pose_gripper_offset(grip_pos.x,grip_pos.y,grip_pos.z)

  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].x = pose.x
  self.args["gripper_commands"].y = pose.y
  self.args["gripper_commands"].z = pose.z
  self.args["gripper_commands"].target_frame = "gripper_home"
end

function MOVE_GRIPPER_FORWARD:init()
  local pose = {}
  if self.fsm.vars.slide ~= nil then
    pose = pose_gripper_offset(slide_gripper_forward_x, 0, slide_gripper_down_z)
  else
    pose = pose_gripper_offset(conveyor_gripper_forward_x, 0, conveyor_gripper_down_z)
  end

  self.args["gripper_commands"] = pose
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = "gripper_home"
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands"].command = "OPEN"
end

function RESET_GRIPPER:init()
  self.args["reset_gripper"].calibrate = true
end
