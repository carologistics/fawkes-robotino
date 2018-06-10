
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"gripper_commands_new","motor_move"}
depends_interfaces = {
  {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
  {v = "if_conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
}

documentation      = [==[

Parameters:
      @param place   the name of the MPS (see navgraph e.g.: "M-BS" for base station of team magenta)
      @param side    optional the side of the mps ("input" or "output")
      @param slide   optional true if you want to put it on the slide

]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local pam = require("parse_module")

-- Constants
local euclidean_fitness_threshold = 8  --threshold for euclidean fitness  (fitness should be higher)
local gripper_tolerance_x = 0.5 -- gripper x tolerance according to conveyor pose
local gripper_tolerance_y = 0.5 -- gripper y tolerance according to conveyor pose
local gripper_tolerance_z = 0.5 -- gripper z tolerance according to conveyor pose

local gripper_pose_offset_x = 0.02  -- conveyor pose offset in x direction
local gripper_pose_offset_y = 0     -- conveyor_pose offset in y direction
local gripper_pose_offset_z = 0.065  -- conveyor_pose offset in z direction

local conveyor_gripper_forward_x = 0.04 -- distance to move gripper forward after align
local conveyor_gripper_down_z = -0.01    -- distance to move gripper down after driving over conveyor
local conveyor_gripper_back_x = -0.06   -- distance to move gripper back after closing gripper

local slide_gripper_forward_x = 0.04  -- distance to move gripper forward after align if the target is slide
local slide_gripper_down_z = -0.01     -- distance to move gripper down after driving over slide
local slide_gripper_back_x = -0.06    -- distance to move gripper back after closing the gripper if the target is slide

local drive_back_x = -0.1      -- distance to drive back after closing the gripper


local cfg_frame_ = "gripper"

local align_target_frame = "gripper"      -- the gripper align is made relative to this frame (according to gripper_commands_new)
local z_movement_target_frame = "gripper" -- the gripper z movement is made relative to this frame (according to gripper_commands_new)
local x_movement_target_frame = "gripper" -- the gripper x movement is made relative to this frame (according to griper_commands_new)



function no_writer()
   return not if_conveyor_pose:has_writer()
end

function tolerance_check(self)
   local pose = pose_offset(self)
   if math.abs(pose.x) <= gripper_tolerance_x and math.abs(pose.y) <= gripper_tolerance_y and math.abs(pose.z) <= gripper_tolerance_z then
      return true
   end
end

function icp_fitness_check(self)
     return if_conveyor_pose:euclidean_fitness() > euclidean_fitness_threshold
end

function pose_offset(self)

  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", "gripper")
   print_info("product_pick: target_pos is x = %f, y = %f, z = %f", target_pos.x, target_pos.y, target_pos.z)
   print_info("product_pick: transformed_pos is x = %f, y = %f,z = %f", transformed_pos.x, transformed_pos.y, transformed_pos.z)

   return { x = transformed_pos.x,
            y = transformed_pos.y,
            z = transformed_pos.z,
  }

end

fsm:define_states{ export_to=_M,
  {"INIT", JumpState},
  {"CHECK_VISION", JumpState},
  {"GRIPPER_ALIGN", SkillJumpState, skills={{gripper_commands_new}}, final_to="CHECK_TOLERANCE",fail_to="CLEANUP_FAILED"},
  {"CHECK_TOLERANCE",JumpState},
  {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_DOWN",fail_to="CLEANUP_FAILED"},
  {"MOVE_GRIPPER_DOWN", SkillJumpState, skills={{gripper_commands_new}}, final_to="OPEN_GRIPPER", fail_to="CLEANUP_FAILED"},
  {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_BACK", fail_to="CLEANUP_FAILED"},
  {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands_new}}, final_to = "DRIVE_BACK", fail_to="CLEANUP_FAILED"},
  {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="CLOSE_GRIPPER", fail_to="CLEANUP_FAILED"},
  {"CLOSE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="CLEANUP_FINAL", fail_to="CLEANUP_FAILED"},
  {"CLEANUP_FINAL", JumpState},
  {"CLEANUP_FAILED", JumpState},
}

fsm:add_transitions{
  {"INIT", "CHECK_VISION", true, desc="Start open gripper"},
  {"CHECK_VISION", "CLEANUP_FAILED", timeout=20, desc="Fitness threshold wasn't reached"},
  {"CHECK_VISION", "CLEANUP_FAILED", cond=no_writer, desc="No writer for conveyor vision"},
  {"CHECK_VISION", "GRIPPER_ALIGN", cond=icp_fitness_check, desc="Fitness threshold reached"},
  {"CHECK_TOLERANCE", "MOVE_GRIPPER_FORWARD", cond=tolerance_check, desc="Pose tolerance ok"},
  {"CHECK_TOLERANCE", "CHECK_VISION", cond = true, desc="Pose tolerance not ok"},
  {"CLEANUP_FINAL", "FINAL", cond = true, desc="Cleaning up after final"},
  {"CLEANUP_FAILED", "FAILED", cond = true, desc="Cleaning up after failed"},
}

function INIT:init()
  if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.EnableSwitchMessage:new())
  local parse_result = pam.parse_to_type_target(if_conveyor_pose,self.fsm.vars.place,self.fsm.vars.side,self.fsm.vars.shelf,self.fsm.vars.slide)
  if_conveyor_pose:msgq_enqueue_copy(if_conveyor_pose.SetStationMessage:new(parse_result.mps_type,parse_result.mps_target))
end

function GRIPPER_ALIGN:init()
  local pose = pose_offset(self)
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = pose.x
  self.args["gripper_commands_new"].y = pose.y
  self.args["gripper_commands_new"].z = pose.z
  self.args["gripper_commands_new"].target_frame = align_target_frame
end

function MOVE_GRIPPER_FORWARD:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].target_frame = x_movement_target_frame

  if self.fsm.vars.slide then
    self.args["gripper_commands_new"].x = slide_gripper_forward_x
  else
    self.args["gripper_commands_new"].x = conveyor_gripper_forward_x
  end
end

function MOVE_GRIPPER_DOWN:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  if self.fsm.vars.slide then
    self.args["gripper_commands_new"].z = slide_gripper_down_z
  else
    self.args["gripper_commands_new"].z = conveyor_gripper_down_z
  end
  self.args["gripper_commands_new"].target_frame = z_movement_target_frame
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands_new"].command = "OPEN"
end

function CLOSE_GRIPPER:init()
  self.args["gripper_commands_new"].command = "CLOSE"
end

function MOVE_GRIPPER_BACK:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  if self.fsm.vars.slide then
    self.args["gripper_commands_new"].x = slide_gripper_back_x
  else
    self.args["gripper_commands_new"].x = conveyor_gripper_back_x
  end
  self.args["gripper_commands_new"].target_frame = x_movement_target_frame
end

function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end
