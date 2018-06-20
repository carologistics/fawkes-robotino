
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
local gripper_tolerance_x = 0.005-- gripper x tolerance according to conveyor pose
local gripper_tolerance_y = 0.005 -- gripper y tolerance according to conveyor pose
local gripper_tolerance_z = 0.005 -- gripper z tolerance according to conveyor pose

local gripper_pose_offset_x = -0.02  -- conveyor pose offset in x direction
local gripper_pose_offset_y = 0.00     -- conveyor_pose offset in y direction
local gripper_pose_offset_z = 0.02  -- conveyor_pose offset in z direction

local conveyor_gripper_forward_x = 0.04 -- distance to move gripper forward after align
local conveyor_gripper_down_z = -0.01    -- distance to move gripper down after driving over conveyor
local conveyor_gripper_back_x = -0.06   -- distance to move gripper back after opening gripper
local conveyor_gripper_up_z = 0.01  -- distance to move gripper up after opening the gripper

local slide_gripper_forward_x = 0.04  -- distance to move gripper forward after align if the target is slide
local slide_gripper_down_z = -0.02     -- distance to move gripper down after driving over slide
local slide_gripper_back_x = -0.06    -- distance to move gripper back after opening the gripper if the target is slide
local slide_gripper_up_z = 0.01 --distance to move gripper up after opening the gripper if the target is slide

local drive_back_x = -0.1      -- distance to drive back after closing the gripper
local MAX_RETRIES = 3

local cfg_frame_ = "gripper"

local align_target_frame = "gripper_fingers" -- the gripper align is made relative to this frame (according to gripper_commands_new)
local z_movement_target_frame = "gripper" -- the gripper z movement is made relative to this frame (according to gripper_commands_new)
local x_movement_target_frame = "gripper" -- the gripper x movement is made relative to this frame (according to griper_commands_new)



function no_writer()
   return not if_conveyor_pose:has_writer()
end

function fitness_ok()
  return if_conveyor_pose:euclidean_fitness() >= euclidean_fitness_threshold
end

function tolerance_ok()
   local pose = pose_offset()
   if if_conveyor_pose:is_busy() then
     return false
   end

   return math.abs(pose.x) <= gripper_tolerance_x
      and math.abs(pose.y) <= gripper_tolerance_y
      and math.abs(pose.z) <= gripper_tolerance_z
end

function result_ready()
  if if_conveyor_pose:is_busy()
     or if_conveyor_pose:msgid() ~= fsm.vars.msgid
  then return false end

  local bb_stamp = fawkes.Time:new(if_conveyor_pose:input_timestamp(0), if_conveyor_pose:input_timestamp(1))
  if not tf:can_transform("conveyor_pose", "gripper_fingers", bb_stamp) then
    return false
  end

  local transform = fawkes.tf.StampedTransform:new()
  tf:lookup_transform("conveyor_pose", "gripper_fingers", transform)
  if transform.stamp:in_usec() < bb_stamp:in_usec() then
    return false
  end
  return true
end

function pose_offset()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", "gripper_fingers")
   print_info("product_pick: target_pos is x = %f, y = %f, z = %f", target_pos.x, target_pos.y, target_pos.z)
   print_info("product_pick: transformed_pos is x = %f, y = %f,z = %f", transformed_pos.x, transformed_pos.y, transformed_pos.z)

   return { x = transformed_pos.x,
            y = transformed_pos.y,
            z = transformed_pos.z,
  }

end

fsm:define_states{ export_to=_M,
   closure={MAX_RETRIES=MAX_RETRIES,tolerance_ok=tolerance_ok,result_ready=result_ready,fitness_ok=fitness_ok},
  {"INIT", JumpState},
  {"GRIPPER_ALIGN", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_FORWARD",fail_to="FAILED"},
  {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands_new}}, final_to="OPEN_GRIPPER",fail_to="FAILED"},
  {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_BACK", fail_to="PRE_FAIL"},
  {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands_new}}, final_to = "DRIVE_BACK", fail_to="PRE_FAIL"},
  {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="CLOSE_GRIPPER", fail_to="PRE_FAIL"},
  {"CLOSE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="FINAL", fail_to="PRE_FAIL"},
  {"PRE_FAIL", SkillJumpState, skills={{gripper_commands_new}}, final_to="FAILED", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT", "GRIPPER_ALIGN", true, desc="Start aligning"},
}

function INIT:init()
  if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.EnableSwitchMessage:new())
  local parse_result = pam.parse_to_type_id_target(if_conveyor_pose,self.fsm.vars.place,self.fsm.vars.side,self.fsm.vars.shelf,self.fsm.vars.slide)
  if_conveyor_pose:msgq_enqueue_copy(if_conveyor_pose.SetStationMessage:new(parse_result.mps_type,parse_result.mps_id,parse_result.mps_target))
  self.fsm.vars.mps_type = parse_result.mps_type
  self.fsm.vars.mps_target = parse_result.mps_target
  self.fsm.vars.mps_id = parse_result.mps_id
  self.fsm.vars.retries = 0
  self.fsm.vars.vision_retries = 0

  -- Override values if host specific config value is set
  if config:exists("/skills/product_put/gripper_pose_offset_x") then
      gripper_pose_offset_x = config:get_float("/skills/product_put/gripper_pose_offset_x")
  end
  if config:exists("/skills/product_put/gripper_pose_offset_y") then
      gripper_pose_offset_y = config:get_float("/skills/product_put/gripper_pose_offset_y")
  end
  if config:exists("/skills/product_put/gripper_pose_offset_z") then
      gripper_pose_offset_z = config:get_float("/skills/product_put/gripper_pose_offset_z")
  end
end

function GRIPPER_ALIGN:init()
  local pose = pose_offset(self)
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = pose.x
  self.args["gripper_commands_new"].y = pose.y
  self.args["gripper_commands_new"].z = pose.z
  self.args["gripper_commands_new"].target_frame = "gripper"
  self.fsm.vars.retries = self.fsm.vars.retries + 1
end

function MOVE_GRIPPER_FORWARD:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].target_frame = x_movement_target_frame

  if self.fsm.vars.slide then
    self.args["gripper_commands_new"].x = slide_gripper_forward_x
    self.args["gripper_commands_new"].z = slide_gripper_down_z
  else
    self.args["gripper_commands_new"].x = conveyor_gripper_forward_x
    self.args["gripper_commands_new"].z = conveyor_gripper_down_z
  end
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands_new"].command = "OPEN"
end

function CLOSE_GRIPPER:init()
  self.args["gripper_commands_new"].command = "CLOSE"
end

function MOVE_GRIPPER_BACK:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].target_frame = x_movement_target_frame
  if self.fsm.vars.slide then
    self.args["gripper_commands_new"].x = slide_gripper_back_x
    self.args["gripper_commands_new"].z = slide_gripper_up_z
  else
    self.args["gripper_commands_new"].x = conveyor_gripper_back_x
    self.args["gripper_commands_new"].z = conveyor_gripper_up_z
  end

end

function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end

function cleanup()
   if (fsm.vars.disable_realsense_afterwards == nil or fsm.vars.disable_realsense_afterwards) then
     if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.DisableSwitchMessage:new())
   end
end

function PRE_FAIL:init()
  self.args["gripper_commands_new"].command="CLOSE"
end

function FAILED:init()
   cleanup()
end

function FINAL:init()
   cleanup()
end
