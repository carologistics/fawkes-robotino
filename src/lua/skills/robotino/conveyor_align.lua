----------------------------------------------------------------------------
--  conveyor_align.lua - align orthogonal to the conveyor using the conveyor vision
--
--  Copyright  2013 The Carologistics Team
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
name               = "conveyor_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move", "gripper_commands_new"}
depends_interfaces = {
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
   {v = "if_conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
}

documentation      = [==[aligns the robot orthogonal to the conveyor by using the
                         conveyor vision
Parameters:
       @param disable_realsense_afterwards   disable the realsense after aligning
       @param place   the name of the MPS (see navgraph e.g.: "M-BS" for base station of team magenta)
       @param side    optional the side of the mps
       @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
       @param slide   optional true if you want to put it on the slide
]==]

-- Initialize as skill module

skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local pam = require("parse_module")

-- Constants
local euclidean_fitness_threshold = 8 -- threshold for euclidean fitness  (fitness should be higher)
local tolerance_trans = 0.015
local tolerance_ori = 0.015
local x_dist_to_mps = -0.31  -- x-distance the robot should have after the align
local cfg_frame_ = "gripper"

local GRIPPER_POSES = {
   {x=0.05, y=0.00, z=0.035},
   {x=0.05, y=0.02, z=0.03},
   {x=0.05, y=-0.02, z=0.03}
}

function no_writer()
   return not if_conveyor_pose:has_writer()
end

function tolerance_ok(self)
   local pose = pose_offset(self)
   return math.abs(pose.x) <= tolerance_trans
      and math.abs(pose.y) <= tolerance_trans
      and math.abs(pose.ori) <= tolerance_ori
end

function drive_ready_check(self)
  local test_target_pos = { x = x_dist_to_mps,
                       y = 0,
                       ori = 0,
  }

  local test_transformed_pos = tfm.transform(test_target_pos, "conveyor_pose", "base_link")
  if test_transformed_pos == nil then
    return false
  end

  return if_conveyor_pose:euclidean_fitness() > euclidean_fitness_threshold
     and not if_conveyor_pose:is_busy()
     and if_conveyor_pose:msgid() == fsm.vars.msgid
end

function pose_offset(self)
      local target_pos = { x = x_dist_to_mps,
                           y = 0,
                           ori = 0,
      }

      local transformed_pos = tfm.transform(target_pos, "conveyor_pose", "base_link")
      print_info("transformed_pos is x = %f, y = %f,ori = %f", transformed_pos.x, transformed_pos.y, transformed_pos.ori)

      return transformed_pos
end


fsm:define_states{ export_to=_M,
   closure={},
   {"INIT", JumpState},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="CHECK_VISION", failed_to="FAILED"},
   {"CHECK_VISION", JumpState},
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}}, final_to="CHECK_TOLERANCE", failed_to="FAILED"},
   {"CHECK_TOLERANCE", JumpState},
}

fsm:add_transitions{
   {"INIT", "MOVE_GRIPPER", cond=true},
   {"CHECK_VISION", "FAILED", timeout=20, desc = "Fitness threshold wasn't reached"},
   {"CHECK_VISION", "FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "DRIVE_FORWARD", cond=drive_ready_check, desc="Fitness threshold reached"},
   {"CHECK_TOLERANCE", "FINAL", cond=tolerance_ok, desc="Pose tolerance ok"},
   {"CHECK_TOLERANCE", "CHECK_VISION", cond = true, desc="Pose tolerance not ok"},
}

function INIT:init()
   self.fsm.vars.gripper_pose_idx = 1
   if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.EnableSwitchMessage:new())
   local parse_result = pam.parse_to_type_target(if_conveyor_pose,self.fsm.vars.place,self.fsm.vars.side,self.fsm.vars.shelf,self.fsm.vars.slide)
   self.fsm.vars.mps_type = parse_result.mps_type
   self.fsm.vars.mps_target = parse_result.mps_target
end

function CHECK_VISION:init()
   local msg = if_conveyor_pose.SetStationMessage:new(self.fsm.vars.mps_type, self.fsm.vars.mps_target)
   if_conveyor_pose:msgq_enqueue_copy(msg)
   self.fsm.vars.msgid = msg:id()
end

function MOVE_GRIPPER:init()
  self.args["gripper_commands_new"] = GRIPPER_POSES[self.fsm.vars.gripper_pose_idx]
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.fsm.vars.gripper_pose_idx = (self.fsm.vars.gripper_pose_idx + 1) % #GRIPPER_POSES
end

function DRIVE_FORWARD:init()
   local pose = pose_offset(self)
   print_info("Drive forward, x = %f , y = %f", pose.x, pose.y)
   self.args["motor_move"].x = pose.x
   self.args["motor_move"].y = pose.y
   self.args["motor_move"].ori = pose.ori
   self.args["motor_move"].tolerance = {
      x = tolerance_trans * 0.9,
      y = tolerance_trans * 0.9,
      ori = tolerance_ori * 0.9
   }
end

function cleanup()
   if (fsm.vars.disable_realsense_afterwards == nil or fsm.vars.disable_realsense_afterwards) then
     if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.DisableSwitchMessage:new())
   end
end

function FAILED:init()
   cleanup()
end

function FINAL:init()
   cleanup()
end

