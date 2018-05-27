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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
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
]==]

-- Initialize as skill module

skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

-- Constants
local euclidean_fitness_threshold = 300 -- threshold for euclidean fitness  (fitness should be higher)
local tolerance_x = 0.5  -- x-tolerance according to conveyor pose
local gripper_x = 0  -- gripper x-coordinate before the align
local gripper_y = 0  -- gripper x-coordinate before the align
local gripper_z = 0  -- gripper x-coordinate before the align
local x_dist_to_mps = 0.2  -- x-distance the robot should have after the align
local cfg_frame_ = "gripper"

function no_writer()
   return not if_conveyor_pose:has_writer()
end

function tolerance_check(self)
   local pose = pose_offset(self)
   if math.abs(pose.x) <= tolerance_x then
      return true
   end
end

function icp_fitness_check(self)
     return if_conveyor_pose:euclidean_fitness() > euclidean_fitness_threshold
end

function pose_offset(self)

  local from = { x = if_conveyor_pose:translation(0),
                 y = if_conveyor_pose:translation(1),
                 z = if_conveyor_pose:translation(2),
                 ori = { x = if_conveyor_pose:rotation(0),
                         y = if_conveyor_pose:rotation(1),
                         z = if_conveyor_pose:rotation(2),
                         w = if_conveyor_pose:rotation(3),
                       }
                }

  print_info("Conveyor pose translation is x = %f, y = %f , z  = %f", from.x, from.y, from.z)
  local cp = tfm.transform6D(from, if_conveyor_pose:frame(), cfg_frame_)
  print_info("Pose offset is x = %f, y = %f, z = %f", cp.x, cp.y, cp.z)
  local ori = 0

   -- Keep distance to mps
   cp.x = cp.x - x_dist_to_mps

   return { x = cp.x,
            y = cp.y,
            z = cp.z,
            ori = ori
          }
end


fsm:define_states{ export_to=_M,
   closure={},
   {"INIT", JumpState},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="CHECK_VISION", failed_to="CLEANUP_FAILED"},
   {"CHECK_VISION", JumpState},
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}}, final_to="CHECK_TOLERANCE", failed_to="CLEANUP_FAILED"},
   {"CHECK_TOLERANCE", JumpState},
   {"CLEANUP_FINAL", JumpState},
   {"CLEANUP_FAILED", JumpState},
}

fsm:add_transitions{
   {"INIT", "MOVE_GRIPPER", cond=true},
   {"CHECK_VISION", "CLEANUP_FAILED", timeout=20, desc = "Fitness threshold wasn't reached"},
   {"CHECK_VISION", "CLEANUP_FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "DRIVE_FORWARD", cond=icp_fitness_check, desc="Fitness threshold reached"},
   {"CHECK_TOLERANCE", "CLEANUP_FINAL", cond=tolerance_check, desc="Pose tolerance ok"},
   {"CHECK_TOLERANCE", "CHECK_VISION", cond = true, desc="Pose tolerance not ok"},
   {"CLEANUP_FINAL", "FINAL", cond=true, desc="Cleaning up after final"},
   {"CLEANUP_FAILED", "FAILED", cond=true, desc="Cleaning up after fail"},
}

function INIT:init()
   if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.EnableSwitchMessage:new())
   if_conveyor_pose:msgq_enqueue_copy(if_conveyor_pose.SetStationMessage:new(if_conveyor_pose.BASE_STATION,if_conveyor_pose.INPUT_CONVEYOR))
end

function MOVE_GRIPPER:init()
  print_info("Move gripper to %f,%f,%f", gripper_x, gripper_y, gripper_z)
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = gripper_x
  self.args["gripper_commands_new"].y = gripper_y
  self.args["gripper_commands_new"].z = gripper_z
end


function DRIVE_FORWARD:init()
  local pose = pose_offset(self)
  print_info("Drive forward, x = %f , y = %f", pose.x, pose.y)
  self.args["motor_move"].x = pose.x
  self.args["motor_move"].y = pose.y
end

function CLEANUP_FINAL:init()
   if (self.fsm.vars.disable_realsense_afterwards == nil or self.fsm.vars.disable_realsense_afterwards) then
     if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.DisableSwitchMessage:new())
   end
end

function CLEANUP_FAILED:init()
   if (self.fsm.vars.disable_realsense_afterwards == nil or self.fsm.vars.disable_realsense_afterwards) then
     if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.DisableSwitchMessage:new())
   end
end
