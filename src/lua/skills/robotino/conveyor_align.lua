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
       @param mps_type   (the mps_type conveyor_align should align to)
       @param mps_target (the mps target conveyor_align should align to)

]==]

-- Initialize as skill module

skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")


local euclidean_fitness_tolerance = 50
local TOLERANCE_Y = 0.5
local TOLERANCE_X = 0.5
local TOLERANCE_Z = 0.5

local cfg_frame_ = "gripper"

function no_writer()
   return not if_conveyor:has_writer()
end

function tolerance_check(self)
   local pose = pose_offset(self)
   if math.abs(pose.y) <= TOLERANCE_Y and math.abs(pose.z) <= TOLERANCE_Z and math.abs(pose.x) <= TOLERANCE_X then
      return true
   end
end

function icp_fitness_check(self)
     return if_conveyor_pose:euclidean_fitness() > euclidean_fitness_tolerance
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
   local cp = tfm.transform6D(from, if_conveyor_pose:frame(), cfg_frame_)

   --local ori = fawkes.tf.get_yaw( fawkes.tf.Quaternion:new(cp.ori.x, cp.ori.y, cp.ori.z, cp.ori.w))
   print_info("Pose offset is x = %f, y = %f, z = %f", cp.x, cp.y, cp.z)
   local ori = 0

   return { x = cp.x,
            y = cp.y,
            z = cp.z,
            ori = ori
          }
end


fsm:define_states{ export_to=_M,
   closure={},
   {"INIT", JumpState},
   {"CHECK_VISION", JumpState},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="CHECK_TOLERANCE", failed_to="CLEANUP_FAILED"},
   {"CHECK_TOLERANCE", JumpState},
   {"CLEANUP_FINAL", JumpState},
   {"CLEANUP_FAILED", JumpState},
}

fsm:add_transitions{
   {"INIT", "CHECK_VISION", cond=true},
   {"CHECK_VISION", "CLEANUP_FAILED", timeout=20, desc="No vis_hist on conveyor vision"},
   {"CHECK_VISION", "CLEANUP_FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "MOVE_GRIPPER", cond=icp_fitness_check, desc="Fitness threshold reached"},
   {"CHECK_TOLERANCE", "CLEANUP_FINAL", cond=tolerance_check, desc="Pose tolerance ok"},
   {"CHECK_TOLERANCE", "CHECK_VISION", cond=true, desc="Pose tolerance not ok"},
   {"CLEANUP_FINAL", "FINAL", cond=true, desc="Cleaning up after final"},
   {"CLEANUP_FAILED", "FAILED", cond=true, desc="Cleaning up after fail"},
}

function INIT:init()
   if_conveyor_pose_switch:msgq_enqueue_copy(if_conveyor_switch.EnableSwitchMessage:new())
   if_conveyor_pose:msgq_enqueue_copy(if_conveyor_pose.SetStationMessage:new(if_conveyor_pose.BASE_STATION,if_conveyor_pose.INPUT_CONVEYOR))
end

function MOVE_GRIPPER:init()
  local pose = pose_des(self)
  print_info("Move gripper to %f,%f,%f", pose.x, pose.y, pose.z)
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = pose.x
  self.args["gripper_commands_new"].y = pose.y
  self.args["gripper_commands_new"].z = pose.z
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
