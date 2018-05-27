----------------------------------------------------------------------------
--  product_pick_new.lua
--
--  Created Wed Apr 15
--  Copyright  2015  Johannes Rothe
--  Copyright  2018  Carsten Stoffels
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
name               = "product_pick"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands_new", "motor_move"}
depends_interfaces = {
  {v = "motor", type = "MotorInterface", id="Robotino" },
  {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
  {v = "conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
}

documentation      = [==[The robot needs to be aligned with the machine, then checks for pose of the conveyor_pose
  and adapts the gripper position if necessary. It then uses the gripper to actually pick the product.
and opens the gripper
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

-- Constants
local euclidean_fitness_threshold = 90  --threshold for euclidean fitness  (fitness should be higher)
local gripper_tolerance_x = 0.5 -- gripper x tolerance according to conveyor pose
local gripper_tolerance_y = 0.5 -- gripper y tolerance according to conveyor pose
local gripper_tolerance_z = 0.5 -- gripper z tolerance according to conveyor pose

local gripper_forward_x = 0 -- distance to move gripper forward after align
local gripper_down_z = 0    -- distance to move gripper down after driving over product
local gripper_back_x = 0    -- distance to move gripper back after closing gripper
local drive_back_x = -0.2   -- distance to drive back after closing the gripper

local cfg_frame_ = "gripper"


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

   return { x = cp.x,
            y = cp.y,
            z = cp.z,
            ori = ori
          }
end


fsm:define_states{ export_to=_M, closure={gripper_if=gripper_if},
   {"INIT", JumpState},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}},final_to="CHECK_VISION", fail_to="FAILED"},
   {"CHECK_VISION", JumpState},
   {"GRIPPER_ALIGN", SkillJumpState, skills={{gripper_commands_new}}, final_to="CHECK_TOLERANCE",fail_to="FAILED"},
   {"CHECK_TOLERANCE",JumpState},
   {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_DOWN",fail_to="FAILED"},
   {"MOVE_GRIPPER_DOWN", SkillJumpState, skills={{gripper_commands_new}}, final_to="CLOSE_GRIPPER", fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_BACK", fail_to="FAILED"},
   {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands_new}}, final_to = "DRIVE_BACK", fail_to="FAILED"},
   {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "OPEN_GRIPPER", true, desc="Start open gripper"},
   {"CHECK_VISION", "FAILED", timeout=20, desc="Fitness threshold wasn't reached"},
   {"CHECK_VISION", "FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "GRIPPER_ALIGN", cond=icp_fitness_check, desc="Fitness threshold reached"},
   {"CHECK_TOLERANCE", "MOVE_GRIPPER_FORWARD", cond=tolerance_check, desc="Pose tolerance ok"},
   {"CHECK_TOLERANCE", "CHECK_VISION", cond = true, desc="Pose tolerance not ok"},
}


function OPEN_GRIPPER:init()
  self.args["gripper_commands_new"].command = "OPEN"
end

function GRIPPER_ALIGN:init()
  local pose = pose_offset(self)
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = pose.x
  self.args["gripper_commands_new"].y = pose.y
  self.args["gripper_commands_new"].z = pose.z
end

function MOVE_GRIPPER_FORWARD:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = gripper_forward_x
end

function MOVE_GRIPPER_DOWN:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].z = gripper_down_z
end

function MOVE_GRIPPER_BACK:init()
  self.args["gripper_commands_new"].command = "MOVEABS"
  self.args["gripper_commands_new"].x = gripper_back_x
end

function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end
