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
depends_skills     = {"motor_move", "gripper_commands"}
depends_interfaces = {
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
   {v = "if_plane_switch", type = "SwitchInterface", id="conveyor_plane/switch"},
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
local euclidean_fitness_threshold = 5 -- threshold for euclidean fitness for targets other than shelf
local slide_euclidean_fitness_threshold = 7 -- threshold for euclidean fitness if target is shelf
local tolerance_trans = 0.04  -- tolerance in x and y direction after the align
local tolerance_ori = 0.05   -- orientation tolerance after the align
local x_dist_to_mps = -0.255  -- x-distance the robot should have after the align
local y_offset_shelf_middle = -0.015 -- y-offset the robot should have picking is done from shelf = "MIDDLE"

-- initial gripper poses depending on the target
local gripper_pose = { x= 0.05, y = 0.00, z = 0.03}

local MAX_RETRIES=3
local MAX_VISION_RETRIES=3

function no_writer()
   return not if_conveyor_pose:has_writer()
end

function input_ok()
  if fsm.vars.shelf == "LEFT" or fsm.vars.shelf == "RIGHT" or fsm.vars.shelf == "MIDDLE" then
    return true
  end
  if fsm.vars.slide then
    return true
  end
  if fsm.vars.side == "input" or fsm.vars.side == "output" then
    return true
  end
  return false
end


function enable_conveyor_plane(enable)
   if not fsm.vars.slide and if_plane_switch:has_writer() then
      local msg
      if enable then
         msg = if_plane_switch.EnableSwitchMessage:new()
      else
         msg = if_plane_switch.DisableSwitchMessage:new()
      end
      if_plane_switch:msgq_enqueue_copy(msg)
   end
end

function tolerance_ok()
   local pose = pose_offset()
   return math.abs(pose.x) <= tolerance_trans
      and math.abs(pose.y) <= tolerance_trans
      and math.abs(pose.ori) <= tolerance_ori
end

function fitness_ok()
  local local_fitness_threshold = 0
  if fsm.vars.slide then
    local_fitness_threshold = slide_euclidean_fitness_threshold
  else
    local_fitness_threshold = euclidean_fitness_threshold
  end

  print_info("conveyor_pose fitness: %f", if_conveyor_pose:euclidean_fitness())

  return if_conveyor_pose:euclidean_fitness() >= local_fitness_threshold
end

function result_ready()
  if if_conveyor_pose:is_busy()
     or if_conveyor_pose:msgid() ~= fsm.vars.msgid
  then return false end

  local bb_stamp = fawkes.Time:new(if_conveyor_pose:input_timestamp(0), if_conveyor_pose:input_timestamp(1))
  if not tf:can_transform("conveyor_pose", "base_link", bb_stamp) then
    return false
  end

  local transform = fawkes.tf.StampedTransform:new()
  tf:lookup_transform("conveyor_pose", "base_link", transform)
  if transform.stamp:in_usec() < bb_stamp:in_usec() then
    return false
  end
  return true
end

function pose_offset()
  if fsm.vars.shelf == "MIDDLE" then
    y_offset = y_offset_shelf_middle
  else
    y_offset = 0
  end
  local target_pos = { x = x_dist_to_mps,
                       y = y_offset,
                       ori = 0,
  }

  local conv_odom_tf = tfm.transform(target_pos, "conveyor_pose", "odom")
  local conv_odom_tf_age = tfm.tf_age("conveyor_pose", "odom")
  local odom_base_tf = tfm.transform(conv_odom_tf, "odom", "base_link")
  local odom_base_tf_age = tfm.tf_age( "odom", "base_link")

  local transformed_pos = tfm.transform(target_pos, "conveyor_pose", "base_link")
  local transformed_pos_age = tfm.tf_age("conveyor_pose", "base_link")

  print_info("conv_odom_tf is x = %f, y = %f,ori = %f, age = %f", conv_odom_tf.x, conv_odom_tf.y, conv_odom_tf.ori, conv_odom_tf_age)
  print_info("odom_base_tf is x = %f, y = %f,ori = %f, age = %f", odom_base_tf.x, odom_base_tf.y, odom_base_tf.ori, odom_base_tf_age)
  print_info("transformed_pos  is x = %f, y = %f,ori = %f, age = %f", transformed_pos.x, transformed_pos.y, transformed_pos.ori, transformed_pos_age)

  return transformed_pos
end


fsm:define_states{ export_to=_M,
   closure={ MAX_RETRIES=MAX_RETRIES, tolerance_ok=tolerance_ok,input_ok=input_ok,
      result_ready=result_ready, fitness_ok=fitness_ok , MAX_VISION_RETRIES=MAX_VISION_RETRIES},
   {"INIT", JumpState},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="CHECK_VISION", failed_to="FAILED"},
   {"CHECK_VISION", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="CHECK_VISION", failed_to="FAILED"},
   {"DECIDE_WHAT", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not input_ok()", desc = "Wrong input format"},
   {"INIT", "MOVE_GRIPPER", cond=true},
   {"CHECK_VISION", "FAILED", timeout=15, desc = "Fitness threshold wasn't reached"},
   {"CHECK_VISION", "FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "DECIDE_WHAT", cond=result_ready, desc="conveyor_pose result ready"},
   {"DECIDE_WHAT", "FINAL", cond="fitness_ok() and tolerance_ok()"},
   {"DECIDE_WHAT", "DRIVE", cond="fitness_ok() and not tolerance_ok() and vars.retries <= MAX_RETRIES"},
   {"DECIDE_WHAT", "FAILED", cond="vars.vision_retries > MAX_VISION_RETRIES"},
   {"DECIDE_WHAT", "CHECK_VISION", cond="not fitness_ok()"},
}

function INIT:init()
   enable_conveyor_plane(true)
   local parse_result = pam.parse_to_type_target(if_conveyor_pose,self.fsm.vars.place,self.fsm.vars.side,self.fsm.vars.shelf,self.fsm.vars.slide)
   self.fsm.vars.mps_type = parse_result.mps_type
   self.fsm.vars.mps_target = parse_result.mps_target
   self.fsm.vars.retries = 0
   self.fsm.vars.vision_retries = 0
end

function CHECK_VISION:init()
   local msg = if_conveyor_pose.RunICPMessage:new(self.fsm.vars.mps_type, self.fsm.vars.mps_target)
   if_conveyor_pose:msgq_enqueue_copy(msg)
   self.fsm.vars.msgid = msg:id()
   self.fsm.vars.vision_retries = self.fsm.vars.vision_retries + 1
end

function CHECK_VISION:exit()
   local msg = if_conveyor_pose.StopICPMessage:new()
   if_conveyor_pose:msgq_enqueue_copy(msg)
end

function MOVE_GRIPPER:init()
  self.args["gripper_commands"] = gripper_pose
  self.args["gripper_commands"].command = "MOVEABS"
end

function DRIVE:init()
   self.fsm.vars.retries = self.fsm.vars.retries + 1
   local pose = pose_offset(self)
   print_info("Drive forward, x = %f , y = %f", pose.x, pose.y)
   self.args["motor_move"].x = pose.x
   self.args["motor_move"].y = pose.y
   self.args["motor_move"].ori = pose.ori
   self.args["motor_move"].vel_trans = 0.03
   self.args["motor_move"].vel_rot = 0.06
   self.args["motor_move"].tolerance = {
      x = tolerance_trans * 0.9,
      y = tolerance_trans * 0.9,
      ori = tolerance_ori * 0.9
   }
end

function cleanup()
   enable_conveyor_plane(false)
end

function FAILED:init()
   cleanup()
end

function FINAL:init()
   cleanup()
end
