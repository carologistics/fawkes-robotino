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
local euclidean_fitness_threshold = 8 -- threshold for euclidean fitness for targets other than shelf
local shelf_euclidean_fitness_threshold = 3 -- threshold for euclidean fitness if target is shelf
local tolerance_trans = 0.02  -- tolerance in x and y direction after the align
local tolerance_ori = 0.025   -- orientation tolerance after the align
local x_dist_to_mps = -0.31  -- x-distance the robot should have after the align
local y_offset_shelf_middle = -0.015 -- y-offset the robot should have picking is done from shelf = "MIDDLE"

-- initial gripper poses depending on the target
local GRIPPER_POSES = {
  shelf_left={x=0.05, y=0.00, z=0.035},
  shelf_middle={x=0.05, y=-0.035, z=0.035},
  shelf_right={x=0.05, y=0.00, z=0.035},
  slide={x=0.05,y=0.00,z=0.035},
  output_conveyor={x=0.05, y=0.00,z=0.045},
  input_conveyor={x=0.035, y=0.00,z=0.045},
}

local MAX_RETRIES=3

function no_writer()
   return not if_conveyor_pose:has_writer()
end

function tolerance_ok()
   local pose = pose_offset()
   return math.abs(pose.x) <= tolerance_trans
      and math.abs(pose.y) <= tolerance_trans
      and math.abs(pose.ori) <= tolerance_ori
end

function fitness_ok()
  local local_fitness_threshold = 0
  if fsm.vars.shelf then
    local_fitness_threshold = shelf_euclidean_fitness_threshold
  else
    local_fitness_threshold = euclidean_fitness_threshold
  end
  
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
  local transformed_pos = tfm.transform(target_pos, "conveyor_pose", "base_link")
  print_info("transformed_pos is x = %f, y = %f,ori = %f", transformed_pos.x, transformed_pos.y, transformed_pos.ori)
  return transformed_pos
end


fsm:define_states{ export_to=_M,
   closure={ MAX_RETRIES=MAX_RETRIES, tolerance_ok=tolerance_ok,
      result_ready=result_ready, fitness_ok=fitness_ok },
   {"INIT", JumpState},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="CHECK_VISION", failed_to="FAILED"},
   {"CHECK_VISION", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="CHECK_VISION", failed_to="FAILED"},
   {"DECIDE_WHAT", JumpState},
}

fsm:add_transitions{
   {"INIT", "MOVE_GRIPPER", cond=true},
   {"CHECK_VISION", "FAILED", timeout=10, desc = "Fitness threshold wasn't reached"},
   {"CHECK_VISION", "FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "DECIDE_WHAT", cond=result_ready, desc="Fitness threshold reached"},
   {"DECIDE_WHAT", "FINAL", cond="fitness_ok() and tolerance_ok()"},
   {"DECIDE_WHAT", "DRIVE", cond="fitness_ok() and not tolerance_ok() and vars.retries <= MAX_RETRIES"},
   {"DECIDE_WHAT", "FAILED", cond="fitness_ok() and not tolerance_ok()"},
   {"DECIDE_WHAT", "CHECK_VISION", cond="not fitness_ok()"}
}

function INIT:init()
   if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.EnableSwitchMessage:new())
   local parse_result = pam.parse_to_type_target(if_conveyor_pose,self.fsm.vars.place,self.fsm.vars.side,self.fsm.vars.shelf,self.fsm.vars.slide)
   self.fsm.vars.mps_type = parse_result.mps_type
   self.fsm.vars.mps_target = parse_result.mps_target
   self.fsm.vars.retries = 0
end

function CHECK_VISION:init()
   local msg = if_conveyor_pose.SetStationMessage:new(self.fsm.vars.mps_type, self.fsm.vars.mps_target)
   if_conveyor_pose:msgq_enqueue_copy(msg)
   self.fsm.vars.msgid = msg:id()
end

function MOVE_GRIPPER:init()

  if self.fsm.vars.shelf == "LEFT" then
    self.args["gripper_commands_new"] = GRIPPER_POSES["shelf_left"]
  elseif self.fsm.vars.shelf == "RIGHT" then
    self.args["gripper_commands_new"] = GRIPPER_POSES["shelf_right"]
  elseif self.fsm.vars.shelf == "MIDDLE" then
    self.args["gripper_commands_new"] = GRIPPER_POSES["shelf_middle"]
  elseif self.fsm.vars.slide then
    self.args["gripper_commands_new"] = GRIPPER_POSES["slide"]
  elseif self.fsm.vars.side == "input" then
    self.args["gripper_commands_new"] = GRIPPER_POSES["input_conveyor"]
  elseif self.fsm.vars.side == "output" then
    self.args["gripper_commands_new"] = GRIPPER_POSES["output_conveyor"]
  end
  self.args["gripper_commands_new"].command = "MOVEABS"
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
  if_conveyor_switch:msgq_enqueue_copy(if_conveyor_switch.DisableSwitchMessage:new())
end

function FAILED:init()
   cleanup()
end

function FINAL:init()
   if (fsm.vars.disable_realsense_afterwards == nil or fsm.vars.disable_realsense_afterwards) then
      cleanup()
   end
end

