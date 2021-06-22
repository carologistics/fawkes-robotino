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
-- TODO:get interface names from congic
depends_interfaces = {
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "if_conveyor_pose", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
   {v = "if_plane_switch", type = "SwitchInterface", id="conveyor_plane/switch"},
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
   {v = "realsense_switch", type = "SwitchInterface", id="realsense2"},
}

documentation      = [==[aligns the robot orthogonal to the conveyor by using the
                         conveyor vision
Parameters:
       @param disable_realsense_afterwards   disable the realsense after aligning
       @param place   the name of the MPS (see navgraph e.g.: "M-BS" for base station of team magenta)
       @param side    optional the side of the mps
       @param slide   optional true if you want to put it on the slide
]==]

-- Initialize as skill module

skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local pam = require("parse_module")

-- Constants
local FITNESS_THRESHOLD = {           -- low is the minimum required fitness to do anything
   conveyor = { low = 15, high = 25 }, -- if fitness is >= high, we assume that the fit is perfect
   slide = { low = 15, high = 25 }     -- and don't re-run ICP after moving
}

local GRIP_OFFSET = {
   x = 0.03,
   y = 0,
   z = 0.02,
   ori = { x = 0, y = 0, z = 0, w = 1 }
}
local TARGET_POS = { -- target pose rel. to the conveyor_pose frame
   x = -0.255,
   y = 0,
   ori = 0
}

-- initial gripper poses depending on the target
local GRIPPER_POSE = { x= 0.05, y = 0.00, z = 0.03}

local MAX_RETRIES=2
local MAX_VISION_RETRIES=2


local gripper_max = {
   x = config:get_float("/arduino/x_max"),
   y = config:get_float("/arduino/y_max") / 2,
   z = config:get_float("/arduino/z_max"),
}
local gripper_min = {
   x = config:get_float("/arduino/x_min"),
   y = config:get_float("/arduino/y_max") / -2,
   z = config:get_float("/arduino/z_min"),
}

function no_writer()
   return not if_conveyor_pose:has_writer()
end

function input_ok()
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
   local gripper_odom = tfm.transform6D(GRIP_OFFSET, "conveyor_pose", "odom")
   if not gripper_odom then
      return nil
   end

   local gripper_dist = tfm.transform6D(gripper_odom, "odom", "gripper")
   if not gripper_dist then
      return nil
   end

   local q = fawkes.tf.Quaternion:new(
      gripper_dist.ori.x,
      gripper_dist.ori.y,
      gripper_dist.ori.z,
      gripper_dist.ori.w
   )
   local yaw = fawkes.tf.get_yaw(q)
   
   print_info("gripper_dist: %f, %f, ,%f, %f",
      gripper_dist.x,
      gripper_dist.y,
      gripper_dist.z,
      yaw
   )

   --For debuging
   print("x: " .. tostring(gripper_dist.x))
   print("y: " .. tostring(gripper_dist.y))
   print("z: " .. tostring(gripper_dist.z))
   print(tostring(gripper_min.x - GRIPPER_POSE.x))
   print(tostring(gripper_min.y - GRIPPER_POSE.y))
   print(tostring(gripper_min.z - GRIPPER_POSE.z))
   if  gripper_dist.x < gripper_max.x - GRIPPER_POSE.x then print_info("1") end
   if  gripper_dist.x > gripper_min.x - GRIPPER_POSE.x  then print_info("2") end
   if  gripper_dist.y < 0.8 * (gripper_max.y - GRIPPER_POSE.y)  then print_info("3") end
   if  gripper_dist.y > 0.8 * (gripper_min.y - GRIPPER_POSE.y)  then print_info("4") end
   if  gripper_dist.z < gripper_max.z - GRIPPER_POSE.z then print_info("5") end
   if  gripper_dist.z > gripper_min.z - GRIPPER_POSE.z then print_info("6") end
   if  math.abs(yaw) <= 0.05 then print_info("7") end


   return gripper_dist.x < gripper_max.x - GRIPPER_POSE.x
      and gripper_dist.x > gripper_min.x - GRIPPER_POSE.x
      and gripper_dist.y < 0.8 * (gripper_max.y - GRIPPER_POSE.y)
      and gripper_dist.y > 0.8 * (gripper_min.y - GRIPPER_POSE.y)
      and gripper_dist.z < gripper_max.z - GRIPPER_POSE.z
      and gripper_dist.z > gripper_min.z - GRIPPER_POSE.z
      and math.abs(yaw) <= 0.05
end

function motor_move_tolerance()
   -- Compute motor_move tolerance as a function of the available gripper range
   return {
      x = math.max(
         0.01,           -- lower bound
         0.5 * math.min( -- gripper range
            math.abs(gripper_max.x - GRIPPER_POSE.x),
            math.abs(gripper_min.x - GRIPPER_POSE.x)
         )
      ),
      y = math.max(
         0.01,           -- lower bound
         0.5 * math.min( -- gripper range
            math.abs(gripper_max.y - GRIPPER_POSE.y),
            math.abs(gripper_min.y - GRIPPER_POSE.y)
         )
      ),
      ori = 0.04
   }
end

function fitness_min()
  print("euclidean fit: " .. tostring(if_conveyor_pose:euclidean_fitness()))
  return if_conveyor_pose:euclidean_fitness() >= FITNESS_THRESHOLD[fsm.vars.target].low
end

function fitness_high()
  print("euclidean fit: " .. tostring(if_conveyor_pose:euclidean_fitness()))
  return if_conveyor_pose:euclidean_fitness() >= FITNESS_THRESHOLD[fsm.vars.target].high
end

function fitness_low()
  return fitness_min() and not fitness_high()
end

function result_ready()
  if if_conveyor_pose:is_busy()
     or if_conveyor_pose:msgid() ~= fsm.vars.msgid
  then
    return false
  end

  if if_conveyor_pose:msgid() == fsm.vars.msgid
     and not if_conveyor_pose:is_busy()
  then
    return true
  end

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


fsm:define_states{ export_to=_M,
   closure={ MAX_RETRIES=MAX_RETRIES, tolerance_ok=tolerance_ok,input_ok=input_ok,
      result_ready=result_ready, fitness_min=fitness_min , MAX_VISION_RETRIES=MAX_VISION_RETRIES,
      fitness_low=fitness_low, fitness_high=fitness_high},
   {"INIT", JumpState},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="LOOK", failed_to="FAILED"},
   {"LOOK", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="DRIVE_DONE", failed_to="FAILED"},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="LOOK", failed_to="FAILED"},
   {"LOOK_DONE", JumpState},
   {"DRIVE_DONE", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not input_ok()", desc = "Wrong input format"},
   {"INIT", "MOVE_GRIPPER", cond=true},
   
   {"LOOK", "FAILED", timeout=15, desc = "Fitness threshold wasn't reached"},
   {"LOOK", "FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"LOOK", "LOOK_DONE", cond=result_ready, desc="conveyor_pose result ready"},
   
   {"LOOK_DONE", "FAILED", cond="vars.retries > MAX_RETRIES"},
   {"LOOK_DONE", "FAILED", cond="vars.vision_retries > MAX_VISION_RETRIES"},
   {"LOOK_DONE", "LOOK", cond="tolerance_ok() == nil", desc="TF error"},
   {"LOOK_DONE", "FINAL", cond="fitness_high()"},
   {"LOOK_DONE", "DRIVE", cond="fitness_min()"},
   {"LOOK_DONE", "MOVE_BACK", cond="not fitness_min()"},

   {"DRIVE_DONE", "FINAL", cond="fitness_high()"},
   {"DRIVE_DONE", "FAILED", cond="vars.vision_retries > MAX_VISION_RETRIES"},
   {"DRIVE_DONE", "LOOK", cond="not fitness_high()"},
}

function INIT:init()
   laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())
   realsense_switch:msgq_enqueue(realsense_switch.EnableSwitchMessage:new())

   enable_conveyor_plane(true)
   local parse_result = pam.parse_to_type_target(
      if_conveyor_pose,
      self.fsm.vars.place,
      self.fsm.vars.side,
      nil,
      self.fsm.vars.slide
   )
   self.fsm.vars.mps_type = parse_result.mps_type
   self.fsm.vars.mps_target = parse_result.mps_target
   self.fsm.vars.retries = 0
   self.fsm.vars.vision_retries = 0

   if self.fsm.vars.slide then
      self.fsm.vars.target = "slide"
   else
      self.fsm.vars.target = "conveyor"
   end
end

function LOOK:init()
   local msg = if_conveyor_pose.RunICPMessage:new(self.fsm.vars.mps_type, self.fsm.vars.mps_target)
   if_conveyor_pose:msgq_enqueue_copy(msg)
   self.fsm.vars.msgid = msg:id()
   self.fsm.vars.vision_retries = self.fsm.vars.vision_retries + 1
end

function LOOK:exit()
   local msg = if_conveyor_pose.StopICPMessage:new()
   if_conveyor_pose:msgq_enqueue_copy(msg)
   
   self.fsm.vars.target_pos_odom = tfm.transform(TARGET_POS, "conveyor_pose", "odom")
   print_info("conveyor_pose fitness: %f", if_conveyor_pose:euclidean_fitness())
end

function MOVE_BACK:init()
   self.args["motor_move"].x = -0.015
   self.args["motor_move"].tolerance = { x = 0.01 }
end

function MOVE_GRIPPER:init()
  self.args["gripper_commands"] = GRIPPER_POSE
   print_info("conveyor_pose fitness: %f", if_conveyor_pose:euclidean_fitness())
  self.args["gripper_commands"].command = "MOVEABS"
end

function DRIVE:init()
   print_info("conveyor_pose fitness: %f", if_conveyor_pose:euclidean_fitness())
   
   self.fsm.vars.retries = self.fsm.vars.retries + 1
   self.args["motor_move"].x = self.fsm.vars.target_pos_odom.x
   self.args["motor_move"].y = self.fsm.vars.target_pos_odom.y
   self.args["motor_move"].ori = self.fsm.vars.target_pos_odom.ori
   self.args["motor_move"].frame = "/odom"
   self.args["motor_move"].vel_trans = 0.03
   self.args["motor_move"].vel_rot = 0.06
   self.args["motor_move"].tolerance = motor_move_tolerance()
end

function cleanup()
   enable_conveyor_plane(false)
   realsense_switch:msgq_enqueue(realsense_switch.DisableSwitchMessage:new())
end

function FAILED:init()
   cleanup()
end

function FINAL:init()
   cleanup()
end
