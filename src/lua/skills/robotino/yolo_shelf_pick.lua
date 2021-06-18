----------------------------------------------------------------------------
--  yolo_shelf_pick.lua - pick from shelf using yolo
--  workpiece vision
--
--  Copyright  2021 The Carologistics Team
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
name               = "yolo_shelf_pick"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "reset_gripper", "gripper_commands", "workpiece_pose"}
depends_interfaces = {
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
   {v = "workpiece_pose_if", type = "WorkpiecePoseInterface", id="WorkpiecePose"},
}

documentation      = [==[picks from the shelf using yolo detection
Parameters:
]==]

-- Initialize as skill module

skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local pam = require("parse_module")

-- Constants

local gripper_pose_offset_x = 0.013   -- conveyor pose offset in x direction
local gripper_pose_offset_y = 0.0   -- conveyor_pose offset in y direction
local gripper_pose_offset_z = 0.033  -- conveyor_pose offset in z direction

-- initial gripper poses depending on the target
local GRIPPER_POSE = { x= 0.0, y = 0.0, z = 0.0}

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

function pose_gripper_offset(x,y,z)
   local target_pos = { x = x,
                         y = y,
                         z = z,
                         ori = { x = 0, y = 0, z = 0, w = 1}
 
    }
    local tmp = { x = 0,
                  y = 0,
                  z = 0,
                  ori = { x = 0, y = 0, z = 0, w = 1}
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
    return { x = math.max(0,math.min(gripper_home_rel.x, gripper_max.x)),
             y = math.max(gripper_max.y,math.min(gripper_home_rel.y,gripper_max.y)),
             z = math.max(0,math.min(gripper_home_rel.z,gripper_max.z))}
 end

function tf_stamp_okay(self)
   local bb_stamp = fawkes.Time:new(workpiece_pose_if:input_timestamp(0), workpiece_pose_if:input_timestamp(1))
   tf:lookup_transform("gripper", "workpiece_pose", self.fsm.vars.gripper_to_wp)
   if self.fsm.vars.gripper_to_wp.stamp:in_usec() > bb_stamp:in_usec() then
     print("lookup")
     return false
   end
   if not tf:can_transform("workpiece_pose", "gripper", self.fsm.vars.gripper_to_wp.stamp) then
      print("transform")
      return false
   end

   return true
end

function wait_for_stamp(self)
  return self.fsm.vars.wp_stamp:in_usec() < fawkes.Time:new(workpiece_pose_if:input_timestamp(0), workpiece_pose_if:input_timestamp(1)):in_usec()
end

fsm:define_states{ export_to=_M,
   closure={MAX_RETRIES=MAX_RETRIES, tf_stamp_okay=tf_stamp_okay, wait_for_stamp=wait_for_stamp},
   {"INIT", JumpState},
   {"LOOK", SkillJumpState, skills={{workpiece_pose}}, final_to="NEW_STAMP", failed_to="LOOK"},
   {"NEW_STAMP", JumpState},
   {"WP_POSE_AVAILABLE", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="OPEN_GRIPPER", failed_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER", failed_to="FAILED"},
   {"MOVE_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="GRIPPER_DOWN", failed_to="FAILED"},
   {"GRIPPER_DOWN", SkillJumpState, skills={{gripper_commands}}, final_to="GRAB", failed_to="FAILED"},
   {"GRAB", SkillJumpState, skills={{gripper_commands}}, final_to="GRIPPER_UP", failed_to="FAILED"},
   {"GRIPPER_UP", SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_AWAY", failed_to="FAILED"},
   {"MOVE_AWAY", SkillJumpState, skills={{motor_move},{reset_gripper}}, final_to="FINAL", failed_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "LOOK", cond=true},
   {"LOOK", "FAILED", cond="vars.yolo_tries > MAX_RETRIES", desc="No Pose after maximum amount of tries"},
   {"NEW_STAMP", "FAILED", timeout=5, desc="No Frame"},
   {"NEW_STAMP", "WP_POSE_AVAILABLE", cond="wait_for_stamp(self)"},
   {"WP_POSE_AVAILABLE", "DRIVE", cond="tf_stamp_okay(self)", desc="Workpiece Pose available"},
   {"WP_POSE_AVAILABLE", "FAILED", cond="not tf_stamp_okay(self)", desc="Workpiece Pose not available"},
}

function INIT:init()
   self.fsm.vars.gripper_to_wp = fawkes.tf.StampedTransform:new()
   self.fsm.vars.yolo_tries = 0
   self.fsm.vars.wp_stamp = fawkes.Time:new(workpiece_pose_if:input_timestamp(0), workpiece_pose_if:input_timestamp(1))
end

function LOOK:init()
   self.args["workpiece_pose"].location="MIDDLE"
   self.fsm.vars.yolo_tries = self.fsm.vars.yolo_tries + 1
end

function DRIVE:init()
   self.args["motor_move"].x = 0.02
end

function OPEN_GRIPPER:init()
   self.args["gripper_commands"].command = "OPEN"
end

function MOVE_GRIPPER:init()
  local target_pos = { x = gripper_pose_offset_x,
                        y = gripper_pose_offset_y,
                        z = gripper_pose_offset_z,
                        ori = { x = 0, y = 0, z = 0, w = 1}
   }
  local grip_pos_odom = tfm.transform6D(target_pos, "workpiece_pose", "odom")
  local grip_pos = tfm.transform6D(grip_pos_odom, "odom", "gripper")

   --print(tostring(self.fsm.vars.gripper_to_wp:getOrigin():x()))
   --GRIPPER_POSE.x = math.max(0, math.min(self.fsm.vars.gripper_to_wp:getOrigin():x(), gripper_max.x))
   --GRIPPER_POSE.y = math.max(-gripper_max.y, math.min(self.fsm.vars.gripper_to_wp:getOrigin():y(), gripper_max.y))
   --GRIPPER_POSE.z = math.max(0, math.min(self.fsm.vars.gripper_to_wp:getOrigin():z(), gripper_max.z))
   GRIPPER_POSE = pose_gripper_offset(grip_pos.x,grip_pos.y, grip_pos.z)
   GRIPPER_POSE.x = GRIPPER_POSE.x - 0.004 
   GRIPPER_POSE.y = GRIPPER_POSE.y - 0.035 
   GRIPPER_POSE.z = 0.0
   self.args["gripper_commands"] = GRIPPER_POSE
   self.args["gripper_commands"].command = "MOVEABS"
   self.args["gripper_commands"].target_frame = "gripper_home"
end

function GRIPPER_DOWN:init()
   self.args["gripper_commands"].x = 0.0
   self.args["gripper_commands"].y = 0.0
   --self.args["gripper_commands"].z = 0.0365
   self.args["gripper_commands"].z = 0.0
   self.args["gripper_commands"].command = "MOVEREL"
end
function GRAB:init()
   self.args["gripper_commands"].command = "CLOSE"
end

function GRIPPER_UP:init()
   self.args["gripper_commands"].x = 0.0
   self.args["gripper_commands"].y = 0.0
   self.args["gripper_commands"].z = 0.01
   self.args["gripper_commands"].command = "MOVEREL"
end

function MOVE_AWAY:init()
   self.args["motor_move"].x = -0.02
end
