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
depends_skills     = {"motor_move","approach_mps", "ax12gripper"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "if_conveyor", type = "Position3DInterface", id="conveyor_pose/pose"},
   {v = "if_gripper", type = "AX12GripperInterface", id="Gripper AX12"},
}

documentation      = [==[aligns the robot orthogonal to the conveyor by using the
conveyor vision
]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

local TOLERANCE_Y = 0.005
local TOLERANCE_Z = 0.002
local MAX_TRIES = 4
local X_DEST_POS_PRE = 0.235
local X_DEST_POS = 0.19
local Z_DEST_POS_WITH_PUCK = 0.032
local Z_DEST_POS_WITHOUT_PUCK = 0.03
local Z_DEST_POS = Z_DEST_POS_WITH_PUCK
local Z_DIVISOR = 2
local cfg_frame_ = "mount_profile"

function no_writer()
   return not if_conveyor:has_writer()
end

function see_conveyor()
   return if_conveyor:visibility_history() > 0
end

function tolerance_y_not_ok()
   return not (math.abs(if_conveyor:translation(1)) <= TOLERANCE_Y)
end

function tolerance_z_not_ok()
   return not (math.abs(if_conveyor:translation(2) - Z_DEST_POS) <= TOLERANCE_Z)
end

function check_for_second_try(self)
   return self.fsm.vars.counter >= 0
end

function pose_offset(self)
   if if_gripper:is_holds_puck() then
      Z_DEST_POS = Z_DEST_POS_WITH_PUCK
   else
      Z_DEST_POS = Z_DEST_POS_WITHOUT_PUCK
   end

   local from = { x = if_conveyor:translation(0),
                  y = if_conveyor:translation(1),
                  z = if_conveyor:translation(2),
                  ori = { x = if_conveyor:rotation(0),
                          y = if_conveyor:rotation(1),
                          z = if_conveyor:rotation(2),
                          w = if_conveyor:rotation(3),
                        }
                }
   local cp = tfm.transform6D(from, if_conveyor:frame(), cfg_frame_)

   -- TODO check nil

   local ori = fawkes.tf.get_yaw( fawkes.tf.Quaternion:new(cp.ori.x, cp.ori.y, cp.ori.z, cp.ori.w))
   print("ori want: ".. ori)
   if math.abs(ori) > 0.7 then
      ori = 0
   end

   return { x = cp.x,
            y = cp.y -0.025,
            z = cp.z - Z_DEST_POS,
            ori = ori
          }
end

function pose_des_pre(self)
   local pose = pose_offset(self)
   pose.x = pose.x - X_DEST_POS_PRE
   return pose
end

function pose_des(self)
   local pose = pose_offset(self)
   pose.x = pose.x - X_DEST_POS
   return pose
end

fsm:define_states{ export_to=_M,
   closure={MAX_TRIES=MAX_TRIES, Z_DEST_POS=Z_DEST_POS},
   {"INIT", JumpState},
   {"APPROACH", SkillJumpState, skills={{approach_mps}}, final_to="DRIVE_PRE", fail_to="FAILED"},
   {"SETTLE_PRE", JumpState},
   {"DRIVE_PRE", SkillJumpState, skills={{motor_move}, {ax12gripper}}, final_to="DECIDE_TRY", fail_to="FAILED"},
   {"DECIDE_TRY", JumpState},
   {"ROTATE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"DRIVE_FINAL", SkillJumpState, skills={{motor_move}, {ax12gripper}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=no_writer, desc="no conveyor vision"},
   {"INIT", "FAILED", timeout=2, desc="no VIS HIST on conveyor vision"},
   {"INIT", "APPROACH", cond=see_conveyor},
   {"SETTLE_PRE", "DRIVE_PRE", cond=true},
   {"DECIDE_TRY", "SETTLE_PRE", cond=check_for_second_try, desc="do 2. alignmend"},
   {"DECIDE_TRY", "FINAL", cond=true, desc="pre aligned"},
}

function INIT:init()
   self.fsm.vars.counter = 0
end

function APPROACH:init()
   self.args["approach_mps"] = { x = 0.2 }
end

function ROTATE:init()
   local pose = pose_des_pre(self)
   self.args["motor_move"] = {ori = pose.ori, tolerance = { x=0.002, y=0.002, ori=0.01 }, vel_rot = 0.2}
end

function DECIDE_TRY:init()
   self.fsm.vars.counter = self.fsm.vars.counter - 1
end

function DRIVE_PRE:init()
   local pose = pose_des_pre(self)

   self.args["motor_move"] = {x = pose.x, y = pose.y, tolerance = { x=0.002, y=0.002, ori=0.01 }}
   self.args["ax12gripper"].command = "RELGOTOZ"
   if tolerance_z_not_ok() then
      self.args["ax12gripper"].z_position = (pose.z * 1000) / Z_DIVISOR
   end
end
