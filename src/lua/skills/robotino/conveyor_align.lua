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
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "if_conveyor", type = "Position3DInterface", id="conveyor_pose/pose"},
   {v = "conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
   {v = "conveyor_config", type = "ConveyorConfigInterface", id="conveyor_pose/config"},
   {v = "if_gripper", type = "AX12GripperInterface", id="Gripper AX12"},
}

documentation      = [==[aligns the robot orthogonal to the conveyor by using the
                         conveyor vision
                         @param product_present boolean If a product lies on the
                                                conveyor. The new conveyor vision
                                                needs this information in form of
                                                an interface message.
]==]

-- Initialize as skill module
-- TODO Argument if there is a product on the conveyor

skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

local TOLERANCE_Y = 0.002
local MAX_TRIES = 4
local Z_DEST_POS_WITH_PUCK = 0.002
local Z_DEST_POS_WITHOUT_PUCK = 0.008
local Z_DEST_POS = Z_DEST_POS_WITH_PUCK

function no_writer()
   return not if_conveyor:has_writer()
end

function see_conveyor()
   return if_conveyor:visibility_history() > 3
end

function tolerances_ok(self)
   local pose = pose_des(self)
   print("pose_y = " .. pose.y)
   if math.abs(pose.y) <= TOLERANCE_Y and math.abs(pose.z) <= TOLERANCE_Z and max_tries_not_reached(self) then
      return true
   end
end

function max_tries_not_reached(self)
   return (self.fsm.vars.counter < MAX_TRIES)
end

function pose_offset(self)
   if if_gripper:is_holds_puck() then
      Z_DEST_POS = Z_DEST_POS_WITH_PUCK
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
            y = cp.y,
            z = cp.z,
            ori = ori
          }
end

--TODO check vis_hist

fsm:define_states{ export_to=_M,
   closure={MAX_TRIES=MAX_TRIES},
   {"INIT", JumpState},
   {"DRIVE_YZ", SkillJumpState, skills={{motor_move}, {ax12gripper}}, final_to="SETTLE", fail_to="FAILED"},
   {"SETTLE", JumpState},
   {"CHECK", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=no_writer, desc="no conveyor vision"},
   {"INIT", "SETTLE", cond=true},
   {"SETTLE", "CHECK", timeout=0.5},
   {"CHECK", "FAILED", cond="vars.counter > MAX_TRIES", desc="max tries reached"},
   {"CHECK", "DRIVE_YZ", cond=tolerance_y_not_ok, desc="tolerance not ok, align y distance"},
   {"CHECK", "FINAL", cond=true},
}

function INIT:init()
   self.fsm.vars.counter = 0
   conveyor_switch:msgq_enqueue_copy(conveyor_switch.EnableSwitchMessage:new())
   if self.fsm.vars.product_present then
      conveyor_config:msgq_enqueue_copy(conveyor_config.EnableProductRemovalMessage:new())
   end
end

function DRIVE_YZ:init()
   if gripper_if:is_holds_puck() then
      Z_DEST_POS = Z_DEST_POS_WITH_PUCK
   else
      Z_DEST_POS = Z_DEST_POS_WITHOUT_PUCK
   end
   self.args["motor_move"] = {y = conveyor_0:translation(1), tolerance = { x=0.002, y=0.002, ori=0.01 }}
   self.args["ax12gripper"].command = "RELGOTOZ"
   self.args["ax12gripper"].z_position = 0 --(Z_DEST_POS * 1000)
end

function cleanup()
   conveyor_switch:msgq_enqueue_copy(conveyor_switch.DisableSwitchMessage:new())
   -- Disable the product removal flag by default
   conveyor_config:msgq_enqueue_copy(conveyor_config.DisableProductRemovalMessage:new())
end

function FINAL:init()
   cleanup()
end

function FAILED:init()
   cleanup()
end
