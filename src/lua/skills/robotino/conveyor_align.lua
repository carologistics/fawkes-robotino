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

local TOLERANCE_Y = 0.003
local TOLERANCE_Z = 0.003
local MAX_TRIES = 10
--local X_DEST_POS = 0.08
local X_DEST_POS = 0.14
local Y_DEST_POS = 0.0
local Z_DEST_POS = 0.052
local Z_DEST_POS_WITH_PUCK = 0.056
local cfg_frame_ = "gripper"

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

function pose_des(self)
   local pose = pose_offset(self)
   pose.x = pose.x - X_DEST_POS
   pose.y = pose.y - Y_DEST_POS
   pose.z = pose.z + Z_DEST_POS
   return pose
end

fsm:define_states{ export_to=_M,
   closure={},
   {"INIT", JumpState},
   {"CHECK_VISION", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}, {ax12gripper}}, final_to="DECIDE_TRY", fail_to="FAILED"},
   {"DECIDE_TRY", JumpState},
}

fsm:add_transitions{
   {"INIT", "CHECK_VISION", cond=true},
   {"CHECK_VISION", "FAILED", timeout=20, desc="No vis_hist on conveyor vision"},
   {"CHECK_VISION", "FAILED", cond=no_writer, desc="No writer for conveyor vision"},
   {"CHECK_VISION", "DRIVE", cond=see_conveyor},
   {"DECIDE_TRY", "FINAL", cond=tolerances_ok, desc="Robot is aligned"},
   {"DECIDE_TRY", "CHECK_VISION", cond=max_tries_not_reached, desc="Do another alignment"},
   {"DECIDE_TRY", "FAILED", cond=true, desc="Couldn't align within MAX_TRIES"},
}

function INIT:init()
   self.fsm.vars.counter = 0
   conveyor_switch:msgq_enqueue_copy(conveyor_switch.EnableSwitchMessage:new())
   if self.fsm.vars.product_present then
      conveyor_config:msgq_enqueue_copy(conveyor_config.EnableProductRemovalMessage:new())
   end
end

function DECIDE_TRY:init()
   self.fsm.vars.counter = self.fsm.vars.counter + 1
   print("Try number " .. self.fsm.vars.counter)
end

function DRIVE:init()
   local pose = pose_des(self)

   self.args["motor_move"] = {x = pose.x, y = pose.y, tolerance = { x=0.002, y=0.002, ori=0.01 }, vel_trans = 0.4} --TODO set tolerances as defined in the global variable
   local z_position = pose.z * 1000
   print("z_pose: " .. z_position)
   self.args["ax12gripper"].command = "RELGOTOZ"
   if math.abs(pose.z) >= TOLERANCE_Z then
      self.args["ax12gripper"].z_position = z_position
   else
      self.args["ax12gripper"].z_position = 0
   end
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
