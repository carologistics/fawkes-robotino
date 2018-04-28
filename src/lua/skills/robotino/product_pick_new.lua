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
name               = "product_pick_new"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands_new", "motor_move"}
depends_interfaces = {
   {v = "if_conv_pos", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
   {v = "conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
}

documentation      = [==[The robot needs to be aligned with the machine, then checks for pose of the conveyor_pose
  and adapts the gripper position if necessary. It then uses the gripper to actually pick the product.
and opens the gripper
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
--local x_distance = 0.315
--local X_DEST_POS = 0.16
--local Y_DEST_POS = 0.0
--local Z_DEST_POS = 0.056
--local Z_DEST_POS_WITH_PUCK = 0.06
local X_OFFSET_RANGE = 0.05
local Y_OFFSET_RANGE = 0.05
local Z_OFFSET_RANGE = 0.05
local gripper_x = 0
local gripper_y = 0
local gripper_z = 0
local gripper_back_z = 0
local drive_back_distance = -0.2
local cfg_frame_ = "gripper"


function no_writer()
   return not if_conveyor:has_writer()
end

function pose_offset(self)
   local from = { x = if_conv_pos:translation(0),
                  y = if_conv_pos:translation(1),
                  z = if_conv_pos:translation(2),
                  ori = { x = if_conv_pos:rotation(0),
                          y = if_conv_pos:rotation(1),
                          z = if_conv_pos:rotation(2),
                          w = if_conv_pos:rotation(3),
                        }
                 }

   print("z_pose intial: " .. cp.z)
   return { x = cp.x,
            y = cp.y,
            z = cp.z,
            ori = ori
          }
end

function pose_des(self)
   local pose = pose_offset(self)
   --pose.x = pose.x - X_DEST_POS
   --pose.y = pose.y - Y_DEST_POS
   --pose.z = pose.z + Z_DEST_POS
   return pose
end

function valid_pose(self)
  local pose = pose_offset(self)
  if pose.x > X_OFFSET_RANGE then
    return false
  end
  if pose.y > Y_OFFSET_RANGE then
    return false
  end
  if pose.z > Z_OFFSET_RANGE then
    return false
  end
  return true
end

fsm:define_states{ export_to=_M, closure={gripper_if=gripper_if},
   {"INIT", JumpState},
   {"CHECK_VISION", JumpState},
--   {"ADJUST_GRIPPER", SkillJumpState, skills={{gripper_commands_new}},final_to="CHECK_VISION",fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}},final_to="MOVE_GRIPPER_YZ", fail_to="FAILED"},
   {"MOVE_GRIPPER_YZ", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_X",fail_to="FAILED"},
   {"MOVE_GRIPPER_X", SkillJumpState, skills={{gripper_commands_new}}, final_to="CLOSE_GRIPPER",fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to = "MOVE_GRIPPER_BACK", fail_to="FAILED"},
   {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands_new}}, final_to = "DRIVE_BACK", fail_to="FAILED"},
   {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"CHECK_VISION", "OPEN_GRIPPER",cond=true , desc= "Conveyor Pose in Range"},
   --{"CHECK_VISION", "ADJUST_GRIPPER",cond=true, desc=" Pose offset to high"},
}


function OPEN_GRIPPER:init()
  self.args["gripper_commands_new"].command = "OPEN"
end

function MOVE_GRIPPER_YZ:init()
  self.args["gripper_commands_new"].x = gripper_z
  self.args["gripper_commands_new"].y = gripper_y
  self.args["gripper_commands_new"].command = "MOVEABS"
end

function MOVE_GRIPPER_X:init()
  self.args["gripper_commands_new"].z = gripper_x
  self.args["gripper_commands_new"].command = "MOVEABS"
end

function CLOSE_GRIPPER:init()
  self.args["gripper_commands_new"].command = "CLOSE"
end

function MOVE_GRIPPER_BACK:init()
  self.args["gripper_commands_new"].z = gripper_back_z
  self.args["gripper_commands_new"].command = "MOVEABS"
end

function DRIVE_BACK:init()
   self.args["motor_move"].x = drive_back_distance
end
