
----------------------------------------------------------------------------
--  product_pick.lua
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
fsm                = SkillHSM:new{name=name, start="OPEN_GRIPPER", debug=false}
depends_skills     = {"ax12gripper", "approach_mps"}
depends_interfaces = {
   {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"}
   {v = "if_conv_pos", type = "ConveyorPoseInterface", id="conveyor_pose/status"},
   {v = "conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
}

documentation      = [==[The robot needs to be aligned with the machine, then just drives forward
and opens the gripper
@param offset_x the offset_x from the navgraph point
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
local cfg_frame_ = "gripper"


function no_writer()
   return not if_conveyor:has_writer()
end

function see_conveyor()
   return if_conv_pos:euclidean_fitness()
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
   {"ADJUST_GRIPPER", SkillJumpState,skills{{}},final_to="CHECK_VISION",fail_to="FAIL"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{}},final_to="MOVE_GRIPPER", fail_to="FAIL"},
   {"MOVE_GRIPPER", SkillJumpState, skills={{}}, final_to="CLOSE_GRIPPER",fail_to="FAIL"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{}}, final_to = "CLOSE_GRIPPER", fail_to="FAIL"},
   {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{}}, final_to = "FINAL", fail_to="FAIL"},
}

fsm:add_transitions{
   {"CHECK_VISION", "OPEN_GRIPPER",cond=valid_pose , desc= "Conveyor Pose in Range"},
   {"CHECK_VISION", "ADJUST_GRIPPER",cond=valid_pose, desc=" Pose offset to high"},
}
