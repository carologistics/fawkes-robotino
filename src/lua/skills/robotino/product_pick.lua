----------------------------------------------------------------------------
--  product_pick.lua
--
--  Created Wed Apr 15
--  Copyright  2015  Johannes Rothe
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
depends_skills     = {"gripper_commands", "motor_move"}
depends_interfaces = {
}

documentation      = [==[
Skill to pick a product from the conveyor.

Parameters:
      @param place   the name of the MPS (see navgraph e.g.: "M-BS" for base station of team magenta)
      @param side    optional the side of the mps ("input" or "output")
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

-- Constants
local conveyor_gripper_forward_x = 0.05 -- distance to move gripper forward after align
local conveyor_gripper_down_z = -0.015    -- distance to move gripper down after driving over product
local conveyor_gripper_down_second_z = -0.005       -- distance to mover gripper down second time
local conveyor_gripper_back_x = -0.07   -- distance to move gripper back after closing gripper
local conveyor_gripper_up_z = 0.05 -- distance to move gripper up after closing gripper

local shelf_gripper_forward_x = 0.048  -- distance to move gripper forward after align to shelf
local shelf_gripper_down_z = -0.01     -- distance to move gripper down after driving over shelf
local shelf_gripper_down_second_z = 0 -- distance to move gripper down second time after driving over shelf
local shelf_gripper_back_x = 0.04   -- distance to move gripper back after closing gripper over shelf
local shelf_gripper_up_z = 0.05  -- distance to move gripper up after closing the gripper over shelf

local drive_back_x = -0.1      -- distance to drive back after closing the gripper

local gripper_pose_offset_x = -0.02  -- conveyor pose offset in x direction
local gripper_pose_offset_y = 0.00     -- conveyor_pose offset in y direction
local gripper_pose_offset_z = 0.02  -- conveyor_pose offset in z direction

local align_target_frame = "gripper_fingers"      -- the gripper align is made relative to this frame (according to gripper_commands)
local movement_target_frame = "gripper" -- the gripper movement is made relative to this frame (according to gripper_commands)




function pose_not_exist()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", align_target_frame)
   if transformed_pos == nil then
     return true
   end
   return false
end


function pose_offset()
  local target_pos = { x = gripper_pose_offset_x,
                        y = gripper_pose_offset_y,
                        z = gripper_pose_offset_z,
                        ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", align_target_frame)
   print_info("product_pick: target_pos is x = %f, y = %f, z = %f", target_pos.x, target_pos.y, target_pos.z)
   print_info("product_pick: transformed_pos is x = %f, y = %f,z = %f", transformed_pos.x, transformed_pos.y, transformed_pos.z)

   return { x = transformed_pos.x,
            y = transformed_pos.y,
            z = transformed_pos.z,
  }
end


fsm:define_states{ export_to=_M,
   closure={arduino_if=arduino_if, pose_not_exist=pose_not_exist},
   {"INIT", JumpState},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands}},final_to="GRIPPER_ALIGN", fail_to="PRE_FAIL"},
   {"GRIPPER_ALIGN", SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER_FORWARD",fail_to="PRE_FAIL"},
   {"MOVE_GRIPPER_FORWARD", SkillJumpState, skills={{gripper_commands}}, final_to="CLOSE_GRIPPER",fail_to="PRE_FAIL"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{ax12gripper}}, final_to="MOVE_GRIPPER_BACK", fail_to="PRE_FAIL"},
   {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands}}, final_to = "DRIVE_BACK", fail_to="FAILED"},
   {"CLOSE_AFTER_CENTER", SkillJumpState, skills={{gripper_commands}}, final_to="HOME_GRIPPER", fail_to="HOME_GRIPPER"},
   {"HOME_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="DRIVE_BACK"},
   {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="CHECK_PUCK", fail_to="FAILED"},
   {"PRE_FAIL", SkillJumpState, skills={{gripper_commands}}, final_to="FAILED", fail_to="FAILED"},
   {"CHECK_PUCK", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="pose_not_exist()"},
   {"INIT", "OPEN_GRIPPER", true, desc="Open gripper for product_pick"},
   {"CHECK_PUCK", "FINAL", cond="gripper_if:gripper_closed()", desc="Hold puck"},
   {"CHECK_PUCK", "FAILED", cond="not gripper_if:gripper_closed()", desc="Don't hold puck!"},
   {"CLOSE_GRIPPER", "MOVE_GRIPPER_BACK", timeout=1},
}


function INIT:init()
  -- Override values if host specific config value is set

  if config:exists("/skills/product_pick/gripper_pose_offset_x") then
      gripper_pose_offset_x = config:get_float("/skills/product_pick/gripper_pose_offset_x")
  end
  if config:exists("/skills/product_pick/gripper_pose_offset_y") then
      gripper_pose_offset_y = config:get_float("/skills/product_pick/gripper_pose_offset_y")
  end
  if config:exists("/skills/product_pick/gripper_pose_offset_z") then
      gripper_pose_offset_z = config:get_float("/skills/product_pick/gripper_pose_offset_z")
  end
  if config:exists("/skills/product_pick/conveyor_gripper_down_z") then
      conveyor_gripper_down_z = config:get_float("/skills/product_pick/conveyor_gripper_down_z")
  end
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands"].command = "OPEN"
end

function CLOSE_GRIPPER:init()
   self.args["gripper_commands"].command= "CLOSE"
end

function GRIPPER_ALIGN:init()
  local pose = pose_offset(self)
  self.args["gripper_commands"] = pose
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame  = movement_target_frame
end

function MOVE_GRIPPER_FORWARD:init()
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = movement_target_frame
  if self.fsm.vars.shelf ~= nil then
    self.args["gripper_commands"].x = shelf_gripper_forward_x
    self.args["gripper_commands"].z = shelf_gripper_down_z
  else
    self.args["gripper_commands"].x = conveyor_gripper_forward_x
    self.args["gripper_commands"].z = conveyor_gripper_down_z
  end

end

function MOVE_GRIPPER_BACK:init()
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = movement_target_frame
  if self.fsm.vars.helf ~= nil then
    self.args["gripper_commands"].x = shelf_gripper_back_x
    self.args["gripper_commands"].z = shelf_gripper_up_z
  else
    self.args["gripper_commands"].x = conveyor_gripper_back_x
    self.args["gripper_commands"].z = conveyor_gripper_up_z
  end
end

function HOME_GRIPPER:init()
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = false
end

function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end

function PRE_FAIL:init()
  self.args["gripper_commands"].command="CLOSE"
end
