----------------------------------------------------------------------------
--  gripper_routine.lua
--
--  Created Tue Jan 4
--  Copyright  2022  Matteo Tschesche
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
name               = "gripper_routine"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands", "motor_move"}
depends_interfaces = {
   {v = "object_tracking_if", type = "ObjectTrackingInterface", id="object-tracking"},
   {v = "arduino", type = "ArduinoInterface", id="Arduino"},
}

documentation      = [==[
Skill to pick a product and to put it down based on param pick_wp.
It is independent of the workpiece location or its target location.

Parameters:
      @param pick_wp   (type bool) decides if a pick or put action is performed
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

-- Constant
local gripper_down_z_pick = -0.05  -- distance to move gripper down after driving over product TODO: get through object tracking yaml
local gripper_down_z_put = -0.028  -- distance to move gripper down after driving over product

local gripper_up_z_pick = 0.01   -- distance to move gripper up after closing gripper
local gripper_up_z_put = 0.045   -- distance to move gripper up after opening gripper

local drive_back_x = -0.3

local gripper_default_pose_x = 0.00   -- conveyor pose offset in x direction
local gripper_default_pose_y = 0.00   -- conveyor_pose offset in y direction
local gripper_default_pose_z = 0.056  -- conveyor_pose offset in z direction

local x_max = 0.115  -- gripper max value in x direction
local y_max = 0.075  -- gripper max value in y direction
local z_max = 0.057  -- gripper max value in z direction

-- read gripper config
if config:exists("/arduino/x_max") then
  x_max = config:get_float("/arduino/x_max")
end
if config:exists("/arduino/y_max") then
  y_max = config:get_float("/arduino/y_max")
end
if config:exists("/arduino/max_z") then
  z_max = config:get_float("/arduino/z_max")
end

fsm:define_states{ export_to=_M, closure={},
   {"INIT",              JumpState},
   {"MOVE_GRIPPER_DOWN", SkillJumpState, skills={{gripper_commands}}, final_to="CHOOSE_ACTION" ,fail_to="FAILED"},
   {"CHOOSE_ACTION",     JumpState},
   {"CLOSE_GRIPPER",     SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER_UP", fail_to="FAILED"},
   {"OPEN_GRIPPER",      SkillJumpState, skills={{gripper_commands}}, final_to="MOVE_GRIPPER_UP", fail_to="FAILED"},
   {"MOVE_GRIPPER_UP",   SkillJumpState, skills={{gripper_commands}}, final_to="GRIPPER_DEFAULT", fail_to="FAILED"},
   {"GRIPPER_DEFAULT",   SkillJumpState, skills={{gripper_commands}}, final_to="DRIVE_BACK", fail_to="FAILED"},
   {"DRIVE_BACK",        SkillJumpState, skills={{motor_move}}, final_to="DECIDE_CLOSE", fail_to="FAILED"},
   {"DECIDE_CLOSE",      JumpState},
   {"CLOSE_DEFAULT",     SkillJumpState, skills={{gripper_commands}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "MOVE_GRIPPER_DOWN",      true, desc="Start Routine"},
   {"CHOOSE_ACTION", "CLOSE_GRIPPER", cond="vars.pick_wp", desc="Picking Up Workpiece"},
   {"CHOOSE_ACTION", "OPEN_GRIPPER",  cond="not vars.pick_wp", desc="Putting Down Workpiece"},
   {"CHOOSE_ACTION", "FAILED",        true, desc="Instructions Unclear"},
   {"DECIDE_CLOSE", "CLOSE_DEFAULT",  cond="not vars.pick_wp", desc="Close Gripper"},
   {"DECIDE_CLOSE", "FINAL",          true},
}

function MOVE_GRIPPER_DOWN:init()
  local gripper_target = tfm.transform6D(
    {x=object_tracking_if:gripper_frame(0),
     y=object_tracking_if:gripper_frame(1),
     z=object_tracking_if:gripper_frame(2),
     ori=fawkes.tf.create_quaternion_from_yaw(0)},
    "base_link", "end_effector_home")

  -- Clip to axis limits
  local x_clipped = math.max(0, math.min(gripper_target.x, x_max))
  local y_clipped = math.max(-y_max/2, math.min(gripper_target.y, y_max/2))

  local z_given = 0
  if self.fsm.vars.pick_wp then
    z_given = gripper_target.z + gripper_down_z_pick
  else
    z_given = gripper_target.z + gripper_down_z_put
  end
  local z_clipped = math.max(0, math.min(z_given, z_max))

  self.args["gripper_commands"].x = x_clipped
  self.args["gripper_commands"].y = y_clipped
  self.args["gripper_commands"].z = z_clipped 
  self.args["gripper_commands"].command = "MOVEABS"

  fsm.vars.target_z = z_clipped
end

function CLOSE_GRIPPER:init()
  self.args["gripper_commands"].command= "CLOSE"
end

function OPEN_GRIPPER:init()
  self.args["gripper_commands"].command = "OPEN"
end

function MOVE_GRIPPER_UP:init()
  local z_given = 0
  if self.fsm.vars.pick_wp then
    z_given = fsm.vars.target_z + gripper_up_z_pick
  else
    z_given = fsm.vars.target_z + gripper_up_z_put
  end

  self.args["gripper_commands"].x = arduino:x_position()
  self.args["gripper_commands"].y = arduino:y_position() - y_max/2
  self.args["gripper_commands"].z = math.max(0, math.min(z_given, z_max))
  self.args["gripper_commands"].command = "MOVEABS"
end

function GRIPPER_DEFAULT:init()
  self.args["gripper_commands"].x = gripper_default_pose_x
  self.args["gripper_commands"].y = gripper_default_pose_y
  self.args["gripper_commands"].z = gripper_default_pose_z
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = false
end

function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end

function CLOSE_DEFAULT:init()
  self.args["gripper_commands"].command= "CLOSE"
end
