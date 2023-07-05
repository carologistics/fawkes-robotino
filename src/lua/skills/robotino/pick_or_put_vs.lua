----------------------------------------------------------------------------
--  pick_or_put_vs.lua
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
name               = "pick_or_put_vs"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands", "motor_move"}
depends_interfaces = {
   {v = "object_tracking_if", type = "ObjectTrackingInterface", id="object-tracking"},
   {v = "arduino", type = "ArduinoInterface", id="Arduino"},
}

documentation      = [==[
Skill to pick a product and to put it down based on param action.
It is independent of the workpiece location or its target location.

Parameters:
      @param target             target object (WORKPIECE | CONVEYOR | SLIDE)
      @param missing_c3_height  distance between C3 height and current wp height
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

local offset_x_pick_target_frame = 0.00
local offset_x_put_conveyor_target_frame = 0.01
local offset_x_put_slide_target_frame = -0.03

local offset_z_pick_target_frame = 0.035
local offset_z_put_conveyor_target_frame = 0.045
local offset_z_put_slide_target_frame = 0.025

local offset_x_pick_routine = 0.0
local offset_x_put_conveyor_routine = 0.0
local offset_x_put_slide_routine = -0.03

local offset_z_pick_routine = 0.0
local offset_z_put_conveyor_routine = 0.0
local offset_z_put_slide_routine = 0.0

local offset_z_up_pick = 0.0125
local offset_z_up_put_conveyor = 0.045
local offset_z_up_put_slide = 0.015

local drive_back_x = -0.1

local gripper_default_pose_x = 0.00   -- conveyor pose offset in x direction
local gripper_default_pose_y = 0.00   -- conveyor_pose offset in y direction
local gripper_default_pose_z = 0.057  -- conveyor_pose offset in z direction

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
if config:exists("/arduino/z_max") then
  z_max = config:get_float("/arduino/z_max")
end

-- read vs configs
if config:exists("plugins/vs_offsets/workpiece/pick_target/offset_x") then
  offset_x_pick_target_frame = config:get_float("plugins/vs_offsets/workpiece/pick_target/offset_x")
end
if config:exists("plugins/vs_offsets/conveyor/put_target/offset_x") then
  offset_x_put_conveyor_target_frame = config:get_float("plugins/vs_offsets/conveyor/put_target/offset_x")
end
if config:exists("plugins/vs_offsets/slide/put_target/offset_x") then
  offset_x_put_slide_target_frame = config:get_float("plugins/vs_offsets/slide/put_target/offset_x")
end

if config:exists("plugins/vs_offsets/workpiece/pick_target/offset_z") then
  offset_z_pick_target_frame = config:get_float("plugins/vs_offsets/workpiece/pick_target/offset_z")
end
if config:exists("plugins/vs_offsets/conveyor/put_target/offset_z") then
  offset_z_put_conveyor_target_frame = config:get_float("plugins/vs_offsets/conveyor/put_target/offset_z")
end
if config:exists("plugins/vs_offsets/slide/put_target/offset_z") then
  offset_z_put_slide_target_frame = config:get_float("plugins/vs_offsets/slide/put_target/offset_z")
end

if config:exists("plugins/vs_offsets/conveyor/pick_routine/offset_x") then
  offset_x_put_conveyor_routine = config:get_float("plugins/vs_offsets/conveyor/pick_routine/offset_x")
end
if config:exists("plugins/vs_offsets/slide/pick_routine/offset_x") then
  offset_x_put_slide_routine = config:get_float("plugins/vs_offsets/slide/pick_routine/offset_x")
end

if config:exists("plugins/vs_offsets/workpiece/pick_routine/offset_z") then
  offset_z_pick_routine = config:get_float("plugins/vs_offsets/workpiece/pick_routine/offset_z")
end
if config:exists("plugins/vs_offsets/conveyor/pick_routine/offset_z") then
  offset_z_put_conveyor_routine = config:get_float("plugins/vs_offsets/conveyor/pick_routine/offset_z")
end
if config:exists("plugins/vs_offsets/slide/pick_routine/offset_z") then
  offset_z_put_slide_routine = config:get_float("plugins/vs_offsets/slide/pick_routine/offset_z")
end

if config:exists("plugins/vs_offsets/workpiece/pick_end/offset_z") then
  offset_z_up_pick = config:get_float("plugins/vs_offsets/workpiece/pick_end/offset_z")
end
if config:exists("plugins/vs_offsets/conveyor/put_end/offset_z") then
  offset_z_up_put_conveyor = config:get_float("plugins/vs_offsets/conveyor/put_end/offset_z")
end
if config:exists("plugins/vs_offsets/slide/put_end/offset_z") then
  offset_z_up_put_slide = config:get_float("plugins/vs_offsets/slide/put_end/offset_z")
end

function input_invalid()
  if fsm.vars.target == "WORKPIECE" or fsm.vars.target == "CONVEYOR"  or fsm.vars.target == "SLIDE" then
    return false
  else
    return true
  end
end

function is_pick_action()
  if fsm.vars.target == "WORKPIECE" then
    return true
  else
    return false
  end
end

function is_put_action()
  return not is_pick_action()
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
   {"INIT", "FAILED",                 cond=input_invalid, desc="Invalid Input"},
   {"INIT", "MOVE_GRIPPER_DOWN",      true, desc="Start Routine"},
   {"CHOOSE_ACTION", "CLOSE_GRIPPER", cond=is_pick_action, desc="Picking Up Workpiece"},
   {"CHOOSE_ACTION", "OPEN_GRIPPER",  cond=is_put_action, desc="Putting Down Workpiece"},
   {"CHOOSE_ACTION", "FAILED",        true, desc="Instructions Unclear"},
   {"DECIDE_CLOSE", "CLOSE_DEFAULT",  cond=is_put_action, desc="Close Gripper"},
   {"DECIDE_CLOSE", "FINAL",          true},
}

function INIT:init()
  print(type(fsm.vars.missing_c3_height))
  fsm.vars.missing_c3_height = tonumber(fsm.vars.missing_c3_height)
end

function MOVE_GRIPPER_DOWN:init()
  local gripper_target = tfm.transform6D(
    {x=object_tracking_if:gripper_frame(0),
     y=object_tracking_if:gripper_frame(1),
     z=object_tracking_if:gripper_frame(2),
     ori=fawkes.tf.create_quaternion_from_yaw(0)},
    "base_link", "end_effector_home")

  -- Clip to axis limits
  local x_given = gripper_target.x
  print("target:")
  print(fsm.vars.target)
  if fsm.vars.target == "WORKPIECE" then
    print("WP")
    x_given = gripper_target.x - offset_x_pick_target_frame + offset_x_pick_routine
  elseif fsm.vars.target == "CONVEYOR" then
    print("CONVEYOR")
    x_given = gripper_target.x - offset_x_put_conveyor_target_frame + offset_x_put_conveyor_routine
  else -- SLIDE
    print("SLIDE")
    x_given = gripper_target.x - offset_x_put_slide_target_frame + offset_x_put_slide_routine
  end

  local x_clipped = math.max(0, math.min(x_given, x_max))
  local y_clipped = math.max(-y_max/2, math.min(gripper_target.y, y_max/2))

  local z_given = 0
  if fsm.vars.target == "WORKPIECE" then
    z_given = gripper_target.z - offset_z_pick_target_frame + offset_z_pick_routine
  elseif fsm.vars.target == "CONVEYOR" then
    z_given = gripper_target.z - offset_z_put_conveyor_target_frame + offset_z_put_conveyor_routine
  else -- SLIDE
    z_given = gripper_target.z - offset_z_put_slide_target_frame + offset_z_put_slide_routine
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
  if fsm.vars.target == "WORKPIECE" then
    z_given = fsm.vars.target_z + offset_z_up_pick
  elseif fsm.vars.target == "CONVEYOR" then
    z_given = fsm.vars.target_z + offset_z_up_put_conveyor - fsm.vars.missing_c3_height
  else -- SLIDE
    z_given = fsm.vars.target_z + offset_z_up_put_slide
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
