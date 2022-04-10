
----------------------------------------------------------------------------
--  manipulate_wp.lua
--
--  Created: Wed Nov 17
--  Copyright  2021  Matteo Tschesche
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
name               = "manipulate_wp"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"goto","motor_move","gripper_routine"}
depends_interfaces = {
   {v = "object_tracking_if", type = "ObjectTrackingInterface", id="object-tracking"},
   {v = "arduino", type = "ArduinoInterface", id="Arduino"},
}

documentation      = [==[
Uses visual servoing to fulfill every workpiece manipulation task.

Parameters:
      @param target  the type of the target object: (WORKPIECE | CONVEYOR | SLIDE)
      @param mps     the name of the MPS (e.g. C-CS1, see navgraph)
      @param side    the side of the mps: (INPUT | OUTPUT | SHELF-LEFT | SHELF-MIDDLE | SHELF-RIGHT | SLIDE)
]==]

local EXPECTED_BASE_OFFSET       = 0.5 -- distance between robotino middle point and workpiece
                                       -- used as initial target position while searching
local GRIPPER_TOLERANCE          = {x=0.005, y=0.001, z=0.001} -- accuracy
local MISSING_MAX                = 2 -- limit for missing object detections in a row while fine-tuning gripper

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")

-- Load config
local for_gazebo = false
local x_max = 0.115
local y_max = 0.075
local z_max = 0.057

local puck_size   = 0.02
local puck_height = 0.0225

local belt_height      = 0.92
local belt_lenght      = 0.35
local belt_offset_side = 0.025

local slide_offset_side = -0.225
local slide_height      = 0.92

local left_shelf_offset_side   = -0.075
local middle_shelf_offset_side = -0.175
local right_shelf_offset_side  = -0.275

-- check if gazebo is used
if config:exists("plugins/object_tracking/for_gazebo") then
  for_gazebo = config:get_bool("plugins/object_tracking/for_gazebo")
end

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

-- read config values for computing expected target position
-- puck
if config:exists("plugins/object_tracking/puck_values/puck_size") then
  puck_size = config:get_float("plugins/object_tracking/puck_values/puck_size")
end
if config:exists("plugins/object_tracking/puck_values/puck_height") then
  puck_height = config:get_float("plugins/object_tracking/puck_values/puck_height")
end
-- belt
if config:exists("plugins/object_tracking/puck_values/belt_height") then
  belt_height = config:get_float("plugins/object_tracking/puck_values/belt_height")
end
if config:exists("plugins/object_tracking/puck_values/belt_lenght") then
  belt_lenght = config:get_float("plugins/object_tracking/puck_values/belt_lenght")
end
if config:exists("plugins/object_tracking/puck_values/belt_offset_side") then
  belt_offset_side = config:get_float("plugins/object_tracking/puck_values/belt_offset_side")
end
-- slide
if config:exists("plugins/object_tracking/puck_values/slide_offset_side") then
  slide_offset_side = config:get_float("plugins/object_tracking/puck_values/slide_offset_side")
end
if config:exists("plugins/object_tracking/puck_values/slide_height") then
  slide_height = config:get_float("plugins/object_tracking/puck_values/slide_height")
end
-- shelf
if config:exists("plugins/object_tracking/puck_values/left_shelf_offset_side") then
  left_shelf_offset_side = config:get_float("plugins/object_tracking/puck_values/left_shelf_offset_side")
end
if config:exists("plugins/object_tracking/puck_values/middle_shelf_offset_side") then
  middle_shelf_offset_side = config:get_float("plugins/object_tracking/puck_values/middle_shelf_offset_side")
end
if config:exists("plugins/object_tracking/puck_values/right_shelf_offset_side") then
  right_shelf_offset_side = config:get_float("plugins/object_tracking/puck_values/right_shelf_offset_side")
end

function gripper_aligned()
  if fsm.vars.gripper_wait < 1 then
    return false
  end

  local gripper_target = tfm.transform6D(
    {x=object_tracking_if:gripper_frame(0),
     y=object_tracking_if:gripper_frame(1),
     z=object_tracking_if:gripper_frame(2),
     ori=fawkes.tf.create_quaternion_from_yaw(0)},
    "base_link", "end_effector_home")

  return math.abs(gripper_target.x - arduino:x_position()) < GRIPPER_TOLERANCE.x
     and math.abs(gripper_target.y - (arduino:y_position() - y_max/2)) < GRIPPER_TOLERANCE.y
     and math.abs(gripper_target.z - arduino:z_position()) < GRIPPER_TOLERANCE.z
end

function set_gripper(x, y, z)
  if not arduino:is_final() then
    fsm.vars.gripper_wait = 0
  elseif fsm.vars.gripper_wait < 10 then
     fsm.vars.gripper_wait = fsm.vars.gripper_wait + 1
  end

  -- Clip to axis limits
  x_clipped = math.max(0, math.min(x, x_max))
  y_clipped = math.max(-y_max/2, math.min(y, y_max/2))
  z_clipped = math.max(0, math.min(z, z_max))

  if x_clipped ~= x then
    fsm.vars.out_of_reach = true
    print("Gripper cannot reache x-value: " .. x .. " ! Clipped to " .. x_clipped)
    return
  end
  if y_clipped ~= y then
    fsm.vars.out_of_reach = true
    print("Gripper cannot reache y-value: " .. y .. " ! Clipped to " .. y_clipped)
    return
  end
  if z_clipped ~= z then
    print("Gripper cannot reache z-value: " .. z .. " ! Clipped to " .. z_clipped)
  end

  if (not arduino:is_final() and (
     math.abs(fsm.vars.gripper_target_pos_x - x_clipped) > GRIPPER_TOLERANCE.x * 2 or
     math.abs(fsm.vars.gripper_target_pos_y - y_clipped) > GRIPPER_TOLERANCE.y * 1 or
     math.abs(fsm.vars.gripper_target_pos_z - z_clipped) > GRIPPER_TOLERANCE.z * 1.5)) or
     (arduino:is_final() and (
     math.abs(fsm.vars.gripper_target_pos_x - x_clipped) > GRIPPER_TOLERANCE.x or
     math.abs(fsm.vars.gripper_target_pos_y - y_clipped) > GRIPPER_TOLERANCE.y or
     math.abs(fsm.vars.gripper_target_pos_z - z_clipped) > GRIPPER_TOLERANCE.z)) then
    fsm.vars.gripper_wait = 0
    fsm.vars.gripper_target_pos_x = x_clipped
    fsm.vars.gripper_target_pos_y = y_clipped
    fsm.vars.gripper_target_pos_z = z_clipped
  else
    return
  end

  move_abs_message:set_x(fsm.vars.gripper_target_pos_x)
  move_abs_message:set_y(fsm.vars.gripper_target_pos_y)
  move_abs_message:set_z(fsm.vars.gripper_target_pos_z)
  move_abs_message:set_target_frame("gripper_home")
  arduino:msgq_enqueue_copy(move_abs_message)
end

function move_gripper_default_pose()
  move_abs_message = arduino.MoveXYZAbsMessage:new()
  move_abs_message:set_x(0)
  move_abs_message:set_y(0)
  move_abs_message:set_z(z_max)
  move_abs_message:set_target_frame("gripper_home")
  arduino:msgq_enqueue_copy(move_abs_message)
end

function compute_expected_pos_x(x_offset)
  if for_gazebo then
    return fsm.vars.mps_x + x_offset * math.cos(fsm.vars.mps_ori) -
           (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.sin(fsm.vars.mps_ori)
  else
    return fsm.vars.mps_x + x_offset * math.sin(fsm.vars.mps_ori) +
           (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.cos(fsm.vars.mps_ori)
  end
end

function compute_expected_pos_y(y_offset)
  if for_gazebo then
    return fsm.vars.mps_y + y_offset * math.sin(fsm.vars.mps_ori) +
           (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.cos(fsm.vars.mps_ori)
  else
    return fsm.vars.mps_y - y_offset * math.cos(fsm.vars.mps_ori) +
           (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.sin(fsm.vars.mps_ori)
  end
end

-- compute the expected target object position
function get_pos_for_side(side)
  if side == "INPUT" then
    return {x = compute_expected_pos_x(0),
            y = compute_expected_pos_y(0)}
  elseif side == "OUTPUT" then
    if for_gazebo then
      return {x = fsm.vars.mps_x + belt_offset_side * math.cos(fsm.vars.mps_ori) +
                  (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.sin(fsm.vars.mps_ori),
              y = fsm.vars.mps_y + belt_offset_side * math.sin(fsm.vars.mps_ori) -
                  (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.cos(fsm.vars.mps_ori)}
    else
      return {x = fsm.vars.mps_x -
                  (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.cos(fsm.vars.mps_ori),
              y = fsm.vars.mps_y -
                  (belt_lenght/2 - puck_size/2 + EXPECTED_BASE_OFFSET) * math.sin(fsm.vars.mps_ori)}
    end
  elseif side == "SLIDE" then
    return {x = compute_expected_pos_x(slide_offset_side),
            y = compute_expected_pos_y(slide_offset_side)}
  elseif side == "SHELF-LEFT" then
    return {x = compute_expected_pos_x(left_shelf_offset_side),
            y = compute_expected_pos_y(left_shelf_offset_side)}
  elseif side == "SHELF-MIDDLE" then
    return {x = compute_expected_pos_x(middle_shelf_offset_side),
            y = compute_expected_pos_y(middle_shelf_offset_side)}
  elseif side == "SHELF-RIGHT" then
    return {x = compute_expected_pos_x(right_shelf_offset_side),
            y = compute_expected_pos_y(right_shelf_offset_side)}
  else
    print_error("Error:" .. fsm.vars.side .. "is not a valid side!")
    return nil
  end
end

function input_invalid()
  if fsm.vars.target_object_type == nil then
    print_error("That is not a valid target!")
    return true
  elseif fsm.vars.expected_mps == nil then
    print_error("That is not a valid mps!")
    return true
  elseif fsm.vars.expected_side == nil then
    print_error("That is not a valid side!")
    return true
  end

  -- sanity check
  if fsm.vars.target == "WORKPIECE" then
    if fsm.vars.side ~= "INPUT" and fsm.vars.side ~= "OUTPUT" then
      if not string.find(fsm.vars.mps, "CS") or
         not string.find(fsm.vars.side, "SHELF-") then
        print_error("There cannot be a workpiece at this mps-side!")
        return true
      end
    end
  elseif fsm.vars.target == "SLIDE" then
    if not string.find(fsm.vars.mps, "RS") or fsm.vars.side ~= "SLIDE" then
      print_error("If a slide is targeted, " ..
        "choose a ringstation as mps and SLIDE as side!")
      return true
    end
  else -- side == "CONVEYOR"
    if fsm.vars.side ~= "INPUT" and fsm.vars.side ~= "OUTPUT" then
      print_error("If a conveyor is targeted, choose INPUT or OUTPUT as side!")
      return true
    end
  end

  if (fsm.vars.target == "SLIDE" or fsm.vars.target == "CONVEYOR") and
      not arduino:is_gripper_closed() then
    print_error("If a conveyor or slide is targeted, " ..
      "make sure there is a workpiece in the gripper!")
    return true
  end
  return false
end

function object_tracker_active()
  return object_tracking_if:has_writer() and object_tracking_if:msgid() > 0
end

fsm:define_states{ export_to=_M, closure={MISSING_MAX=MISSING_MAX},
   {"INIT",                  JumpState},
   {"START_TRACKING",        JumpState},
   {"SEARCH",                SkillJumpState, skills={{goto}},            final_to="MOVE_BASE_AND_GRIPPER", fail_to="FAILED"},
   {"MOVE_BASE_AND_GRIPPER", SkillJumpState, skills={{motor_move}},      final_to="FINE_TUNE_GRIPPER",     fail_to="SEARCH"},
   {"FINE_TUNE_GRIPPER",     JumpState},
   {"GRIPPER_ROUTINE",       SkillJumpState, skills={{gripper_routine}}, final_to="FINAL", fail_to="FINE_TUNE_GRIPPER"},
}

fsm:add_transitions{
   {"INIT", "FAILED",                             cond=input_invalid, desc="Invalid Input"},
   {"INIT", "START_TRACKING",                     cond=true, desc="Valid Input"},
   {"START_TRACKING", "FAILED",                   timeout=60, desc="Object tracker is not starting"},
   {"START_TRACKING", "SEARCH",                   cond=object_tracker_active},
   {"FINE_TUNE_GRIPPER", "GRIPPER_ROUTINE",       cond=gripper_aligned, desc="Gripper aligned"},
   {"FINE_TUNE_GRIPPER", "MOVE_BASE_AND_GRIPPER", cond="vars.out_of_reach", desc="Gripper out of reach"},
   {"FINE_TUNE_GRIPPER", "SEARCH",                cond="vars.missing_detections > MISSING_MAX", desc="Tracking lost target"},
}

function INIT:init()
  fsm.vars.missing_detections = 0
  fsm.vars.msgid              = 0
  fsm.vars.out_of_reach       = false

  local TARGET_NAMES = {["WORKPIECE"] = object_tracking_if.WORKPIECE,
                        ["CONVEYOR"]  = object_tracking_if.CONVEYOR_BELT_FRONT,
                        ["SLIDE"]     = object_tracking_if.SLIDE_FRONT}
  local MPS_NAMES = {["M-BS"]  = object_tracking_if.M_BS,
                     ["M-RS1"] = object_tracking_if.M_RS1,
                     ["M-RS2"] = object_tracking_if.M_RS2,
                     ["M-CS1"] = object_tracking_if.M_CS1,
                     ["M-CS2"] = object_tracking_if.M_CS2,
                     ["M-DS"]  = object_tracking_if.M_DS,
                     ["M-SS"]  = object_tracking_if.M_SS,
                     ["C-BS"]  = object_tracking_if.C_BS,
                     ["C-RS1"] = object_tracking_if.C_RS1,
                     ["C-RS2"] = object_tracking_if.C_RS2,
                     ["C-CS1"] = object_tracking_if.C_CS1,
                     ["C-CS2"] = object_tracking_if.C_CS2,
                     ["C-DS"]  = object_tracking_if.C_DS,
                     ["C-SS"]  = object_tracking_if.C_SS}
  local SIDE_NAMES = {["INPUT"]        = object_tracking_if.INPUT_CONVEYOR,
                      ["OUTPUT"]       = object_tracking_if.OUTPUT_CONVEYOR,
                      ["SHELF-LEFT"]   = object_tracking_if.SHELF_LEFT,
                      ["SHELF-MIDDLE"] = object_tracking_if.SHELF_MIDDLE,
                      ["SHELF-RIGHT"]  = object_tracking_if.SHELF_RIGHT,
                      ["SLIDE"]        = object_tracking_if.SLIDE}

  -- get ENUM of input variables
  fsm.vars.target_object_type = TARGET_NAMES[fsm.vars.target]
  fsm.vars.expected_mps = MPS_NAMES[fsm.vars.mps]
  fsm.vars.expected_side = SIDE_NAMES[fsm.vars.side]

  fsm.vars.gripper_target_pos_x = 0
  fsm.vars.gripper_target_pos_y = 0
  fsm.vars.gripper_target_pos_z = 0
  fsm.vars.gripper_wait       = 0
end

function START_TRACKING:init()
  local msg = object_tracking_if.StartTrackingMessage:new(
    fsm.vars.target_object_type, fsm.vars.expected_mps, fsm.vars.expected_side)
  object_tracking_if:msgq_enqueue_copy(msg)

  -- compute expected target location
  local node = navgraph:node(fsm.vars.mps)
  fsm.vars.mps_x = node:x()
  fsm.vars.mps_y = node:y()
  fsm.vars.mps_ori = node:property_as_float("orientation")

  local expected_pos = get_pos_for_side(fsm.vars.side)
  fsm.vars.expected_pos_x = expected_pos.x
  fsm.vars.expected_pos_y = expected_pos.y

  if fsm.vars.side == "OUTPUT" then
    if for_gazebo then
      fsm.vars.expected_pos_ori = fsm.vars.mps_ori + 1.57
    else
      fsm.vars.expected_pos_ori = fsm.vars.mps_ori
    end
  else
    if for_gazebo then
      fsm.vars.expected_pos_ori = fsm.vars.mps_ori + 1.57 + math.pi
    else
      fsm.vars.expected_pos_ori = fsm.vars.mps_ori + math.pi
    end
  end

  -- meanwhile open gripper
  if fsm.vars.target == "WORKPIECE" then
    local open_msg = arduino.OpenGripperMessage:new()
    arduino:msgq_enqueue(open_msg)
  end
end

function SEARCH:init()
  fsm.vars.time_start = fawkes.Time:new():in_msec()
  move_gripper_default_pose()
  self.args["goto"] = {x = fsm.vars.expected_pos_x,
                       y = fsm.vars.expected_pos_y,
                       ori = fsm.vars.expected_pos_ori,
                       end_early = true}
end

function SEARCH:exit()
  local now = fawkes.Time:new():in_msec()
  print_info("[VS] Positioning took " .. now - fsm.vars.time_start .. " milliseconds")
end

function MOVE_BASE_AND_GRIPPER:init()
  fsm.vars.time_start = fawkes.Time:new():in_msec()
  -- move base to target pose using visual servoing
  self.args["motor_move"] = {x = object_tracking_if:base_frame(0),
                             y = object_tracking_if:base_frame(1),
                             ori = object_tracking_if:base_frame(5),
                             frame = "base_link",
                             visual_servoing = true}

  -- and move gripper to relative target gripper pose
  local base_x = object_tracking_if:base_frame(0)
  local base_y = object_tracking_if:base_frame(1)
  local gripper_x = object_tracking_if:gripper_frame(0)
  local gripper_y = object_tracking_if:gripper_frame(1)
  local gripper_z = object_tracking_if:gripper_frame(2)

  local diff_x = (gripper_x - base_x) * (gripper_x - base_x)
  local diff_y = (gripper_y - base_y) * (gripper_y - base_y)
  local forward_distance = math.sqrt(diff_x + diff_y)

  local gripper_target = tfm.transform6D(
    {x=forward_distance,
     y=0,
     z=gripper_z,
     ori=fawkes.tf.create_quaternion_from_yaw(0)},
    "base_link", "end_effector_home")

  fsm.vars.gripper_wait = 10
  set_gripper(gripper_target.x, 0, gripper_target.z)
end

function FINE_TUNE_GRIPPER:init()
  fsm.vars.missing_detections = 0
  fsm.vars.out_of_reach       = false
  fsm.vars.gripper_wait       = 10
end

function FINE_TUNE_GRIPPER:loop()
  if fsm.vars.msgid ~= object_tracking_if:msgid() then
    fsm.vars.msgid = object_tracking_if:msgid()
    if object_tracking_if:is_detected() then
      fsm.vars.missing_detections = 0
    else
      fsm.vars.missing_detections = fsm.vars.missing_detections + 1
    end
  end

  -- align gripper with target using visual servoing
  local gripper_target = tfm.transform6D(
    {x=object_tracking_if:gripper_frame(0),
     y=object_tracking_if:gripper_frame(1),
     z=object_tracking_if:gripper_frame(2),
     ori=fawkes.tf.create_quaternion_from_yaw(0)},
    "base_link", "end_effector_home")

  set_gripper(gripper_target.x,
              gripper_target.y,
              gripper_target.z)
end

function FINE_TUNE_GRIPPER:exit()
  local now = fawkes.Time:new():in_msec()
  print_info("[VS] Alignment took " .. now - fsm.vars.time_start .. " milliseconds")
end

function GRIPPER_ROUTINE:init()
  fsm.vars.time_start = fawkes.Time:new():in_msec()
  -- perform pick or put routine
  if fsm.vars.target == "WORKPIECE" then
    self.args["gripper_routine"].pick_wp = true
  else
    self.args["gripper_routine"].pick_wp = false
  end
end

function GRIPPER_ROUTINE:exit()
  local now = fawkes.Time:new():in_msec()
  print_info("[VS] Gripper routine took " .. now - fsm.vars.time_start .. " milliseconds")
end

-- end tracking afterwards

function FINAL:init()
  local msg = object_tracking_if.StopTrackingMessage:new()
  object_tracking_if:msgq_enqueue_copy(msg)
end

function FAILED:init()
  move_gripper_default_pose()
  local msg = object_tracking_if.StopTrackingMessage:new()
  object_tracking_if:msgq_enqueue_copy(msg)
end
