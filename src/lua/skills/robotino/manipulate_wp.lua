
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
depends_skills     = {"goto","motor_move","pick_or_put_vs"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"},
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
   {v = "object_tracking_if", type = "ObjectTrackingInterface", id="object-tracking"},
   {v = "tag_0", type = "Position3DInterface", id="/tag-vision/0"},
   {v = "tag_1", type = "Position3DInterface", id="/tag-vision/1"},
   {v = "tag_2", type = "Position3DInterface", id="/tag-vision/2"},
   {v = "tag_3", type = "Position3DInterface", id="/tag-vision/3"},
   {v = "tag_4", type = "Position3DInterface", id="/tag-vision/4"},
   {v = "tag_5", type = "Position3DInterface", id="/tag-vision/5"},
   {v = "tag_6", type = "Position3DInterface", id="/tag-vision/6"},
   {v = "tag_7", type = "Position3DInterface", id="/tag-vision/7"},
   {v = "tag_8", type = "Position3DInterface", id="/tag-vision/8"},
   {v = "tag_9", type = "Position3DInterface", id="/tag-vision/9"},
   {v = "tag_10", type = "Position3DInterface", id="/tag-vision/10"},
   {v = "tag_11", type = "Position3DInterface", id="/tag-vision/11"},
   {v = "tag_12", type = "Position3DInterface", id="/tag-vision/12"},
   {v = "tag_13", type = "Position3DInterface", id="/tag-vision/13"},
   {v = "tag_14", type = "Position3DInterface", id="/tag-vision/14"},
   {v = "tag_15", type = "Position3DInterface", id="/tag-vision/15"},
   {v = "tag_info", type = "TagVisionInterface", id="/tag-vision/info"},
   {v = "arduino", type = "ArduinoInterface", id="Arduino"},
}

documentation      = [==[
Uses visual servoing to fulfill every workpiece manipulation task.

Parameters:
      @param target  the type of the target object: (WORKPIECE | CONVEYOR | SLIDE)
      @param mps     the name of the MPS (e.g. C-CS1, see navgraph)
      @param side    the side of the mps: (INPUT | OUTPUT | SHELF-LEFT | SHELF-MIDDLE | SHELF-RIGHT | SLIDE)
]==]

local LASER_BASE_OFFSET    = 0.35 -- distance between robotino middle point and laser-line
                                  -- used for DRIVE_TO_LASER_LINE
local GRIPPER_TOLERANCE    = {x=0.005, y=0.001, z=0.001} -- accuracy
local MISSING_MAX          = 2 -- limit for missing object detections in a row while fine-tuning gripper
local LINE_MATCH_TOLERANCE = 0.3 -- meter threshold of laserline center to tag
local MIN_VIS_HIST_LINE    = 5 -- minimum visibility history for laser-line before considering it
local MIN_VIS_HIST_TAG     = 5 -- minimum visibility history for tag before considering it

-- Initialize as skill module
skillenv.skill_module(_M)
local llutils = require("fawkes.laser-lines_utils")
local tag_utils = require("tag_utils")
local tfm = require("fawkes.tfutils")

-- Load config
local x_max = 0.115
local y_max = 0.075
local z_max = 0.057

local belt_offset_side = 0.025
local slide_offset_side = -0.225
local left_shelf_offset_side   = -0.075
local middle_shelf_offset_side = -0.175
local right_shelf_offset_side  = -0.275

local drive_back_x = -0.1

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
-- conveyor
if config:exists("plugins/object_tracking/puck_values/belt_offset_side") then
  belt_offset_side = config:get_float("plugins/object_tracking/puck_values/belt_offset_side")
end
-- slide
if config:exists("plugins/object_tracking/puck_values/slide_offset_side") then
  slide_offset_side = config:get_float("plugins/object_tracking/puck_values/slide_offset_side")
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

-- Match tag to navgraph point
function match_line(tag,lines)
   local matched_line = nil

   if tag and tag:visibility_history() >= MIN_VIS_HIST_TAG then
      local tag_laser = tfm.transform6D(
         { x=tag:translation(0), y=tag:translation(1), z=tag:translation(2),
            ori = { x=tag:rotation(0), y=tag:rotation(1), z=tag:rotation(2), w=tag:rotation(3)  }
         }, tag:frame(), "/base_laser"
      )
      local min_dist = LINE_MATCH_TOLERANCE
      for k,line in pairs(lines) do
         local line_center = llutils.center(line, 0)
         local dist = math.vec_length(tag_laser.x - line_center.x, tag_laser.y - line_center.y)
         if line:visibility_history() >= MIN_VIS_HIST_LINE
            and dist < min_dist
         then
            min_dist = dist
            matched_line = line
            printf("Line dist: %f", dist)
         end
      end
   end

   return matched_line
end

function laser_line_found()
  local tag = tag_utils.iface_for_id(fsm.vars.tags, tag_info, fsm.vars.tag_id)
  fsm.vars.matched_line = match_line(tag, fsm.vars.lines)
  return fsm.vars.matched_line ~= nil
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
     and math.abs(math.min(gripper_target.z, z_max) - arduino:z_position()) < GRIPPER_TOLERANCE.z
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

  -- if (fsm.vars.target == "SLIDE" or fsm.vars.target == "CONVEYOR") and
  --     not arduino:is_gripper_closed() then
  --   print_error("If a conveyor or slide is targeted, " ..
  --     "make sure there is a workpiece in the gripper!")
  --   return true
  -- end
  return false
end

function object_tracker_active()
  return object_tracking_if:has_writer() and object_tracking_if:msgid() > 0
end

fsm:define_states{ export_to=_M, closure={MISSING_MAX=MISSING_MAX},
   {"INIT",                  JumpState},
   {"START_TRACKING",        JumpState},
   {"FIND_LASER_LINE",       JumpState},
   {"DRIVE_BACK",            SkillJumpState, skills={{motor_move}},      final_to="SEARCH_LASER_LINE", fail_to="SEARCH_LASER_LINE"},
   {"SEARCH_LASER_LINE",     JumpState},
   {"SPIN",                  SkillJumpState, skills={{motor_move}},      final_to="SEARCH_LASER_LINE", fail_to="SEARCH_LASER_LINE"},
   {"DRIVE_TO_LASER_LINE",   SkillJumpState, skills={{motor_move}},      final_to="AT_LASER_LINE", fail_to="FAILED"},
   {"AT_LASER_LINE",         JumpState},
   {"MOVE_BASE_AND_GRIPPER", SkillJumpState, skills={{motor_move}},      final_to="FINE_TUNE_GRIPPER", fail_to="FIND_LASER_LINE"},
   {"FINE_TUNE_GRIPPER",     JumpState},
   {"GRIPPER_ROUTINE",       SkillJumpState, skills={{pick_or_put_vs}}, final_to="FINAL", fail_to="FINE_TUNE_GRIPPER"},
}

fsm:add_transitions{
   {"INIT", "FAILED",                             cond=input_invalid, desc="Invalid Input"},
   {"INIT", "START_TRACKING",                     cond=true, desc="Valid Input"},
   {"START_TRACKING", "FAILED",                   timeout=2, desc="Object tracker is not starting"},
   {"START_TRACKING", "FIND_LASER_LINE",          cond=object_tracker_active},
   {"FIND_LASER_LINE", "DRIVE_TO_LASER_LINE",     cond=laser_line_found},
   {"FIND_LASER_LINE", "DRIVE_BACK",              timeout=1, desc="Could not find laser-line, drive back"},
   {"SEARCH_LASER_LINE", "DRIVE_TO_LASER_LINE",   cond=laser_line_found},
   {"SEARCH_LASER_LINE", "FAILED",                cond="vars.search_attemps > 10", desc="Tried 10 times, could not find laser-line"},
   {"SEARCH_LASER_LINE", "SPIN",                  timeout=1, desc="Could not find laser-line, spin"},
   {"AT_LASER_LINE", "MOVE_BASE_AND_GRIPPER",     cond="vars.consecutive_detections > 2", desc="Found Object"},
   {"AT_LASER_LINE", "FAILED",                    timeout=2, desc="Object not found"},
   {"FINE_TUNE_GRIPPER", "GRIPPER_ROUTINE",       cond=gripper_aligned, desc="Gripper aligned"},
   {"FINE_TUNE_GRIPPER", "MOVE_BASE_AND_GRIPPER", cond="vars.out_of_reach", desc="Gripper out of reach"},
   {"FINE_TUNE_GRIPPER", "FIND_LASER_LINE",       cond="vars.missing_detections > MISSING_MAX", desc="Tracking lost target"},
}

function INIT:init()
  laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())

  fsm.vars.lines = {}
  fsm.vars.lines[line1:id()] = line1
  fsm.vars.lines[line2:id()] = line2
  fsm.vars.lines[line3:id()] = line3
  fsm.vars.lines[line4:id()] = line4
  fsm.vars.lines[line5:id()] = line5
  fsm.vars.lines[line6:id()] = line6
  fsm.vars.lines[line7:id()] = line7
  fsm.vars.lines[line8:id()] = line8

  fsm.vars.tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7,
  tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }

  if fsm.vars.side == "OUTPUT" then
    fsm.vars.tag_id = navgraph:node(fsm.vars.mps):property_as_float("tag_output")
  else
    fsm.vars.tag_id = navgraph:node(fsm.vars.mps):property_as_float("tag_input")
  end

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
  -- start object tracking
  local msg = object_tracking_if.StartTrackingMessage:new(
    fsm.vars.target_object_type, fsm.vars.expected_mps, fsm.vars.expected_side)
  object_tracking_if:msgq_enqueue_copy(msg)

  -- open gripper
  if fsm.vars.target == "WORKPIECE" then
    local open_msg = arduino.OpenGripperMessage:new()
    arduino:msgq_enqueue(open_msg)
  end

  -- move to default pose
  move_gripper_default_pose()
end

function FIND_LASER_LINE:init()
  -- start searching for laser line
  fsm.vars.search_attemps = 0
end

function DRIVE_BACK:init()
  self.args["motor_move"].x = drive_back_x
end

function SEARCH_LASER_LINE:init()
  fsm.vars.search_attemps = fsm.vars.search_attemps + 1
end

function SPIN:init()
  self.args["motor_move"].ori = math.pi / 5
end

function DRIVE_TO_LASER_LINE:init()
  fsm.vars.consecutive_detections = 0
  fsm.vars.tracking_msgid = 0
  local offset_y = 0
  if fsm.vars.side == "INPUT" then
    offset_y = belt_offset_side
  elseif fsm.vars.side == "OUTPUT" then
    offset_y = -belt_offset_side
  elseif fsm.vars.side == "SLIDE" then
    offset_y = slide_offset_side
  elseif fsm.vars.side == "SHELF-LEFT" then
    offset_y = left_shelf_offset_side
  elseif fsm.vars.side == "SHELF-MIDDLE" then
    offset_y = middle_shelf_offset_side
  elseif fsm.vars.side == "SHELF-RIGHT" then
    offset_y = right_shelf_offset_side
  end

  local center = llutils.center(fsm.vars.matched_line)
  local p = llutils.point_in_front(center, LASER_BASE_OFFSET)
  local laser_target = tfm.transform6D(
        {  x = p.x,
           y = p.y + offset_y,
           z = 0,
           ori = fawkes.tf.create_quaternion_from_yaw(fsm.vars.matched_line:bearing()) },
        fsm.vars.matched_line:frame_id(), "/odom"
        )
  if laser_target then
    self.args["motor_move"] = {x = laser_target.x,
                               y = laser_target.y,
                               frame = "/odom",
                               ori = fawkes.tf.get_yaw(laser_target.ori),
                               end_early = true}
  else
    print_error("Transform Error: matched_line to odom")
  end
end

function DRIVE_TO_LASER_LINE:loop()
  if fsm.vars.tracking_msgid ~= object_tracking_if:msgid() then
    fsm.vars.tracking_msgid = object_tracking_if:msgid()
    if object_tracking_if:is_detected() then
      fsm.vars.consecutive_detections = fsm.vars.consecutive_detections + 1
    else
      fsm.vars.consecutive_detections = 0
    end
  end
end

function AT_LASER_LINE:loop()
  if fsm.vars.tracking_msgid ~= object_tracking_if:msgid() then
    fsm.vars.tracking_msgid = object_tracking_if:msgid()
    if object_tracking_if:is_detected() then
      fsm.vars.consecutive_detections = fsm.vars.consecutive_detections + 1
    else
      fsm.vars.consecutive_detections = 0
    end
  end
end

function MOVE_BASE_AND_GRIPPER:init()
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
  -- count missing detections
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

function GRIPPER_ROUTINE:init()
  -- perform pick or put routine
  if fsm.vars.target == "WORKPIECE" then
    self.args["pick_or_put_vs"].action = "PICK"
  else
    self.args["pick_or_put_vs"].action = "PUT"
  end
end

-- end tracking afterwards

function FINAL:init()
  object_tracking_if:msgq_enqueue(object_tracking_if.StopTrackingMessage:new())
end

function FAILED:init()
  move_gripper_default_pose()
  object_tracking_if:msgq_enqueue(object_tracking_if.StopTrackingMessage:new())
end
