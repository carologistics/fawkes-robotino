----------------------------------------------------------------------------
--  manipulate_wp.lua
----  Created: Wed Nov 17
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
name = "manipulate_wp"
fsm = SkillHSM:new{name = name, start = "INIT", debug = true}
depends_skills = {"moveto", "motor_move", "pick_or_put_vs"}
depends_interfaces = {
    {v = "line1", type = "LaserLineInterface", id = "/laser-lines/1"},
    {v = "line2", type = "LaserLineInterface", id = "/laser-lines/2"},
    {v = "line3", type = "LaserLineInterface", id = "/laser-lines/3"},
    {v = "line4", type = "LaserLineInterface", id = "/laser-lines/4"},
    {v = "line5", type = "LaserLineInterface", id = "/laser-lines/5"},
    {v = "line6", type = "LaserLineInterface", id = "/laser-lines/6"},
    {v = "line7", type = "LaserLineInterface", id = "/laser-lines/7"},
    {v = "line8", type = "LaserLineInterface", id = "/laser-lines/8"},
    {v = "laserline_switch", type = "SwitchInterface", id = "laser-lines"}, {
        v = "object_tracking_if",
        type = "ObjectTrackingInterface",
        id = "object-tracking"
    }, {v = "arduino", type = "ArduinoInterface", id = "Arduino"}
}

documentation = [==[
Uses visual servoing to fulfill every workpiece manipulation task.

Parameters:
      @param target  the type of the target object: (WORKPIECE | CONVEYOR | SLIDE)
      @param mps     the name of the MPS (e.g. C-CS1, see navgraph)
      @param side    the side of the mps: (INPUT | OUTPUT | SHELF-LEFT | SHELF-MIDDLE | SHELF-RIGHT | SLIDE)
      @param dry_run true, if the gripper should not interact with the workpiece and only need to check if
                     the workpiece is there (optional, bool)
      @param query   defines if dry_run expects a workpiece to be at the location or wet (optional, THERE | ABSENT)
                     THERE by default
      @param map_pos true, if MPS Pos is compared to Map Pos(optional,bool) True by default
]==]

local LASER_BASE_OFFSET = 0.5 -- distance between robotino middle point and laser-line
-- used for DRIVE_TO_LASER_LINE
local GRIPPER_TOLERANCE = {x = 0.005, y = 0.001, z = 0.001} -- accuracy
local MISSING_MAX = 5 -- limit for missing object detections in a row while fine-tuning gripper
local MIN_VIS_HIST_LINE = 5 -- minimum visibility history for laser-line before considering it
local MIN_ACTUAL_DIST = 1.8 -- minimum distance b/w bot and laser center

-- Initialize as skill module

skillenv.skill_module(_M)
local llutils = require("fawkes.laser-lines_utils")
local tfm = require("fawkes.tfutils")

local belt_offset_side = 0.025
local slide_offset_side = -0.225
local left_shelf_offset_side = -0.075
local middle_shelf_offset_side = -0.175
local right_shelf_offset_side = -0.275

local ring_height = 0.01

local drive_back_x = -0.4

-- read gripper config
local x_max = config:get_float("/arduino/x_max")
local y_max = config:get_float("/arduino/y_max")
local z_max = config:get_float("/arduino/z_max")

-- default gripper pose
local default_x = 0
local default_y = y_max / 2
local default_z = 0.03

local default_x_exit = 0.06
local default_y_exit = -0.03
local default_z_exit = 0.03
-- read config values for computing expected target position
-- conveyor
if config:exists("plugins/object_tracking/belt_values/belt_offset_side") then
    belt_offset_side = config:get_float(
                           "plugins/object_tracking/belt_values/belt_offset_side")
end
-- slide
if config:exists("plugins/object_tracking/slide_values/slide_offset_side") then
    slide_offset_side = config:get_float(
                            "plugins/object_tracking/slide_values/slide_offset_side")
end
-- shelf
if config:exists("plugins/object_tracking/shelf_values/left_shelf_offset_side") then
    left_shelf_offset_side = config:get_float(
                                 "plugins/object_tracking/shelf_values/left_shelf_offset_side")
end
if config:exists("plugins/object_tracking/shelf_values/middle_shelf_offset_side") then
    middle_shelf_offset_side = config:get_float(
                                   "plugins/object_tracking/shelf_values/middle_shelf_offset_side")
end
if config:exists("plugins/object_tracking/shelf_values/right_shelf_offset_side") then
    right_shelf_offset_side = config:get_float(
                                  "plugins/object_tracking/shelf_values/right_shelf_offset_side")
end

-- read wp config
if config:exists("plugins/object_tracking/puck_values/ring_height") then
    ring_height = config:get_float(
                      "plugins/object_tracking/puck_values/ring_height")
end

-- Match laser line to tf-mps point

function match_line(lines)
    local matched_line = nil

    local min_dist = MIN_ACTUAL_DIST
    local best_distance = nil

    for k, line in pairs(lines) do
        if (line:visibility_history() >= MIN_VIS_HIST_LINE) then
            local line_center = llutils.center(line, 0)
            local base_center = nil
            local distance = nil
            if fsm.vars.map_pos == true then
                base_center = tfm.transform6D({
                    x = line_center.x,
                    y = line_center.y,
                    z = 0,
                    ori = fawkes.tf.create_quaternion_from_yaw(line:bearing())
                }, line:frame_id(), fsm.vars.mps)
                distance = math.sqrt(base_center.x * base_center.x +
                                         base_center.y * base_center.y)
            end
            local robot_center = tfm.transform6D({
                x = line_center.x,
                y = line_center.y,
                z = 0,
                ori = fawkes.tf.create_quaternion_from_yaw(line:bearing())
            }, line:frame_id(), "base_link")
            local robot_distance = math.sqrt(
                                       robot_center.x * robot_center.x +
                                           robot_center.y * robot_center.y)

            if (fsm.vars.map_pos == true and distance < 0.50) or
                fsm.vars.map_pos == false then
                if best_distance == nil then
                    best_distance = robot_distance
                    matched_line = line
                elseif best_distance > robot_distance then
                    best_distance = robot_distance
                    matched_line = line
                end
            end
        end
    end
    return matched_line
end

function laser_line_found()
    fsm.vars.matched_line = match_line(fsm.vars.lines)
    return fsm.vars.matched_line ~= nil
end

function gripper_aligned()
    if fsm.vars.gripper_wait < 1 then return false end

    local gripper_target = tfm.transform6D({
        x = object_tracking_if:gripper_frame(0),
        y = object_tracking_if:gripper_frame(1),
        z = object_tracking_if:gripper_frame(2),
        ori = fawkes.tf.create_quaternion_from_yaw(0)
    }, "base_link", "end_effector_home")

    return within_tolerance(arduino:x_position(), 0, 0.0001) and
               within_tolerance(arduino:y_position() - y_max / 2, y_max / 2,
                                0.0001) and
               within_tolerance(
                   math.max(0.01, math.min(gripper_target.z, z_max)),
                   arduino:z_position(), GRIPPER_TOLERANCE.z)
end

function set_gripper(x, y, z)
    if not arduino:is_final() then
        fsm.vars.gripper_wait = 0
    elseif fsm.vars.gripper_wait < 10 then
        fsm.vars.gripper_wait = fsm.vars.gripper_wait + 1
    end

    -- Clip to axis limits
    x_clipped = math.max(0, math.min(x, x_max))
    y_clipped = math.max(-y_max / 2, math.min(y, y_max / 2))
    z_clipped = math.max(0.01, math.min(z, z_max))

    if x_clipped ~= x then
        print("Gripper cannot reache x-value: " .. x .. " ! Clipped to " ..
                  x_clipped)
    end
    if y_clipped ~= y then
        fsm.vars.out_of_reach = true
        print("Gripper cannot reache y-value: " .. y .. " ! Clipped to " ..
                  y_clipped)
        return
    end
    if z_clipped ~= z then
        print("Gripper cannot reache z-value: " .. z .. " ! Clipped to " ..
                  z_clipped)
    end

    if (not arduino:is_final() and
        (math.abs(fsm.vars.gripper_target_pos_y - y_clipped) >
            GRIPPER_TOLERANCE.y * 1 or
            math.abs(fsm.vars.gripper_target_pos_z - z_clipped) >
            GRIPPER_TOLERANCE.z * 1.5)) or
        (arduino:is_final() and fsm.vars.target == "SLIDE" and
            (math.abs(fsm.vars.gripper_target_pos_y - y_clipped) >
                GRIPPER_TOLERANCE.y * 1.3 or
                math.abs(fsm.vars.gripper_target_pos_z - z_clipped) >
                GRIPPER_TOLERANCE.z * 1.3)) or (arduino:is_final() and
        (math.abs(fsm.vars.gripper_target_pos_y - y_clipped) >
            GRIPPER_TOLERANCE.y or
            math.abs(fsm.vars.gripper_target_pos_z - z_clipped) >
            GRIPPER_TOLERANCE.z)) then
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
    move_abs_message:set_target_frame("end_effector_home")
    arduino:msgq_enqueue_copy(move_abs_message)
end

function move_gripper_default_pose()
    move_abs_message = arduino.MoveXYZAbsMessage:new()
    move_abs_message:set_x(default_x)
    move_abs_message:set_y(default_y)
    move_abs_message:set_z(default_z)
    move_abs_message:set_target_frame("end_effector_home")
    arduino:msgq_enqueue_copy(move_abs_message)
end

function move_gripper_default_pose_exit()
    local abs_message = arduino.MoveXYZAbsMessage:new()
    abs_message:set_x(default_x_exit)
    abs_message:set_y(default_y_exit)
    abs_message:set_z(default_z_exit)
    abs_message:set_target_frame("end_effector_home")
    arduino:msgq_enqueue_copy(abs_message)
    local close_msg = arduino.CloseGripperMessage:new()
    arduino:msgq_enqueue(close_msg)
end

function input_invalid()
    -- handle optional dry run
    if fsm.vars.dry_run == nil then fsm.vars.dry_run = false end

    if fsm.vars.query == "ABSENT" then
        fsm.vars.reverse_output = true
    else
        fsm.vars.reverse_output = false
    end

    if (fsm.vars.target_object_type == nil or
        string.gsub(fsm.vars.target_object_type, "^%s*(.-)%s*$", "%1") == 0) then
        print_error("That is not a valid target!")
        return true
    elseif (fsm.vars.expected_mps == nil or
        string.gsub(fsm.vars.expected_mps, "^%s*(.-)%s*$", "%1") == 0) then
        print_error("That is not a valid mps!")
        return true
    elseif (fsm.vars.expected_side == nil or
        string.gsub(fsm.vars.expected_side, "^%s*(.-)%s*$", "%1") == 0) then
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
            print_error(
                "If a conveyor is targeted, choose INPUT or OUTPUT as side!")
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

function within_tolerance(value, target, margin)
    return math.abs(value - target) < margin
end

function ready_for_gripper_movement()
    return within_tolerance(arduino:x_position(), default_x, 0.0001) and
               within_tolerance(arduino:y_position() - y_max / 2, default_y,
                                0.0001) and
               within_tolerance(arduino:z_position(), default_z, 0.0001)
end

function dry_expected_object_found()
    return fsm.vars.consecutive_detections > 2 and fsm.vars.dry_run and
               not fsm.vars.reverse_output
end

function dry_unexpected_object_found()
    return fsm.vars.consecutive_detections > 2 and fsm.vars.dry_run and
               fsm.vars.reverse_output
end

fsm:define_states{
    export_to = _M,
    closure = {MISSING_MAX = MISSING_MAX},
    {"INIT", JumpState},
    {"START_TRACKING", JumpState},
    {"FIND_LASER_LINE", JumpState},
    {
        "DRIVE_TO_LASER_LINE",
        SkillJumpState,
        skills = {{motor_move}},
        final_to = "WAIT_FOR_GRIPPER",
        fail_to = "FAILED"
    },
    {"SEARCH_LASER_LINE", JumpState},
    {"WAIT_FOR_GRIPPER", JumpState},
    {"AT_LASER_LINE", JumpState},
    {"DRY_RUN_ABSENT", JumpState},
    {
        "MOVE_BASE_AND_GRIPPER",
        SkillJumpState,
        skills = {{motor_move}},
        final_to = "FINE_TUNE_GRIPPER",
        fail_to = "FIND_LASER_LINE"
    },
    {"FINE_TUNE_GRIPPER", JumpState},
    {
        "GRIPPER_ROUTINE",
        SkillJumpState,
        skills = {{pick_or_put_vs}},
        final_to = "FINAL",
        fail_to = "FINE_TUNE_GRIPPER"
    }
}

fsm:add_transitions{
    {"INIT", "FAILED", cond = input_invalid, desc = "Invalid Input"},
    {"INIT", "START_TRACKING", cond = true, desc = "Valid Input"}, {
        "START_TRACKING",
        "FAILED",
        timeout = 2,
        desc = "Object tracker is not starting"
    }, {"START_TRACKING", "FIND_LASER_LINE", cond = object_tracker_active},
    {"FIND_LASER_LINE", "DRIVE_TO_LASER_LINE", cond = laser_line_found}, {
        "FIND_LASER_LINE",
        "SEARCH_LASER_LINE",
        timeout = 1,
        desc = "Could not find laser-line, drive back"
    }, {
        "SEARCH_LASER_LINE",
        "FAILED",
        cond = "vars.search_attemps > 10",
        desc = "Tried 10 times, could not find laser-line"
    }, {"SEARCH_LASER_LINE", "DRIVE_TO_LASER_LINE", cond = laser_line_found}, {
        "WAIT_FOR_GRIPPER",
        "AT_LASER_LINE",
        cond = ready_for_gripper_movement,
        desc = "Found Object"
    }, {
        "WAIT_FOR_GRIPPER",
        "START_TRACKING",
        timeout = 5,
        desc = "Something went wrong with axis movement"
    }, {
        "AT_LASER_LINE",
        "FINAL",
        cond = dry_expected_object_found,
        desc = "Found Object"
    }, {
        "AT_LASER_LINE",
        "DRY_RUN_ABSENT",
        cond = "vars.reverse_output",
        desc = "dry run with no object expected"
    }, {
        "AT_LASER_LINE",
        "MOVE_BASE_AND_GRIPPER",
        cond = "vars.consecutive_detections > 2",
        desc = "Found Object"
    }, {"AT_LASER_LINE", "FAILED", timeout = 2, desc = "Object not found"}, {
        "DRY_RUN_ABSENT",
        "FAILED",
        cond = dry_unexpected_object_found,
        desc = "Found Object"
    }, {"DRY_RUN_ABSENT", "FINAL", timeout = 2, desc = "Object not found"},
    {"FINE_TUNE_GRIPPER", "START_TRACKING", timeout = 10, desc = "Oscillating"},
    {
        "FINE_TUNE_GRIPPER",
        "GRIPPER_ROUTINE",
        cond = gripper_aligned,
        desc = "Gripper aligned"
    }, {
        "FINE_TUNE_GRIPPER",
        "MOVE_BASE_AND_GRIPPER",
        cond = "vars.out_of_reach",
        desc = "Gripper out of reach"
    }, {
        "FINE_TUNE_GRIPPER",
        "FIND_LASER_LINE",
        cond = "vars.missing_detections > MISSING_MAX",
        desc = "Tracking lost target"
    }
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

    fsm.vars.missing_detections = 0
    fsm.vars.msgid = 0
    fsm.vars.out_of_reach = false

    local TARGET_NAMES = {
        ["WORKPIECE"] = object_tracking_if.WORKPIECE,
        ["CONVEYOR"] = object_tracking_if.CONVEYOR_BELT_FRONT,
        ["SLIDE"] = object_tracking_if.SLIDE_FRONT
    }
    local MPS_NAMES = {
        ["M-BS"] = object_tracking_if.M_BS,
        ["M-RS1"] = object_tracking_if.M_RS1,
        ["M-RS2"] = object_tracking_if.M_RS2,
        ["M-CS1"] = object_tracking_if.M_CS1,
        ["M-CS2"] = object_tracking_if.M_CS2,
        ["M-DS"] = object_tracking_if.M_DS,
        ["M-SS"] = object_tracking_if.M_SS,
        ["C-BS"] = object_tracking_if.C_BS,
        ["C-RS1"] = object_tracking_if.C_RS1,
        ["C-RS2"] = object_tracking_if.C_RS2,
        ["C-CS1"] = object_tracking_if.C_CS1,
        ["C-CS2"] = object_tracking_if.C_CS2,
        ["C-DS"] = object_tracking_if.C_DS,
        ["C-SS"] = object_tracking_if.C_SS
    }
    local SIDE_NAMES = {
        ["INPUT"] = object_tracking_if.INPUT_CONVEYOR,
        ["OUTPUT"] = object_tracking_if.OUTPUT_CONVEYOR,
        ["SHELF-LEFT"] = object_tracking_if.SHELF_LEFT,
        ["SHELF-MIDDLE"] = object_tracking_if.SHELF_MIDDLE,
        ["SHELF-RIGHT"] = object_tracking_if.SHELF_RIGHT,
        ["SLIDE"] = object_tracking_if.SLIDE
    }

    if fsm.vars.map_pos ~= false then fsm.vars.map_pos = true end

    -- get ENUM of input variables
    fsm.vars.target_object_type = TARGET_NAMES[fsm.vars.target]
    fsm.vars.expected_mps = MPS_NAMES[fsm.vars.mps]
    fsm.vars.expected_side = SIDE_NAMES[fsm.vars.side]

    fsm.vars.gripper_target_pos_x = 0
    fsm.vars.gripper_target_pos_y = 0
    fsm.vars.gripper_target_pos_z = 0
    fsm.vars.gripper_wait = 0
end

function INIT:exit() fsm.vars.error = "invalid input" end

function START_TRACKING:init()
    -- start object tracking
    local msg = object_tracking_if.StartTrackingMessage:new(fsm.vars
                                                                .target_object_type,
                                                            fsm.vars
                                                                .expected_mps,
                                                            fsm.vars
                                                                .expected_side)
    object_tracking_if:msgq_enqueue_copy(msg)

    -- open gripper
    if fsm.vars.target == "WORKPIECE" and not fsm.vars.dry_run then
        local open_msg = arduino.OpenGripperMessage:new()
        arduino:msgq_enqueue(open_msg)
    end

    -- move to default pose
    move_gripper_default_pose()
end

function START_TRACKING:exit() fsm.vars.error = "OT interface closed" end

function FIND_LASER_LINE:init()
    -- start searching for laser line
    fsm.vars.search_attemps = 0
end

function SEARCH_LASER_LINE:loop()
    fsm.vars.search_attemps = fsm.vars.search_attemps + 1
end

function SEARCH_LASER_LINE:exit() fsm.vars.error = "laser-line not found" end

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
    local laser_target = tfm.transform6D({
        x = p.x,
        y = p.y - offset_y,
        z = 0,
        ori = fawkes.tf.create_quaternion_from_yaw(
            fsm.vars.matched_line:bearing())
    }, fsm.vars.matched_line:frame_id(), "/odom")

    if laser_target then
        self.args["motor_move"] = {
            x = laser_target.x,
            y = laser_target.y,
            frame = "/odom",
            ori = fawkes.tf.get_yaw(laser_target.ori),
            end_early = false,
            dry_run = fsm.vars.dry_run
        }
    else
        print_error("Transform Error: matched_line to odom")
    end
end

function DRY_RUN_ABSENT:loop()
    if fsm.vars.tracking_msgid ~= object_tracking_if:msgid() then
        fsm.vars.tracking_msgid = object_tracking_if:msgid()
        if object_tracking_if:is_detected() then
            fsm.vars.consecutive_detections =
                fsm.vars.consecutive_detections + 1
        else
            fsm.vars.consecutive_detections = 0
        end
    end
end

function DRIVE_TO_LASER_LINE:loop()
    if fsm.vars.tracking_msgid ~= object_tracking_if:msgid() then
        fsm.vars.tracking_msgid = object_tracking_if:msgid()
        if object_tracking_if:is_detected() then
            fsm.vars.consecutive_detections =
                fsm.vars.consecutive_detections + 1
        else
            fsm.vars.consecutive_detections = 0
        end
    end
end

function DRIVE_TO_LASER_LINE:exit() fsm.vars.error = "object not found" end

function AT_LASER_LINE:loop()
    if fsm.vars.tracking_msgid ~= object_tracking_if:msgid() then
        fsm.vars.tracking_msgid = object_tracking_if:msgid()
        if object_tracking_if:is_detected() then
            fsm.vars.consecutive_detections =
                fsm.vars.consecutive_detections + 1
        else
            fsm.vars.consecutive_detections = 0
        end
    end
end

function AT_LASER_LINE:exit() fsm.vars.error = "object not found" end

function MOVE_BASE_AND_GRIPPER:init()
    -- move base to target pose using visual servoing
    self.args["motor_move"] = {
        x = object_tracking_if:base_frame(0),
        y = object_tracking_if:base_frame(1),
        ori = -object_tracking_if:base_frame(5),
        frame = "base_link",
        visual_servoing = true
    }

    -- and move gripper to relative target gripper pose
    local base_x = object_tracking_if:base_frame(0)
    local base_y = object_tracking_if:base_frame(1)
    local gripper_x = object_tracking_if:gripper_frame(0)
    local gripper_y = object_tracking_if:gripper_frame(1)
    local gripper_z = object_tracking_if:gripper_frame(2)

    local diff_x = (gripper_x - base_x) * (gripper_x - base_x)
    local diff_y = (gripper_y - base_y) * (gripper_y - base_y)
    local forward_distance = math.sqrt(diff_x + diff_y)

    local gripper_target = tfm.transform6D({
        x = forward_distance,
        y = 0,
        z = gripper_z,
        ori = fawkes.tf.create_quaternion_from_yaw(0)
    }, "base_link", "end_effector_home")

    fsm.vars.gripper_wait = 10
    set_gripper(0, y_max / 2, gripper_target.z)
end

function FINE_TUNE_GRIPPER:init()
    fsm.vars.missing_detections = 0
    fsm.vars.out_of_reach = false
    fsm.vars.gripper_wait = 10
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
    local gripper_target = tfm.transform6D({
        x = object_tracking_if:gripper_frame(0),
        y = object_tracking_if:gripper_frame(1),
        z = object_tracking_if:gripper_frame(2),
        ori = fawkes.tf.create_quaternion_from_yaw(0)
    }, "base_link", "end_effector_home")

    set_gripper(0, y_max / 2, gripper_target.z)
end

function GRIPPER_ROUTINE:init()
    -- end tracking since VS is over
    object_tracking_if:msgq_enqueue(object_tracking_if.StopTrackingMessage:new())

    -- perform pick or put routine
    self.args["pick_or_put_vs"].target = fsm.vars.target

    if fsm.vars.side == "SHELF-LEFT" or fsm.vars.side == "SHELF-MIDDLE" or
        fsm.vars.side == "SHELF-RIGHT" then
        self.args["pick_or_put_vs"].shelf = true
    else
        self.args["pick_or_put_vs"].shelf = false
    end
end

-- end tracking afterwards

function FINAL:init()
    move_gripper_default_pose_exit()
    object_tracking_if:msgq_enqueue(object_tracking_if.StopTrackingMessage:new())
end

function FAILED:init()
    move_gripper_default_pose_exit()
    object_tracking_if:msgq_enqueue(object_tracking_if.StopTrackingMessage:new())

    -- keep track of error
    fsm:set_error(fsm.vars.error)
end
