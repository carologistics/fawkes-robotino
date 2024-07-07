----------------------------------------------------------------------------
--  switch_ot.lua
--
--  Created: Wed Jul 05
--  Copyright  2024  Matteo Tschesche
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

-- Crucial skill informatio
name = "switch_ot"
fsm = SkillHSM:new{name = name, start = "INIT", debug = true}
depends_skills = {}
depends_interfaces = {
    {
        v = "object_tracking_if",
        type = "ObjectTrackingInterface",
        id = "object-tracking"
    }, {v = "arduino", type = "ArduinoInterface", id = "Arduino"},
    {v = "laserline_switch", type = "SwitchInterface", id = "laser-lines"}
}

documentation = [==[
Switches object tracking on or off.

Parameters:
      @param switch  the type of message send to the object tracking (START | END)
      @param target  the type of the target object: (WORKPIECE | CONVEYOR | SLIDE)
      @param mps     the name of the MPS (e.g. C-CS1, see navgraph)
      @param side    the side of the mps: (INPUT | OUTPUT | SHELF-LEFT | SHELF-MIDDLE | SHELF-RIGHT | SLIDE)
]==]

-- Initialize as skill module

skillenv.skill_module(_M)
local llutils = require("fawkes.laser-lines_utils")
local tfm = require("fawkes.tfutils")

-- Load config
local x_max = 0.115
local y_max = 0.075
local z_max = 0.057

local belt_offset_side = 0.025
local slide_offset_side = -0.225
local left_shelf_offset_side = -0.075
local middle_shelf_offset_side = -0.175
local right_shelf_offset_side = -0.275

local ring_height = 0.01

local drive_back_x = -0.4

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

function move_gripper_default_pose()
    move_abs_message = arduino.MoveXYZAbsMessage:new()
    move_abs_message:set_x(0)
    move_abs_message:set_y(0)
    move_abs_message:set_z(z_max)
    move_abs_message:set_target_frame("gripper_home")
    arduino:msgq_enqueue_copy(move_abs_message)
end

function input_invalid()
    -- handle optional dry run
    if not (fsm.vars.switch == "START" or fsm.vars.switch == "END") then
        return true
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

function is_starting()
    if fsm.vars.switch == "START" then
        return true
    else
        return false
    end
end

function is_ending()
    if fsm.vars.switch == "END" then
        return true
    else
        return false
    end
end

fsm:define_states{
    export_to = _M,
    closure = {MISSING_MAX = MISSING_MAX},
    {"INIT", JumpState},
    {"CHOOSE_ACTION", JumpState},
    {"START_TRACKING", JumpState},
    {"END_TRACKING", JumpState}
}

fsm:add_transitions{
    {"INIT", "FAILED", cond = input_invalid, desc = "Invalid Input"},
    {"INIT", "CHOOSE_ACTION", cond = true, desc = "Valid Input"},
    {
        "CHOOSE_ACTION",
        "START_TRACKING",
        cond = is_starting,
        desc = "Valid Input"
    },
    {"CHOOSE_ACTION", "END_TRACKING", cond = is_ending, desc = "Valid Input"},
    {
        "START_TRACKING",
        "FAILED",
        timeout = 2,
        desc = "Object tracker is not starting"
    }, {"START_TRACKING", "FINAL", cond = object_tracker_active},
    {"END_TRACKING", "FINAL", cond = true}
}

function INIT:init()
    laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())

    fsm.vars.msgid = 0

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

    -- get ENUM of input variables
    fsm.vars.target_object_type = TARGET_NAMES[fsm.vars.target]
    fsm.vars.expected_mps = MPS_NAMES[fsm.vars.mps]
    fsm.vars.expected_side = SIDE_NAMES[fsm.vars.side]
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

    -- move to default pose
    move_gripper_default_pose()
end

function START_TRACKING:exit() fsm.vars.error = "OT interface closed" end

function END_TRACKING:init()
    object_tracking_if:msgq_enqueue(object_tracking_if.StopTrackingMessage:new())
end

function END_TRACKING:exit() fsm.vars.error = "OT interface still open" end
