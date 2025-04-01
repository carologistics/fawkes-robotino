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
name = "pick_or_put_vs"
fsm = SkillHSM:new{name = name, start = "INIT", debug = true}
depends_skills = {"gripper_commands", "motor_move"}
depends_interfaces = {
    {
        v = "object_tracking_if",
        type = "ObjectTrackingInterface",
        id = "object-tracking"
    }, {v = "arduino", type = "ArduinoInterface", id = "Arduino"}
}

documentation = [==[
Skill to pick a product and to put it down based on param action.
It is independent of the workpiece location or its target location.

Parameters:
      @param target             target object (WORKPIECE | CONVEYOR | SLIDE)
      @param shelf              bool (default: false)
      @param x                  set x directly and ingnore object tracking in x (optional, float)
      @param y                  set y directly and ingnore object tracking in y (optional, float)
      @param z                  set z directly and ingnore object tracking in z (optional, float)
]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

local drive_back_x = -0.2

-- read gripper config
local x_max = config:get_float("/arduino/x_max") -- gripper max value in x direction
local y_max = config:get_float("/arduino/y_max") -- gripper max value in y direction
local z_max = config:get_float("/arduino/z_max") -- gripper max value in z direction

-- read vs configs
local offset_x_workpiece_target_frame = config:get_float(
                                            "plugins/vs_offsets/workpiece/target/x")
local offset_x_shelf_target_frame = config:get_float(
                                        "plugins/vs_offsets/shelf/target/x")
local offset_x_conveyor_target_frame = config:get_float(
                                           "plugins/vs_offsets/conveyor/target/x")
local offset_x_slide_target_frame = config:get_float(
                                        "plugins/vs_offsets/slide/target/x")

local offset_z_workpiece_target_frame = config:get_float(
                                            "plugins/vs_offsets/workpiece/target/z")
local offset_z_shelf_target_frame = config:get_float(
                                        "plugins/vs_offsets/shelf/target/z")
local offset_z_conveyor_target_frame = config:get_float(
                                           "plugins/vs_offsets/conveyor/target/z")
local offset_z_slide_target_frame = config:get_float(
                                        "plugins/vs_offsets/slide/target/z")

local offset_z_conveyor_down = config:get_float(
                                   "plugins/vs_offsets/conveyor/down/z")
local offset_z_slide_down = config:get_float("plugins/vs_offsets/slide/down/z")

local offset_z_workpiece_up = config:get_float(
                                  "plugins/vs_offsets/workpiece/up/z")
local offset_z_shelf_up = config:get_float("plugins/vs_offsets/shelf/up/z")
local offset_z_conveyor_up =
    config:get_float("plugins/vs_offsets/conveyor/up/z")
local offset_z_slide_up = config:get_float("plugins/vs_offsets/slide/up/z")

-- default gripper pose
local default_x = 0.0
local default_y = -0.07
local default_z = 0.025

function input_invalid()
    if fsm.vars.shelf ~= true then fsm.vars.shelf = false end
    if fsm.vars.target == "WORKPIECE" or fsm.vars.target == "CONVEYOR" or
        fsm.vars.target == "SLIDE" then
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

function is_put_action() return not is_pick_action() end

fsm:define_states{
    export_to = _M,
    closure = {},
    {"INIT", JumpState},
    {"CHOOSE_FORWARD_ROUTINE", JumpState},
    {
        "MOVE_GRIPPER_FORWARD",
        SkillJumpState,
        skills = {{gripper_commands}},
        final_to = "MOVE_GRIPPER_DOWN",
        fail_to = "FAILED"
    },
    {
        "MOVE_GRIPPER_DOWN",
        SkillJumpState,
        skills = {{gripper_commands}},
        final_to = "CHOOSE_ACTION",
        fail_to = "FAILED"
    },
    {"CHOOSE_ACTION", JumpState},
    {
        "CLOSE_GRIPPER",
        SkillJumpState,
        skills = {{gripper_commands}},
        final_to = "MOVE_GRIPPER_UP",
        fail_to = "FAILED"
    },
    {
        "OPEN_GRIPPER",
        SkillJumpState,
        skills = {{gripper_commands}},
        final_to = "MOVE_GRIPPER_UP",
        fail_to = "FAILED"
    },
    {
        "MOVE_GRIPPER_UP",
        SkillJumpState,
        skills = {{gripper_commands}},
        final_to = "DRIVE_BACK",
        fail_to = "FAILED"
    },
    {
        "DRIVE_BACK",
        SkillJumpState,
        skills = {{motor_move}},
        final_to = "FINAL",
        fail_to = "FAILED"
    }
}

fsm:add_transitions{
    {"INIT", "FAILED", cond = input_invalid, desc = "Invalid Input"},
    {"INIT", "MOVE_GRIPPER_FORWARD", true, desc = "Start Routine"}, {
        "CHOOSE_ACTION",
        "CLOSE_GRIPPER",
        cond = is_pick_action,
        desc = "Picking Up Workpiece"
    }, {
        "CHOOSE_ACTION",
        "OPEN_GRIPPER",
        cond = is_put_action,
        desc = "Putting Down Workpiece"
    }, {"CHOOSE_ACTION", "FAILED", true, desc = "Instructions Unclear"}
}

function INIT:init()
    fsm.vars.object_tracking_target = tfm.transform6D({
        x = object_tracking_if:gripper_frame(0),
        y = object_tracking_if:gripper_frame(1),
        z = object_tracking_if:gripper_frame(2),
        ori = fawkes.tf.create_quaternion_from_yaw(0)
    }, "base_link", "end_effector_home")

    fsm.vars.gripper_target = {x = 0.0, y = 0.0, z = 0.0}
    if fsm.vars.x ~= nil and fsm.vars.x > 0.0 then
        fsm.vars.gripper_target.x = fsm.vars.x
    else
        fsm.vars.gripper_target.x = fsm.vars.object_tracking_target.x
    end
    if fsm.vars.y ~= nil then
        fsm.vars.gripper_target.y = fsm.vars.y
    else
        fsm.vars.gripper_target.y = fsm.vars.object_tracking_target.y
    end
    if fsm.vars.z ~= nil and fsm.vars.z > 0.0 then
        fsm.vars.gripper_target.z = fsm.vars.z
    else
        fsm.vars.gripper_target.z = fsm.vars.object_tracking_target.z
    end
end

function MOVE_GRIPPER_FORWARD:init()
    -- Clip to axis limits
    local x_given = 0
    local z_given = 0
    local sense_wp = false
    if fsm.vars.target == "WORKPIECE" then sense_wp = true end

    local x_clipped = math.max(0, math.min(fsm.vars.gripper_target.x, x_max))
    local y_clipped = math.max(-y_max / 2,
                               math.min(fsm.vars.gripper_target.y, y_max / 2))
    local z_clipped = math.max(0.01, math.min(fsm.vars.gripper_target.z, z_max))

    self.args["gripper_commands"].x = x_clipped
    self.args["gripper_commands"].y = y_clipped
    self.args["gripper_commands"].z = z_clipped
    self.args["gripper_commands"].sense = sense_wp
    self.args["gripper_commands"].command = "MOVEABS"
end

function MOVE_GRIPPER_DOWN:init()
    local z_given = 0
    if fsm.vars.target == "WORKPIECE" and fsm.vars.shelf then
        z_given = fsm.vars.gripper_target.z + offset_z_shelf_down
    elseif fsm.vars.target == "WORKPIECE" then
        z_given = fsm.vars.gripper_target.z + offset_z_workpiece_down
    elseif fsm.vars.target == "CONVEYOR" then
        z_given = fsm.vars.gripper_target.z + offset_z_conveyor_down
    else -- SLIDE
        z_given = fsm.vars.gripper_target.z + offset_z_slide_down
    end

    local x_clipped = math.max(0, math.min(fsm.vars.gripper_target.x, x_max))
    local y_clipped = math.max(-y_max / 2,
                               math.min(fsm.vars.gripper_target.y, y_max / 2))
    local z_clipped = math.max(0.01, math.min(z_given, z_max))

    self.args["gripper_commands"].x = x_clipped
    self.args["gripper_commands"].y = y_clipped
    self.args["gripper_commands"].z = z_clipped
    self.args["gripper_commands"].command = "MOVEABS"

end

function CLOSE_GRIPPER:init() self.args["gripper_commands"].command = "CLOSE" end

function OPEN_GRIPPER:init() self.args["gripper_commands"].command = "OPEN" end

function MOVE_GRIPPER_UP:init()
    local z_given = 0
    if fsm.vars.target == "WORKPIECE" and fsm.vars.shelf then
        z_given = fsm.vars.gripper_target.z - offset_z_shelf_target_frame +
                      offset_z_shelf_up
    elseif fsm.vars.target == "WORKPIECE" then
        z_given = fsm.vars.gripper_target.z - offset_z_workpiece_target_frame +
                      offset_z_workpiece_up
    elseif fsm.vars.target == "CONVEYOR" then
        z_given = fsm.vars.gripper_target.z - offset_z_conveyor_target_frame +
                      offset_z_conveyor_up
    else -- SLIDE
        z_given = fsm.vars.gripper_target.z - offset_z_slide_target_frame +
                      offset_z_slide_up
    end

    local z_clipped = math.max(0.01, math.min(z_given, z_max))

    self.args["gripper_commands"].x = arduino:x_position()
    self.args["gripper_commands"].y = arduino:y_position() - y_max / 2
    self.args["gripper_commands"].z = math.max(0.01, math.min(z_clipped, z_max))
    self.args["gripper_commands"].command = "MOVEABS"
end

function DRIVE_BACK:init()
    self.args["motor_move"].x = drive_back_x

    -- move gripper back
    move_abs_message = arduino.MoveXYZAbsMessage:new()
    move_abs_message:set_x(default_x)
    move_abs_message:set_y(default_y)
    move_abs_message:set_z(default_z)
    move_abs_message:set_target_frame("end_effector_home")
    arduino:msgq_enqueue_copy(move_abs_message)
end

function DRIVE_BACK:exit()
    -- close gripper if needed
    if fsm.vars.target ~= "WORKPIECE" then
        local close_msg = arduino.CloseGripperMessage:new()
        arduino:msgq_enqueue_copy(close_msg)
    end
end
