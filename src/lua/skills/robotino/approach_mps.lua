----------------------------------------------------------------------------
--  approach_mps.lua
--
--  Created Sun Jun 26 11:44:40 2016
--  Copyright  2016  Frederik Zwilling
--             2017  Tobias Neumann
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
name = "approach_mps"
fsm = SkillHSM:new{name = name, start = "INIT", debug = false}
depends_skills = {"motor_move"}
depends_interfaces = {
    {v = "if_conveyor", type = "Position3DInterface", id = "conveyor_pose/pose"},
    {
        v = "conveyor_switch",
        type = "SwitchInterface",
        id = "conveyor_pose/switch"
    }, {v = "if_front_dist", type = "Position3DInterface", id = "front_dist"}
}

documentation = [==[
                        The robot just drives forward according to the current distance to the mps and the desired position
                        @param "x" int The x distance of the conveyor in the base_link frame when finished
--                        @param "use_conveyor" default is true, set this to false when the conveyor is not visibly
                     ]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local cfg_frame_ = "gripper"

function transformed_pose(self)
    local from = {
        x = if_conveyor:translation(0),
        y = if_conveyor:translation(1),
        z = if_conveyor:translation(2),
        ori = {
            x = if_conveyor:rotation(0),
            y = if_conveyor:rotation(1),
            z = if_conveyor:rotation(2),
            w = if_conveyor:rotation(3)
        }
    }
    local cp = tfm.transform6D(from, if_conveyor:frame(), cfg_frame_)

    -- TODO check nil

    local ori = fawkes.tf.get_yaw(fawkes.tf.Quaternion:new(cp.ori.x, cp.ori.y,
                                                           cp.ori.z, cp.ori.w))

    return {x = cp.x, y = cp.y, z = cp.z, ori = ori}
end

function conveyor_ready(self)
    if not self.fsm.vars.use_conveyor then return false end

    self.fsm.vars.pose = transformed_pose()
    if if_conveyor:visibility_history() > 5 and self.fsm.vars.pose.x < 0.5 then
        return true
    else
        printf(
            "mps_approach failed: visibility history is %f, dist to object in front is %f. I don't drive with this visibility history or this far without collision avoidance",
            if_conveyor:visibility_history(), self.fsm.vars.pose.x)
        return false
    end
end

function laser_lines_ready(self)
    if self.fsm.vars.use_conveyor then return false end

    if if_front_dist:visibility_history() > 0 and self.fsm.vars.ll_dist < 1.0 then
        return true
    else
        printf(
            "mps_approach failed: visibility history is %f, dist to object in front is %f. I don't drive with this visibility history or this far without collision avoidance",
            if_front_dist:visibility_history(), self.fsm.vars.ll_dist)
        return false
    end
end

fsm:define_states{
    export_to = _M,
    closure = {
        conveyor_ready = conveyor_ready,
        laser_lines_ready = laser_lines_ready
    },
    {"INIT", JumpState},
    {
        "APPROACH_CONVEYOR",
        SkillJumpState,
        skills = {{motor_move}},
        final_to = "FINAL",
        fail_to = "INIT_LASER_LINES"
    },
    {"INIT_LASER_LINES", JumpState},
    {
        "APPROACH_LASERLINE",
        SkillJumpState,
        skills = {{motor_move}},
        final_to = "FINAL",
        fail_to = "FAILED"
    }
}

fsm:add_transitions{
    {"INIT", "APPROACH_CONVEYOR", cond = conveyor_ready},
    {"INIT", "APPROACH_LASERLINE", cond = laser_lines_ready},
    {"INIT", "INIT_LASER_LINES", timeout = 5.0},
    {"INIT_LASER_LINES", "APPROACH_LASERLINE", cond = "laser_lines_ready(self)"},
    {"INIT_LASER_LINES", "FAILED", timeout = 1.0}
}

function INIT:init()
    if self.fsm.vars.use_conveyor == nil then
        self.fsm.vars.use_conveyor = true
    end

    conveyor_switch:msgq_enqueue_copy(conveyor_switch.EnableSwitchMessage:new())
    self.fsm.vars.ll_dist = if_front_dist:translation(0) - self.fsm.vars.x
end

function INIT_LASER_LINES:init() end

function APPROACH_CONVEYOR:init()
    local x_goal = self.fsm.vars.pose.x - self.fsm.vars.x
    printf("distance is: %f => drive to: %f", self.fsm.vars.pose.x, x_goal)
    self.args["motor_move"] = {
        x = self.fsm.vars.x,
        vel_trans = 0.05,
        frame = "conveyor_pose"
    }
end

function APPROACH_LASERLINE:init()
    local x_goal = if_front_dist:translation(0) - self.fsm.vars.x
    printf("distance is: %f => drive to: %f", if_front_dist:translation(0),
           x_goal)
    self.args["motor_move"] = {
        x = self.fsm.vars.x,
        vel_trans = 0.05,
        frame = "front_dist"
    }
end

function cleanup()
    conveyor_switch:msgq_enqueue_copy(conveyor_switch.DisableSwitchMessage:new())
end

function FINAL:init() cleanup() end

function FAILED:init()
    cleanup()
    self.fsm.vars.pose = transformed_pose()
    printf(
        "mps_approach failed (maybe because of subskill): visibility history is %f, dist to object in front is %f.",
        if_conveyor:visibility_history(), self.fsm.vars.pose.x)
end
