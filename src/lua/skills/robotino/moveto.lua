----------------------------------------------------------------------------
--  moveto.lua -
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré
--              2015  Tobias Neumann
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
name = "moveto"
fsm = SkillHSM:new{name = name, start = "CHECK_INPUT"}
depends_skills = {}
depends_interfaces = {
    --   {v = "pose", type="Position3DInterface", id="Pose"},
    {v = "navigator", type = "NavigatorInterface", id = "Navigator"},
    {v = "throttle", type = "SwitchInterface", id = "motor-throttle"},
    {v = "laserline_switch", type = "SwitchInterface", id = "laser-lines"}
}

documentation = [==[Move to a known location via place or x, y, ori.
if place is set, this will be used and x, y and ori will be ignored

@param place  Name of the place we want to go to.
@param x      x we want to drive to
@param y      y we want to drive to
@param slow   move the robot slow
@param ori    ori we want to drive to

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tf_mod = require 'fawkes.tfutils'

if config:exists("/skills/moveto/distance_to_travel") then
    distance_to_travel = config:get_float("/skills/moveto/distance_to_travel")
else
    distance_to_travel = 0.5
end

-- Tunables
-- local REGION_TRANS=0.2

function check_tf(self)
    if self.fsm.vars.place ~= nil then
        if string.match(self.fsm.vars.place, "WAIT") then return false end
        if string.match(self.fsm.vars.place, "^[MC][-]Z[1-7][1-8]$") then
            return false
        end
        if fsm.vars.place ~= "" then
            local tf_point = tf_mod.transform6D({
                x = 0,
                y = 0,
                z = 0,
                ori = fawkes.tf.create_quaternion_from_yaw(0)
            }, fsm.vars.place, "map")
            return tf_point == nil
        end
        return true
    end
    return false
end

function target_reached()
    if navigator:is_final() and navigator:error_code() ~= 0 then return false end
    return navigator:is_final()
end

function has_navigator() return navigator:has_writer() end

function can_navigate(self)
    return self.fsm.vars.x ~= nil and self.fsm.vars.y ~= nil
end
-- Define the error code mappings
local planner_errors = {
    [0] = "NONE",
    [200] = "UNKNOWN",
    [201] = "INVALID_PLANNER",
    [202] = "TF_ERROR",
    [203] = "START_OUTSIDE_MAP",
    [204] = "GOAL_OUTSIDE_MAP",
    [205] = "START_OCCUPIED",
    [206] = "GOAL_OCCUPIED",
    [207] = "TIMEOUT",
    [208] = "NO_VALID_PATH"
}

local follow_path_errors = {
    [0] = "NONE",
    [100] = "UNKNOWN",
    [101] = "INVALID_CONTROLLER",
    [102] = "TF_ERROR",
    [103] = "INVALID_PATH",
    [104] = "PATIENCE_EXCEEDED",
    [105] = "FAILED_TO_MAKE_PROGRESS",
    [106] = "NO_VALID_CONTROL"
}

-- Function to get the error message
local function get_error_message(error_code)
    if planner_errors[error_code] then
        return planner_errors[error_code]
    elseif follow_path_errors[error_code] then
        return follow_path_errors[error_code]
    else
        return "UNKNOWN_ERROR"
    end
end

function target_unreachable()
    if navigator:is_final() and navigator:error_code() ~= 0 then
        local error_message = get_error_message(navigator:error_code())
        fsm.vars.error = error_message
        return true
    end
    return false
end

function travelled_distance(self)
    -- Skill will final after it travelled the euclidean distance of 1m from its starting position--
    local x = (self.fsm.vars.cur_x - self.fsm.vars.initial_position_x) *
                  (self.fsm.vars.cur_x - self.fsm.vars.initial_position_x)
    local y = (self.fsm.vars.cur_y - self.fsm.vars.initial_position_y) *
                  (self.fsm.vars.cur_y - self.fsm.vars.initial_position_y)
    local distance_travelled = math.sqrt(x + y)

    if distance_travelled > distance_to_travel then
        return true
    else
        return false
    end
end

fsm:define_states{
    export_to = _M,
    closure = {
        check_tf = check_tf,
        reached_target_region = reached_target_region,
        has_navigator = has_navigator,
        travelled_distance = travelled_distance
    },
    {"CHECK_INPUT", JumpState},
    {"WAIT_TF", JumpState},
    {"INIT", JumpState},
    {"MOVING", JumpState},
    {"TIMEOUT", JumpState}
}

fsm:add_transitions{
    {
        "CHECK_INPUT",
        "FAILED",
        cond = "not has_navigator()",
        desc = "Navigator not running"
    }, {"CHECK_INPUT", "INIT", cond = can_navigate},
    {"CHECK_INPUT", "WAIT_TF", cond = true},
    {"WAIT_TF", "INIT", cond = can_navigate},
    {"INIT", "FAILED", precond = check_tf, desc = "no tf"},
    {"INIT", "FAILED", cond = "not vars.target_valid", desc = "target invalid"},
    {"INIT", "MOVING", cond = true}, {"MOVING", "TIMEOUT", timeout = 2}, -- Give the interface some time to update
    {"TIMEOUT", "FINAL", cond = target_reached, desc = "Target reached"},
    {
        "TIMEOUT",
        "FAILED",
        cond = target_unreachable,
        desc = "Target unreachable"
    }
}

function INIT:init()
    self.fsm.vars.target_valid = true
    self.fsm.vars.waiting_pos = false
    self.fsm.vars.slow = self.fsm.vars.slow or false
    if not self.fsm.vars.slow then
        throttle:msgq_enqueue(throttle.DisableSwitchMessage:new())
    end

    if self.fsm.vars.place ~= nil then
        -- check for waiting position
        if string.match(self.fsm.vars.place, "WAIT") then
            self.fsm.vars.waiting_pos = true
        end
        if string.match(self.fsm.vars.place, "^[MC][-]Z[1-7][1-8]$") then
            -- place argument is a zone, e.g. M-Z21
            self.fsm.vars.zone = self.fsm.vars.place
            self.fsm.vars.x = tonumber(string.sub(self.fsm.vars.place, 4, 4)) -
                                  0.5
            self.fsm.vars.y = tonumber(string.sub(self.fsm.vars.place, 5, 5)) -
                                  0.5
            if string.sub(self.fsm.vars.place, 1, 1) == "M" then
                self.fsm.vars.x = 0 - self.fsm.vars.x
            end
        else
            -- place argument is a navgraph point
            local tf_point = tf_mod.transform6D({
                x = 0,
                y = 0,
                z = 0,
                ori = fawkes.tf.create_quaternion_from_yaw(0)
            }, fsm.vars.place, "map")

            if tf_point ~= nil then
                self.fsm.vars.x = tf_point.x
                self.fsm.vars.y = tf_point.y
                self.fsm.vars.ori = fawkes.tf.get_yaw(tf_point.ori);

                self.fsm.vars.ori_tolerance = 3.14;
                self.fsm.vars.trans_tolerance = 0.6;

            else
                self.fsm.vars.target_valid = false
                self.fsm.error = "target invalid"
            end
        end
    else
        -- infinity tells the navigator to ignore ori
        self.fsm.vars.ori = self.fsm.vars.ori or 1 / 0
    end

    self.fsm.vars.region_trans = self.fsm.vars.region_trans or REGION_TRANS
end

function WAIT_TF:loop()
    local cur_pose = tf_mod.transform({x = 0, y = 0, ori = 0}, "base_link",
                                      "map")
    if cur_pose == nil then
        print_warn("Failed to transform from 'base_link' to 'map'!")
        return
    end
    self.fsm.vars.x = cur_pose.x
    self.fsm.vars.y = cur_pose.y
    self.fsm.vars.ori = cur_pose.ori
    self.fsm.vars.initial_position_x = cur_pose.x
    self.fsm.vars.initial_position_y = cur_pose.y
    self.fsm.vars.initial_position_ori = cur_pose.ori
    self.fsm.vars.cur_x = cur_pose.x
    self.fsm.vars.cur_y = cur_pose.y
    self.fsm.vars.cur_ori = cur_pose.ori
end

function MOVING:init()
    self.fsm.vars.msgid_timeout = os.time() + 1

    print(self.fsm.vars.ori)

    local msg = navigator.CartesianGotoWithFrameMessage:new(self.fsm.vars.x,
                                                            self.fsm.vars.y,
                                                            self.fsm.vars.ori,
                                                            "map")

    -- Use a tolerance if it is defined.
    if self.fsm.vars.trans_tolerance ~= nil and self.fsm.vars.ori_tolerance ~=
        nil then
        msg = navigator.CartesianGotoWithFrameWithToleranceMessage:new(self.fsm
                                                                           .vars
                                                                           .x,
                                                                       self.fsm
                                                                           .vars
                                                                           .y,
                                                                       self.fsm
                                                                           .vars
                                                                           .ori,
                                                                       "map",
                                                                       self.fsm
                                                                           .vars
                                                                           .trans_tolerance,
                                                                       self.fsm
                                                                           .vars
                                                                           .ori_tolerance)
    end

    fsm.vars.moveto_msgid = navigator:msgq_enqueue(msg)
end

function TIMEOUT:loop()
    if fsm.vars.waiting_pos == true then
        local got_cur_pose = false

        while not got_cur_pose do
            local cur_pose = tf_mod.transform({x = 0, y = 0, ori = 0},
                                              "base_link", "map")
            if cur_pose ~= nil then
                self.fsm.vars.cur_x = cur_pose.x
                self.fsm.vars.cur_y = cur_pose.y
                self.fsm.vars.cur_ori = cur_pose.ori
                got_cur_pose = true
            end
        end
    end
end

function MOVING:reset()
    if navigator:has_writer() and not navigator:is_final() and
        self.fsm.vars.waiting_pos == false then
        printf("moveto: sending stop");
        navigator:msgq_enqueue(navigator.StopMessage:new(fsm.vars.msgid or 0))
    end
end

function FAILED:init()
    -- keep track of error
    fsm:set_error(fsm.vars.error)
end
