----------------------------------------------------------------------------
--  moveto_corner_turn.lua
--
--  Created: Thu Jun 12 15:25:00 2023
--  Copyright  2023 Daniel Hones
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
name = "moveto_corner_turn"
fsm = SkillHSM:new{name = name, start = "INIT", debug = true}
depends_skills = {"motor_move", "moveto", "turn_to_search"}
depends_interfaces = {
    {v = "pose", type = "Position3DInterface", id = "Pose"},
    {v = "throttle", type = "SwitchInterface", id = "motor-throttle"}
}

documentation = [==[Simple exploration for evaluation of danielhonies thesis
@param: place place we want to go to
@param: x     x we want to drive to
@param: y     y we want to drive to
@param: min_x map min x coordinate
@param: max_x map max x coordinate
@param: min_y map min y coordinate
@param: max_y map max y coordinate
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT_UPPER_LIMIT = 60
local MIN_X_MAP = -5
local MAX_X_MAP = 0
local MIN_Y_MAP = 0
local MAX_Y_MAP = 6
local turn = 4

fsm:define_states{
    export_to = _M,
    {"INIT", JumpState},
    {"GOTO_CORNER", SkillJumpState, skills = {{moveto}}, final_to = "TURN"},
    {"TURN", SkillJumpState, skills = {{turn_to_search}}, final_to = "FINAL"}

}

fsm:add_transitions{{"INIT", "GOTO_CORNER", cond = true}}

function TURN:init() self.args["turn_to_search"] = {turns = turn * 2} end

function GOTO_CORNER:init()
    throttle:msgq_enqueue(throttle.EnableSwitchMessage:new())
    local index = 0;
    if self.fsm.vars.min_x ~= nil then MIN_X_MAP = self.fsm.vars.min_x end
    if self.fsm.vars.max_x ~= nil then MAX_X_MAP = self.fsm.vars.max_x end
    if self.fsm.vars.min_y ~= nil then MIN_Y_MAP = self.fsm.vars.min_y end
    if self.fsm.vars.max_y ~= nil then MAX_Y_MAP = self.fsm.vars.max_y end

    if self.fsm.vars.place ~= nil then
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
        end
    end
    turn = 4;
    if self.fsm.vars.x + 1 > MAX_X_MAP and self.fsm.vars.y - 2 > MIN_Y_MAP and
        self.fsm.vars.y + 1 < MAX_Y_MAP then
        -- 1,y
        index = 1;
        turn = 2;
    elseif self.fsm.vars.x - 1 < MIN_X_MAP and self.fsm.vars.y - 2 > MIN_Y_MAP and
        self.fsm.vars.y + 1 < MAX_Y_MAP then
        -- 0,y
        index = 3;
        turn = 2;
    elseif self.fsm.vars.x - 1 > MIN_X_MAP and self.fsm.vars.x + 1 < MAX_X_MAP and
        self.fsm.vars.y + 1 > MAX_Y_MAP then
        -- x,1
        index = 2;
        turn = 2;
    elseif self.fsm.vars.x - 1 > MIN_X_MAP and self.fsm.vars.x + 1 < MAX_X_MAP and
        self.fsm.vars.y - 2 < MIN_Y_MAP then
        -- x,0
        index = 0;
        turn = 2;
    elseif self.fsm.vars.x + 1 > MAX_X_MAP and self.fsm.vars.y + 1 > MAX_Y_MAP then
        -- 1,1
        index = 2;
        turn = 1;
    elseif self.fsm.vars.x + 1 > MAX_X_MAP and self.fsm.vars.y - 2 < MIN_Y_MAP then
        -- 1,0
        index = 1;
        turn = 1;
    elseif self.fsm.vars.x - 1 < MIN_X_MAP and self.fsm.vars.y + 1 > MAX_Y_MAP then
        -- 0,1
        index = 3;
        turn = 1;
    elseif self.fsm.vars.x - 1 < MIN_X_MAP and self.fsm.vars.y - 2 < MIN_Y_MAP then
        -- 0,0
        index = 0;
        turn = 1;
    end

    self.args["moveto"] = {
        ori = 1.570795 * index,
        x = self.fsm.vars.x,
        y = self.fsm.vars.y,
        slow = true
    }
end
