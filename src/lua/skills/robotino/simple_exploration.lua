----------------------------------------------------------------------------
--  simple_exploration.lua
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
name = "simple_exploration"
fsm = SkillHSM:new{name = name, start = "INIT", debug = false}
depends_skills = {"motor_move", "moveto"}
depends_interfaces = {{v = "pose", type = "Position3DInterface", id = "Pose"}}

documentation = [==[Simple exploration for evaluation of danielhonies thesis

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT_UPPER_LIMIT = 60

fsm:define_states{
    export_to = _M,
    {"INIT", JumpState},
    {"CORNER_0_0", SkillJumpState, skills = {{moveto}}, final_to = "TURN1"},
    {"TURN1", SkillJumpState, skills = {{motor_move}}, final_to = "CORNER_0_1"},
    {"CORNER_0_1", SkillJumpState, skills = {{moveto}}, final_to = "TURN2"},
    {"TURN2", SkillJumpState, skills = {{motor_move}}, final_to = "CORNER_1_1"},
    {"CORNER_1_1", SkillJumpState, skills = {{moveto}}, final_to = "TURN3"},
    {"TURN3", SkillJumpState, skills = {{motor_move}}, final_to = "CORNER_1_0"},
    {"CORNER_1_0", SkillJumpState, skills = {{moveto}}, final_to = "TURN4"},
    {"TURN4", SkillJumpState, skills = {{motor_move}}, final_to = "FINAL"}
}

fsm:add_transitions{{"INIT", "CORNER_0_0", cond = true}}

function TURN1:init() self.args["motor_move"] = {ori = 1.570795, vel_rot = 0.2} end

function TURN2:init() self.args["motor_move"] = {ori = 1.570795, vel_rot = 0.2} end

function TURN3:init() self.args["motor_move"] = {ori = 1.570795, vel_rot = 0.2} end

function TURN4:init() self.args["motor_move"] = {ori = 1.570795, vel_rot = 0.2} end

function CORNER_0_0:init()
    self.args["moveto"] = {ori = 1.570795, x = -0.5, y = 0.5}
end

function CORNER_0_1:init()
    self.args["moveto"] = {ori = 3.14159, x = -0.5, y = 5.5}
end

function CORNER_1_1:init()
    self.args["moveto"] = {ori = -1.570795, x = -4.5, y = 5.5}
end

function CORNER_1_0:init() self.args["moveto"] = {ori = 0, x = -4.5, y = 1.5} end
