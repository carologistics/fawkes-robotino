----------------------------------------------------------------------------
--  turn_to_search.lua
--
--  Created: Thu Jun 08 15:25:00 2023
--  Copyright  2023 Daniel Swoboda
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
name = "turn_to_search"
fsm = SkillHSM:new{name = name, start = "INIT", debug = false}
depends_skills = {"motor_move"}
depends_interfaces = {{v = "pose", type = "Position3DInterface", id = "Pose"}}

documentation = [==[Turns and waits the robot in 45 degree intervals
@param: turns   how many 45 degree turns we want to take

]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT_UPPER_LIMIT = 5

fsm:define_states{
    export_to = _M,
    {"INIT", JumpState},
    {"WAIT1", JumpState},
    {"TURN1", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT2"},
    {"WAIT2", JumpState},
    {"TURN2", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT3"},
    {"WAIT3", JumpState},
    {"TURN3", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT4"},
    {"WAIT4", JumpState},
    {"TURN4", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT5"},
    {"WAIT5", JumpState},
    {"TURN5", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT6"},
    {"WAIT6", JumpState},
    {"TURN6", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT7"},
    {"WAIT7", JumpState},
    {"TURN7", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT8"},
    {"WAIT8", JumpState},
    {"TURN8", SkillJumpState, skills = {{motor_move}}, final_to = "WAIT9"},
    {"WAIT9", JumpState, final_to = "FINAL"}
}

fsm:add_transitions{
    {"INIT", "WAIT1", cond = true},
    {"WAIT1", "TURN1", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN1", "WAIT2", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT2", "TURN2", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN2", "WAIT3", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT3", "TURN3", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN3", "WAIT4", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT4", "TURN4", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN4", "WAIT5", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT5", "TURN5", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN5", "WAIT6", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT6", "TURN6", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN6", "WAIT7", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT7", "TURN7", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN7", "WAIT8", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT8", "TURN8", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"TURN8", "WAIT9", timeout = TIMEOUT_UPPER_LIMIT}, -- this just creates the transision
    {"WAIT9", "FINAL", timeout = TIMEOUT_UPPER_LIMIT} -- this just creates the transision
}

function WAIT1:init() self.timeout_time = 2 end

function TURN1:init() self.args["motor_move"] = {ori = 0.785398} end

function WAIT2:init() self.timeout_time = 2 end

function TURN2:init()
    if self.fsm.vars.turns > 1 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end

function WAIT3:init()
    if self.fsm.vars.turns > 1 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function TURN3:init()
    if self.fsm.vars.turns > 2 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end

function WAIT4:init()
    if self.fsm.vars.turns > 2 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function TURN4:init()
    if self.fsm.vars.turns > 3 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end

function WAIT5:init()
    if self.fsm.vars.turns > 3 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function WAIT6:init()
    if self.fsm.vars.turns > 4 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function TURN5:init()
    if self.fsm.vars.turns > 4 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end

function WAIT7:init()
    if self.fsm.vars.turns > 5 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function TURN6:init()
    if self.fsm.vars.turns > 5 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end

function WAIT8:init()
    if self.fsm.vars.turns > 6 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function TURN7:init()
    if self.fsm.vars.turns > 6 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end

function WAIT9:init()
    if self.fsm.vars.turns > 7 then
        self.timeout_time = 2
    else
        self.timeout_time = 0.0
    end
end

function TURN8:init()
    if self.fsm.vars.turns > 7 then
        self.args["motor_move"] = {ori = 0.785398}
    end
end
