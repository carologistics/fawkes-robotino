----------------------------------------------------------------------------
--  tsp_solve.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2021  Gjorgji Nikolovski
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
name               = "tsp_solve"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"goto"}
depends_interfaces = {}
documentation      = 
[==[ 
    tsp_solve
    This skill does:
        - Solves the travelling salesman problem given a list of coordinates for possible drive points in a 5x5 grid using a python script call
        @param params  [string]             space separated list of parameters. first parameter is the roundtrip condition (values 0 and 1).the rest are coordinates in robocup 
                                            5x5 grid (first entry of coordinate list is starting point). example: [M-Z12, M-Z33, M-Z51]
    Notes: 
        - The first element in the coordinate list is assumed to be the starting point of the robot.
        - Coordinates can be of structure M-ZXX or G-X-X. Using the M-Z prefix results in the skill executing goto itself. Using the G- prefix results in
        the skill returning the optimal sequence of coordinates in an errorstring made up of the coordinates in G-X-X format beginning with an '>' followed
        one after another
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function travel_finished(self)
    return self.fsm.vars.coords_index >= #self.fsm.vars.result_coords
end

fsm:define_states{ export_to=_M, closure={travel_finished=travel_finished},
    {"INIT",                    JumpState},
    {"WAIT",                    JumpState},
    {"GOTO",                    SkillJumpState, skills={{goto}}, final_to="WAIT", fail_to="FAILED"},
}

fsm:add_transitions{
    {"INIT", "GOTO", cond="vars.finished and not vars.agent_call"},
    {"INIT", "FAILED", cond="vars.finished and vars.agent_call"},
    {"WAIT", "GOTO", timeout=5},
    {"WAIT", "FINAL", cond="travel_finished(self)"}
}

function INIT:init()
    self.fsm.vars.finished = false
    local py_input_string = ""
    local params_table = string.gmatch(self.fsm.vars.params, "[^%s]+")
    local roundtrip = params_table[1]
    local coords = table.unpack(params_table, 2, #params_table)
    self.fsm.vars.agent_call = string.sub(coords[1],1,1) == "G"
    if self.fsm.vars.agent_call then
        for i, zone in ipairs(coords) do
            if zone ~= "" and zone ~= "\n" then
                py_input_string = py_input_string.." "..string.sub(zone, -3, -3)..string.sub(zone, -1, -1)
            end
        end
    else
        for i, zone in ipairs(coords) do
            if zone ~= "" and zone ~= "\n" then
                py_input_string = py_input_string.." "..string.sub(zone, -2, -1)
            end
        end
    end
    local handle = io.popen("python /home/robotino/fawkes-robotino/src/lua/skills/robotino/tsp_robotino.py "..tostring(roundtrip)..py_input_string)
    self.fsm.vars.py_result_string = string.sub(handle:read("*a"),0,-1)
    handle:close()
    
    if self.fsm.vars.agent_call then
        self.fsm.vars.agent_string = ""
        local numbers = string.gmatch(self.fsm.vars.py_result_string, "[^%s]+")
        for number in table.unpack(numbers, 1, #numbers-1) do
            self.fsm.vars.agent_string = self.fsm.vars.agent_string.."G-"..string.sub(number,1,1).."-"..string.sub(number,2,2).." "
        end
        self.fsm.vars.agent_string = self.fsm.vars.agent_string.."G-"..string.sub(numbers[-1],1,1).."-"..string.sub(numbers[-1],2,2)
    else
        self.fsm.vars.result_coords = {}
        for number in string.gmatch(self.fsm.vars.py_result_string, "[^%s]+") do
            table.insert(self.fsm.vars.result_coords, "M-Z"..number)
        end
        self.fsm.vars.coords_index = 1
    end
    
    self.fsm.vars.finished = true
end

function WAIT:init()
end

function GOTO:init()
    local transform = fawkes.tf.StampedTransform:new()
    tf:lookup_transform("map", "odom", transform)
    self.args["goto"] = { place = self.fsm.vars.result_coords[fsm.vars.coords_index], ori=fawkes.tf.get_yaw(transform), ori_tolerance=1.6, trans_tolerance=0.7 }
    self.fsm.vars.coords_index = fsm.vars.coords_index + 1
end

function FAILED:init()
    if self.fsm.vars.error then
        self.fsm:set_error(self.fsm.vars.agent_string)
        return
    end
    if self.fsm.vars.agent_string then
        self.fsm:set_error(self.fsm.vars.agent_string)
    end
    
end
