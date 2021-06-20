----------------------------------------------------------------------------
--  tsp_solve.lua
--
--  Created: Thu Aug 14 14:32:47 2021
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"goto"}
documentation      = [==[ 
    tsp_solve
    This skill does:
        - Solves the travelling salesman problem given a list of coordinates for possible drive points in a 5x5 grid using a python script call
        @param grid_coords  [list of strings]    list of coordinates in robocup 5x5 grid (first entry in list is starting). example: [M-Z12, M-Z33, M-Z51]
        @param loop         [int]                wether to end at the start point or not (value is 0 or 1)
]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

fsm:define_states{ export_to=_M, closure={},
    {"INIT",                    JumpState},
    {"WAIT",                    JumpState},
    {"GOTO",                    SkillJumpState, skills={"goto"}, final_to="WAIT", fail_to="FAILED"},
    {"END",                     JumpState}
}

fsm:add_transitions{
    {"INIT", "GOTO", cond=true},
    {"GOTO", "WAIT", cond=true},
    {"WAIT", "GOTO", timeout=1},
    {"WAIT", "END", cond=travel_finished}
}


function travel_finished(self)
    return self.fsm.vars.coords_index == #self.fsm.vars.result_coords-1
end



function INIT:init()

    local py_input_string = string.sub(self.fsm.vars.grid_coords[0], -2, -1)
    for coord in table.unpack(self.fsm.vars.grid_coords,1) do
        py_input_string = py_input_string.." "..string.sub(coord, -2, -1) 
    end
    local handle = io.popen("python tsp_robotino "..tostring(loop)..py_input_string)
    self.fsm.vars.py_result_string = string.sub(handle.read("*a"),0,-1)
    handle.close()
    self.fsm.vars.result_coords = {}
    for number in string.gmatch(self.fsm.vars.py_result_string, "[^%s]+") do
        table.insert(self.fsm.vars.result_coords, "M-Z"..number)
    end
    self.fsm.vars.coords_index = -1
end

function WAIT:wait()
end

function END:end()
end

function GOTO:do_goto()
    if self.fsm.vars.coords_index ~= #self.fsm.vars.result_coords-1 then
        self.fsm.vars.coords_index = fsm.vars.coords_index + 1
        self.args["goto"] = { place = self.fsm.vars.result_coords[fsm.vars.coords_index] }
    end
end
