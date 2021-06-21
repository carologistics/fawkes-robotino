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
depends_interfaces = {
    {v = "navigator", type="NavigatorInterface", id="Navigator"},
}
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

function travel_finished(self)
    return self.fsm.vars.coords_index == #self.fsm.vars.result_coords
end

fsm:define_states{ export_to=_M, closure={travel_finished=travel_finished},
    {"INIT",                    JumpState},
    {"WAIT",                    JumpState},
    {"GOTO",                    SkillJumpState, skills={{goto}}, final_to="WAIT", fail_to="FAILED"},
}

fsm:add_transitions{
    {"INIT", "GOTO", cond="vars.finished"},
    {"WAIT", "GOTO", timeout=5},
    {"WAIT", "FINAL", cond="travel_finished(self)"}
}

function INIT:init()
    self.fsm.vars.finished = false
    local py_input_string = ""
    for i, zone in ipairs(self.fsm.vars.grid_coords) do
      py_input_string = py_input_string.." "..string.sub(zone, -2, -1)
    end
    local handle = io.popen("python /home/robotino/fawkes-robotino/src/lua/skills/robotino/tsp_robotino.py "..tostring(self.fsm.vars.loop)..py_input_string)
    self.fsm.vars.py_result_string = string.sub(handle:read("*a"),0,-1)
    handle:close()
    self.fsm.vars.result_coords = {}
    for number in string.gmatch(self.fsm.vars.py_result_string, "[^%s]+") do
        table.insert(self.fsm.vars.result_coords, "M-Z"..number)
    end
    self.fsm.vars.coords_index = 1
    self.fsm.vars.finished = true
end

function WAIT:init()
end

function GOTO:init()
    if self.fsm.vars.coords_index ~= #self.fsm.vars.result_coords then
        local node = navgraph:node(self.fsm.vars.result_coords[fsm.vars.coords_index])
        if node:is_valid() then
            if node:has_property("orientation") then
              self.fsm.vars.ori = node:property_as_float("orientation");
            end
        end
        self.args["goto"] = { place = self.fsm.vars.result_coords[fsm.vars.coords_index], ori=self.fsm.vars.ori, ori_tolerance=1.6, trans_tolerance=0.7 }
        self.fsm.vars.coords_index = fsm.vars.coords_index + 1
    end
end