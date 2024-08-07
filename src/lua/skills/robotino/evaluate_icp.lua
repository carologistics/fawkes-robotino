----------------------------------------------------------------------------
--  evaluate_icp.lua
--
--  Created Sun April 10
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
name = "evaluate_icp"
fsm = SkillHSM:new{name = name, start = "INIT", debug = true}
depends_skills = {"moveto", "get_product_from", "bring_product_to"}
depends_interfaces = {}

documentation = [==[
This skill is used to evaluate the point cloud based approach against the
visual servoing method. It requieres a starting point id as input and moves the
bot to this location after every manipulation attempt. Each possible side of
C-BS, C-RS1, and C-CS1 is target of either a pick or put action.

Parameters:
      @param startpoint_id (type int, in [1,...,10]) start manipulate_wp from this position for every predefined in-/output
      @param next_output_id (optional, type int, in [1,...,7]) start evaluating with this output
      @param next_input_id (optional, type int, in [1,2,3]) start evaluating with this input
]==]

local startpoints = {
    {x = -0.5, y = 4.5, ori = 3.14}, {x = -0.5, y = 4.5, ori = -1.57},
    {x = -3.5, y = 3.5, ori = 3.14}, {x = -3.5, y = 3.5, ori = 0.0},
    {x = -4.5, y = 1.5, ori = -1.57}, {x = -2.5, y = 1.5, ori = 1.57},
    {x = -2.5, y = 1.5, ori = 0.0}, {x = -2.5, y = 1.5, ori = -1.57},
    {x = -0.5, y = 0.5, ori = 3.14}, {x = -0.5, y = 0.5, ori = 1.57}
}

local target_outputs = {
    {target = "WORKPIECE", mps = "C-CS1", side = "input", shelf = "LEFT"},
    {target = "WORKPIECE", mps = "C-RS1", side = "output"},
    {target = "WORKPIECE", mps = "C-BS", side = "input"},
    {target = "WORKPIECE", mps = "C-CS1", side = "input", shelf = "MIDDLE"},
    {target = "WORKPIECE", mps = "C-BS", side = "output"},
    {target = "WORKPIECE", mps = "C-CS1", side = "input", shelf = "RIGHT"},
    {target = "WORKPIECE", mps = "C-CS1", side = "output"}
}

local target_inputs = {
    {target = "CONVEYOR", mps = "C-CS1", side = "input"},
    {target = "CONVEYOR", mps = "C-RS1", side = "input"},
    {target = "SLIDE", mps = "C-RS1", side = "input", slide = true}
}

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

function startpoint_valid() return fsm.vars.startpoint ~= nil end

function finished() return fsm.vars.output_done and fsm.vars.input_done end

fsm:define_states{
    export_to = _M,
    closure = {},
    {"INIT", JumpState},
    {
        "GOTO_START",
        SkillJumpState,
        skills = {{moveto}},
        final_to = "CHOOSE_ACTION",
        fail_to = "CHOOSE_ACTION"
    },
    {"CHOOSE_ACTION", JumpState},
    {
        "PICK",
        SkillJumpState,
        skills = {{get_product_from}},
        final_to = "PUT_POSSIBLE",
        fail_to = "GOTO_START"
    },
    {
        "PUT",
        SkillJumpState,
        skills = {{bring_product_to}},
        final_to = "GOTO_START",
        fail_to = "GOTO_START"
    },
    {"PUT_POSSIBLE", JumpState}
}

fsm:add_transitions{
    {"INIT", "GOTO_START", cond = startpoint_valid, desc = "Go to startpoint"},
    {"INIT", "FAILED", true, desc = "Startpoint is invalid"},
    {"CHOOSE_ACTION", "FINAL", cond = finished, desc = "All scenarios tested"},
    {
        "CHOOSE_ACTION",
        "PUT",
        cond = "not vars.pick_wp and not vars.input_done",
        desc = "Start putting down workpiece"
    }, {"CHOOSE_ACTION", "PICK", true, desc = "Start picking up workpiece"},
    {"PUT_POSSIBLE", "GOTO_START", timeout = 2, desc = "Evaluating grasp"}
}

function INIT:init()
    fsm.vars.startpoint = startpoints[fsm.vars.startpoint_id]
    print_info("[ICP] Evaluating starting point " .. fsm.vars.startpoint_id)
    print_info("[ICP] starting x value: " .. fsm.vars.startpoint.x)
    print_info("[ICP] starting y value: " .. fsm.vars.startpoint.y)
    print_info("[ICP] starting orientation: " .. fsm.vars.startpoint.ori)
    fsm.vars.output_done = false
    fsm.vars.input_done = false

    -- if next in-/output id is not set, start with 1
    if fsm.vars.next_output_id == nil then fsm.vars.next_output_id = 1 end
    if fsm.vars.next_input_id == nil then fsm.vars.next_input_id = 1 end

    -- if output does not exist, next output is set to 1 in case there are inputs
    -- to be tested
    if fsm.vars.next_output_id > 7 then
        fsm.vars.output_done = true
        fsm.vars.next_output_id = 1
    end

    -- if input does not exist, do not test them
    if fsm.vars.next_input_id > 3 then fsm.vars.input_done = true end

    -- start with picking
    fsm.vars.pick_wp = true
end

function GOTO_START:init()
    self.args["moveto"] = {
        x = fsm.vars.startpoint.x,
        y = fsm.vars.startpoint.y,
        ori = fsm.vars.startpoint.ori
    }
end

function PICK:init()
    fsm.vars.time_start = fawkes.Time:new():in_msec()
    fsm.vars.current_output_id = fsm.vars.next_output_id
    fsm.vars.next_target_output = target_outputs[fsm.vars.next_output_id]
    print_info("[ICP] Evaluating output " .. fsm.vars.next_output_id)
    print_info("[ICP] output target: " .. fsm.vars.next_target_output.target)
    print_info("[ICP] output mps: " .. fsm.vars.next_target_output.mps)
    print_info("[ICP] output side: " .. fsm.vars.next_target_output.side)

    if fsm.vars.next_target_output.shelf ~= nil then
        print_info("[ICP] output shelf: " .. fsm.vars.next_target_output.shelf)

        self.args["get_product_from"] = {
            place = fsm.vars.next_target_output.mps,
            side = fsm.vars.next_target_output.side,
            shelf = fsm.vars.next_target_output.shelf
        }
    else
        self.args["get_product_from"] = {
            place = fsm.vars.next_target_output.mps,
            side = fsm.vars.next_target_output.side
        }
    end

    -- take next output
    fsm.vars.next_output_id = fsm.vars.next_output_id + 1

    -- if output does not exist, next output is set to 1 in case there are inputs
    -- to be tested
    if fsm.vars.next_output_id > 7 then
        fsm.vars.output_done = true
        fsm.vars.next_output_id = 1
    end
end

function PICK:exit()
    local now = fawkes.Time:new():in_msec()
    print_info(
        "[ICP] Execution time for output " .. fsm.vars.current_output_id .. ": " ..
            now - fsm.vars.time_start)
end

function PUT:init()
    fsm.vars.time_start = fawkes.Time:new():in_msec()
    fsm.vars.current_input_id = fsm.vars.next_input_id
    fsm.vars.next_target_input = target_inputs[fsm.vars.next_input_id]
    print_info("[ICP] Evaluating input " .. fsm.vars.next_input_id)
    print_info("[ICP] input target: " .. fsm.vars.next_target_input.target)
    print_info("[ICP] input mps: " .. fsm.vars.next_target_input.mps)
    print_info("[ICP] input side: " .. fsm.vars.next_target_input.side)

    if fsm.vars.next_target_input.slide ~= nil then
        print_info("[ICP] input slide: " .. fsm.vars.next_target_input.slide)

        self.args["bring_product_to"] = {
            place = fsm.vars.next_target_input.mps,
            side = fsm.vars.next_target_input.side,
            slide = fsm.vars.next_target_input.slide
        }
    else
        self.args["bring_product_to"] = {
            place = fsm.vars.next_target_input.mps,
            side = fsm.vars.next_target_input.side
        }
    end

    -- take next input
    fsm.vars.next_input_id = fsm.vars.next_input_id + 1

    -- if input does not exist, all inputs are tested
    if fsm.vars.next_input_id > 3 then fsm.vars.input_done = true end

    -- pick afterwards
    fsm.vars.pick_wp = true
end

function PUT:exit()
    local now = fawkes.Time:new():in_msec()
    print_info("[ICP] Execution time for input " .. fsm.vars.current_input_id ..
                   ": " .. now - fsm.vars.time_start)
end

function PUT_POSSIBLE:init()
    -- set put as next action
    fsm.vars.pick_wp = false
end
