----------------------------------------------------------------------------
--  gripper.lua - Skill to open or close Robotino AX12 gripper
--
--  Created: Sat Feb 28 10:46:33 2015
--  Copyright  2014  Sebastian Reuter
--             2014  Tim Niemueller
--             2015  Nicolas Limpert
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
name = "gcode"
fsm = SkillHSM:new{name = name, start = "CHECK_WRITER", debug = false}
depends_skills = nil
depends_interfaces = {
    {v = "gcode", type = "GcodeInterface", id = "Gcode"}
}

documentation = [==[
    @param commands  array of gcode strings to execute, e.g.
                     {"G1 X1 Y0", "G1 X2 Y0", "G0 Z5"}
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function input_ok()
    local cmds = fsm.vars.commands
    if type(cmds) ~= "table" or #cmds == 0 then
        print("No commands given (expecting a non‐empty array)")
        return false
    end
    for i, c in ipairs(cmds) do
        if type(c) ~= "string" then
            print(("Command #%d is not a string!"):format(i))
            return false
        end
    end
    return true
end

-- States
fsm:define_states{
    export_to = _M,
    closure = {
        gcode = gcode,
        input_ok = input_ok,
    },
    {"CHECK_WRITER", JumpState},
    {"COMMAND", JumpState},
}

-- Transitions
fsm:add_transitions{
    {
        "CHECK_WRITER",
        "FAILED",
        cond = "not input_ok()",
        desc = "Input not correct"
    }, {
        "CHECK_WRITER",
        "FAILED",
        precond = "not gcode:has_writer()",
        desc = "No writer for gripper"
    },
    {
        "COMMAND", "FINAL", timeout = 0.2, desc = "Wait for command to finish"
    },
    {"CHECK_WRITER", "COMMAND", cond = true, desc = "Writer ok got to command"},
}

function COMMAND:init()
    for _, cmdstr in ipairs(self.fsm.vars.commands) do
        local msg = gcode.GcodeMessage:new()

        -- print(("Command to send: %s"):format(cmdstr))
        msg:set_command(cmdstr)
        gcode:msgq_enqueue(msg)
    end
end
