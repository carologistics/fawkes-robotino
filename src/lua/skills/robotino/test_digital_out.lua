
----------------------------------------------------------------------------
--  test_digital_out.lua - Skill to test the functionality of the digital outputs
--
--  Created: Fri Jun 21 10:46:33 2019
--  Copyright  2019  Morian Sonnet
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
name               = "test_digital_out"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
    {v = "robotino", type = "RobotinoSensorInterface", id="Robotino"},
}

documentation      = [==[
    @param command    can be : ( SET )
    @param num  the number of the output which shall be acted on
    @param enabled true or false
]==]



-- Initialize as skill module
skillenv.skill_module(_M)


-- States
fsm:define_states{
   export_to=_M,
   closure={robotino=robotino},
   {"COMMAND", JumpState},
   {"CHECK_WRITER", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"COMMAND", "FINAL", timeout=0.2},
   {"CHECK_WRITER", "FAILED", precond="not robotino:has_writer()", desc="No writer for interface"},
   {"CHECK_WRITER", "COMMAND", cond=true},
}

function COMMAND:init()
   message = robotino.SetDigitalOutputMessage:new()
   message:set_digital_out(self.fsm.vars.num)
   message:set_enabled(self.fsm.vars.enabled)
   robotino:msgq_enqueue(message)
end
