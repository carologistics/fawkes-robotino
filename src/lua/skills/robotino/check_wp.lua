----------------------------------------------------------------------------
--  check_wp.lua
--
--  Created Wed Apr 15
--  Copyright  2019 Sebastian Eltester
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
name               = "check_wp"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=true}
depends_skills     = {}
depends_interfaces = {
   {v = "robotino_sensor", type = "RobotinoSensorInterface", id="Robotino"} -- Interface to read I/O ports
}

documentation      = [==[
Skill to check the gripper sensor for a workpiece.

Parameters:
   @param check_wp
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- function to evaluate sensor data
function is_grabbed()
 if robotino_sensor:is_digital_in(0) == false and robotino_sensor:is_digital_in(1) == true then -- white cable on DI1 and black on DI2
    return true
 else
    return false
 end
end

function has_writer()
 if not robotino_sensor:has_writer() then
   print_warn("No robotino sensor writer to check sensor")
   return false
 else
    return true
 end
end

fsm:define_states{ export_to=_M,
   closure={is_grabbed=is_grabbed, check_writer=check_writer},
   {"CHECK_WRITER", JumpState}
   {"INIT", JumpState},
}

fsm:add_transitions{
   {"CHECK_WRITER", "FINAL", cond="not has_writer()", desc="No writer for gripper sensor"}
   {"CHECK_WRITER", "INIT", cond=true}
   {"INIT", "FINAL", cond=is_grabbed, desc="Holding workpiece"},
   {"INIT", "FINAL", cond=vars.check_wp, desc="Workpiece check disabled"},
   {"INIT", "FAILED", cond=true}, check_writer=check_writer,
}

