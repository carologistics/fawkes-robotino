----------------------------------------------------------------------------
--  conveyor_align.lua - align orthogonal to the conveyor using the conveyor vision
--
--  Copyright  2013 The Carologistics Team
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
name               = "conveyor_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "conveyor_0", type = "Position3DInterface"},
}

documentation      = [==[aligns the robot orthogonal to the conveyor by using the
conveyor vision
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local TOLERANCE = 0.02
local MAX_TRIES = 3

function no_writer()
   return not conveyor:has_writer()
end

function tolerance_not_ok()
   return not math.abs(conveyor_0:translation(1)) <= TOLERANCE  
end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"SETTLE", JumpState},
   {"CHECK", JumpState},
}

fsm:add_transitions{
   closure={tolerance_ok=tolerance_ok, MAX_TRIES=MAX_TRIES},
   {"INIT", "FAILED", cond=no_writer},
   {"INIT", "DRIVE", cond=true},
   {"DRIVE", "SETTLE", cond=true},
   {"SETTLE", "CHECK", timeout=1},
   {"CHECK", "FAILED", cond="vars.tries > MAX_TRIES - 1"},
   {"CHECK", "DRIVE", cond=tolerance_not_ok},
   {"CHECK", "FINAL", cond=true},
}

function INIT:init()
   self.fsm.vars.counter = 0
end

function DRIVE:init()
   self.skills[1].y = conveyor_0:translation(1)
end

function CHECK:init()
   self.fsm.vars.counter = self.fsm.vars.counter + 1
end
