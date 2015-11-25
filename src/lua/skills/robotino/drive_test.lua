
----------------------------------------------------------------------------
--  drive_test.lua
--
--  Created: Sat Jun 14 15:13:19 2014
--  Copyright  2014       Frederik Zwilling
--             2014-2015  Tobias Neumann
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
name               = "drive_test"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"ppgoto_waypoints"}
depends_interfaces = { }

documentation      = [==[Drives between the given list of navgraph-points

Parameters:
      pps: List of points to drive to e.g. drive_test{pps={"P64", "P92", "P73", "P62", "P93"}}
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"GOTO", SkillJumpState, skills={{ppgoto_waypoints}}, final_to="GOTO", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "GOTO", cond=true}
}

function GOTO:init()
   self.args["ppgoto_waypoints"].wp = self.fsm.vars.pps
end
