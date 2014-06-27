
----------------------------------------------------------------------------
--  drive_test.lua
--
--  Created: Sat Jun 14 15:13:19 2014
--  Copyright  2014  Frederik Zwilling
--             2014  Tobias Neumann
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
depends_skills     = {"ppgoto"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino"}
}

documentation      = [==[Drives between the given list of navgraph-points

Parameters:
      pps: List of points to drive to e.g. drive_test{pps={"P64", "P92", "P73", "P62", "P93"}}
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"GOTO", SkillJumpState, skills={{ppgoto}}, final_to="GOTO", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "GOTO", timeout=1}
}

function INIT:init() 
   fsm.vars.table_size = table.getn( self.fsm.vars.pps )             -- get list of targets
   fsm.vars.table_pos  = 1
end

function GOTO:init()
   if fsm.vars.table_pos > fsm.vars.table_size then                  -- if at end, start restart
      fsm.vars.table_pos = 1
   end
   fsm.vars.next_target = self.fsm.vars.pps[fsm.vars.table_pos]      -- get next target
   
   fsm.vars.table_pos = fsm.vars.table_pos + 1                       -- increment list position

   self.skills[1].place = fsm.vars.next_target
end
