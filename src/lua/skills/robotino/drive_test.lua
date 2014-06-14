
----------------------------------------------------------------------------
--  drive_test.lua
--
--  Created: Sat Jun 14 15:13:19 2014
--  Copyright  2014  Frederik Zwilling
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
fsm                = SkillHSM:new{name=name, start="GOTO_1", debug=false}
depends_skills     = {"ppgoto"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino"}
}

documentation      = [==[Drives between two navgraph-points there and back

Parameters:
      z1: Point to drive to first
      z2: Point to drive to second
]==]
-- Initialize as skill module
skillenv.skill_module(_M)


fsm:define_states{ export_to=_M,
   {"GOTO_1", SkillJumpState, skills={{ppgoto}}, final_to="GOTO_2", fail_to="FAILED"},
   {"GOTO_2", SkillJumpState, skills={{ppgoto}}, final_to="GOTO_1", fail_to="FAILED"}
}

fsm:add_transitions{
}

function GOTO_1:init()
   if self.fsm.vars.z1 == nil then
      printf("No place for z1 given!")
   end

   self.skills[1].place = self.fsm.vars.z1
end

function GOTO_2:init()
   if self.fsm.vars.z2 == nil then
      printf("No place for z2 given!")
   end

   self.skills[1].place = self.fsm.vars.z2
end