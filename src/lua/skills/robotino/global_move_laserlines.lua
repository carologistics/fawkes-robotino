
----------------------------------------------------------------------------
--  global_move_laserlines.lua
--
--  Created: Sat Jul 10 20:57:19 2014
--  Copyright  2014  Tobias Neumann
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
name               = "global_move_laserlines"
fsm                = SkillHSM:new{name=name, start="GLOBAL", debug=false}
depends_skills     = {"global_motor_move", "align_laserlines"}
depends_interfaces = {
}

documentation      = [==[Drives to the given place with global_motor_move and low limits and align with laserlines

Parameters:
   place (string) : Navgraph point to got to
   puck: (bool)   : true for puck false or empty otherwise
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"GLOBAL", SkillJumpState, skills={{global_motor_move}}, final_to="ALIGN", fail_to="FAILED"},
   {"ALIGN", SkillJumpState, skills={{align_laserlines}}, final_to="FINAL", fail_to="GLOBAL_BACKUP"},
   {"GLOBAL_BACKUP", SkillJumpState, skills={{global_motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
}

function GLOBAL:init()
   self.skills[1].place   = fsm.vars.place
   self.skills[1].puck    = fsm.vars.puck
   self.skills[1].turn    = false
end

function ALIGN:init()
   self.skills[1].place = fsm.vars.place
end

function GLOBAL_BACKUP:init()
   self.skills[1].place   = fsm.vars.place
   self.skills[1].puck    = fsm.vars.puck
   self.skills[1].turn    = true
end
