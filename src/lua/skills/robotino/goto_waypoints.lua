----------------------------------------------------------------------------
--  goto_waypoints.lua
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
name               = "goto_waypoints"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"goto"}
depends_interfaces = { }

documentation      = [==[Drives the given list of navgraph-points without using the pathplaner

Parameters:
      wp:     List of points to drive to e.g. goto_waypoints{wp={"P64", "P92", "P73", "P62", "P93"}}
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

function waypoints_done()
   return fsm.vars.table_pos > fsm.vars.table_size
end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"GOTO", SkillJumpState, skills={{goto}}, final_to="GOTO", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",   "GOTO", cond=true},
   {"GOTO", "FINAL",  cond=waypoints_done},
}

function INIT:init()
   fsm.vars.table_size = table.getn( self.fsm.vars.wp )         -- get list of targets
   fsm.vars.table_pos  = 0
end

function GOTO:init()
   fsm.vars.table_pos = fsm.vars.table_pos + 1                  -- increment list position
   self.args["goto"] = { place = self.fsm.vars.wp[fsm.vars.table_pos] }
end
