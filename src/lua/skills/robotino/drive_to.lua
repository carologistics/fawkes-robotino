
----------------------------------------------------------------------------
--  drive_to.lua
--
--  Created: Sat Jul 12 13:25:47 2014
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
name               = "drive_to"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "take_puck_to", "ppgoto","global_move_laserlines" }
depends_interfaces = { }

documentation      = [==[Drive to close point (property = highway_exit) with colisoin avoidance and last part with global_move_laserlines

Parameters:
      place: Where to drive to
      puck:  If we need to drive with a puck
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"SKILL_TAKE_PUCK", SkillJumpState, skills={{take_puck_to}}, final_to="TIMEOUT", fail_to="FAILED"},
   {"SKILL_PPGOTO",    SkillJumpState, skills={{ppgoto}},       final_to="TIMEOUT", fail_to="FAILED"},
   {"TIMEOUT", JumpState},
   {"SKILL_GLOBAL_MOVE_LASERLINES", SkillJumpState, skills={{global_move_laserlines}}, final_to="FINAL", fail_to="FINAL"},
}

fsm:add_transitions{
   { "INIT",    "FAILED", cond="not navgraph" },
   { "INIT",    "SKILL_TAKE_PUCK", cond="self.fsm.vars.puck" },
   { "INIT",    "SKILL_PPGOTO",    cond=true },
   { "TIMEOUT", "SKILL_GLOBAL_MOVE_LASERLINES", timeout=0.5 },
}

function INIT:init()
   -- do gerneral stuff here
   if self.fsm.vars.same_place then
      self.fsm.vars.closest_node = self.fsm.vars.place
   else
      self.fsm.vars.closest_node = navgraph:closest_node_to(self.fsm.vars.place, "highway_exit"):name()
   end
end

function SKILL_TAKE_PUCK:init()
   self.skills[1].place = self.fsm.vars.closest_node
end

function SKILL_PPGOTO:init()
   self.skills[1].place = self.fsm.vars.closest_node
end

function SKILL_GLOBAL_MOVE_LASERLINES:init()
   self.skills[1].place = self.fsm.vars.place
   self.skills[1].puck  = self.fsm.vars.puck
end
