
----------------------------------------------------------------------------
--  shelf_put.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
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
name               = "shelf_put"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { }
depends_interfaces = { }

documentation      = [==[ shelf_put

                          This skill does:
                          - drives to the given Navgraphpunkt (ppgoto)
                          - aligns to the machine             (align_mps)
                          - Puts puck onto Shelf              (SKILL-TODO)

                          @param nav_point  string  the name of the navgraph point to drive to
                          @param slot       string  the slot to put the puck of; options ( LEFT | MIDDLE | RIGHT )
                          @param tag_id     int     the tag_id to variafy the alignmend to the correct machine

                          TODO this skill
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT",       JumpState,  },
   {"DUMMY_WAIT", JumpState,  },
}

fsm:add_transitions{
   {"INIT",       "DUMMY_WAIT", cond=true},
   {"DUMMY_WAIT", "FINAL",      timeout=10},
}
