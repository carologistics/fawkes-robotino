
----------------------------------------------------------------------------
--  align_mps.lua
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
name               = "align_mps"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { }
depends_interfaces = { }

documentation      = [==[ align_mps

                          This skill does:
                          - aligns to the machine via sensor information AND a optional offset that is given as config values by the tag_id
                          - executes an additional offset by the given x, y, ori params

                          @param tag_id     int     the tag_id to variafy the alignmend to the correct machine
                          @param x          float   optional the x offset after the alignmend
                          @param y          float   optional the y offset after the alignmend
                          @param ori        float   optional the ori offset after the alignmend

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
