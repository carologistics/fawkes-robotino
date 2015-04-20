
----------------------------------------------------------------------------
--  shelf_pick.lua
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
name               = "slide_put"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move", "ax12gripper"}
depends_interfaces = {
   {v = "line1", type = "LaserLineInterface"},
}

documentation      = [==[ slide_put

                          This skill does:
                          - drives to the given Navgraphpunkt (ppgoto)
                          - aligns to the machine             (align_mps)
                          - Picks of Shelf                    (SKILL-TODO)

                          @param nav_point  string  the name of the navgraph point to drive to
                          @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
                          @param tag_id     int     the tag_id to variafy the alignmend to the correct machine

                          TODO this skill
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT",       JumpState,  },
   {"GOTO_SLIDE", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_SLIDE", fail_to="FAILED"},
   {"APPROACH_SLIDE", SkillJumpState, skills={{motor_move}}, final_to="STORE_PRODUCT", fail_to="FAILED"},
   {"STORE_PRODUCT", SkillJumpState, skills={{ax12gripper}}, final_to="LEAVE_SLIDE", fail_to="FAILED"},
   {"LEAVE_SLIDE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",       "GOTO_SLIDE", cond=true},
}


function GOTO_SLIDE:init()
   self.skills[1].x = 0.2
   if line1:end_point_1(1) < 0 then
     self.skills[1].y = line1:end_point_1(1) + 0.1
   else
     self.skills[1].y = line1:end_point_2(1) + 0.1
   end
--   self.skills[1].y = -0.02 -(self.fsm.vars.slot * 0.094)
end

function APPROACH_SLIDE:init()
   self.skills[1].x = 0.12
end

function STORE_PRODUCT:init()
   self.skills[1].open = true
end

function LEAVE_SLIDE:init()
   self.skills[1].x = -0.1
end
