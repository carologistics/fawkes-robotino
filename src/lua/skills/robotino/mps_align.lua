
----------------------------------------------------------------------------
--  align_mps.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--                   Johannes Rothe
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
name               = "mps_align"
fsm                = SkillHSM:new{name=name, start="SKILL_ALIGN_TAG", debug=true}
depends_skills     = { "align_tag", "motor_move" }
depends_interfaces = {
{v = "line1", type="LaserLineInterface", id="/laser-lines/1"}
}

documentation      = [==[ align_mps

                          This skill does:
                          - aligns to the machine via sensor information AND a optional offsets which are given as parameters 

                          @param tag_id     int     optional the tag_id for align_tag
                          @param x          float   the x offset after the alignmend
                          @param y          float   optional the y offset after the alignmend
                          @param ori        float   optional the ori offset after the alignmend
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")

local TAG_X_ERR=0.02

function see_line()
   printf("vis_hist: %f", line1:visibility_history())
   return line1:visibility_history() > 30
end

fsm:define_states{ export_to=_M, closure={see_line = see_line},
   {"SKILL_ALIGN_TAG", SkillJumpState, skills={{align_tag}},
      final_to="SEE_LINE", fail_to="FAILED"},
   {"SEE_LINE", JumpState},
   {"LINE_SETTLE", JumpState},
   {"ALIGN_WITH_LASERLINES", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"SKILL_ALIGN_TAG", "FAILED", precond="not vars.x", desc="x argument missing"},
   {"SEE_LINE", "LINE_SETTLE", cond=see_line, desc="Seeing a line"},
   {"SEE_LINE", "FAILED", timeout=3, desc="Not seeing a line, continue just aligned by tag"},
   {"LINE_SETTLE", "ALIGN_WITH_LASERLINES", timeout=0.5, desc="let the line distance settle"}
}

function SKILL_ALIGN_TAG:init()
   -- use defaults for optional args
   self.fsm.vars.y = self.fsm.vars.y or 0
   self.fsm.vars.ori = self.fsm.vars.ori or 0
   -- align by ALIGN_DISTANCE from tag to base_link with align_tag
   local tag_transformed = tfm.transform({x=self.fsm.vars.x, y=self.fsm.vars.y, ori=self.fsm.vars.ori}, "/base_link", "/cam_tag")
   -- give align_tag the id if we have one
   self.skills[1].tag_id = self.fsm.vars.tag_id
   self.skills[1].x = self.fsm.vars.x + TAG_X_ERR
   self.skills[1].y = -self.fsm.vars.y
   self.skills[1].ori = self.fsm.vars.ori
end

function ALIGN_WITH_LASERLINES:init()
   -- align by ALIGN_DISTANCE from tag to base_link with the lase_line
   local line_transformed = tfm.transform({x=line1:point_on_line(0), y=0, ori=line1:bearing()}, line1:frame_id(), "/base_link")
   printf("line transformed x: %f", line_transformed.x)
   printf("line transformed ori: %f", line_transformed.ori)
   self.skills[1].x = line_transformed.x - self.fsm.vars.x
   self.skills[1].y = 0 -- can't improve y coordinate with laserlines so leave it
   self.skills[1].ori = line_transformed.ori + self.fsm.vars.ori
   self.skills[1].tolerance = {x=0.01, y=0.01, ori=0.02}
end
