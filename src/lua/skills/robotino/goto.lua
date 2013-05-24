
----------------------------------------------------------------------------
--  goto.lua - 
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré
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
name               = "goto"
fsm                = SkillHSM:new{name=name, start="CHECK_PARAMS"}
depends_skills     = { "relgoto", "ppgoto" }
depends_interfaces = {
   {v = "pose", type="Position3DInterface", id="Pose"}
}

documentation      = [==[Move to a known (named) location.
@param goto_name Name of the place we want to go to.
@

Available names:
================
M1-M10 = m1-m10:              Machines 1-10,
R1, R2:                       Recycling gates 1 and 2,
T = TEST = Test:              Test station,
EGI = ExpressGoodInsertion:   Express good insertion area,
D1-D3 = Delivery1-Delivery3:  Delivery gates 1-3.
]==]

local MAX_TRANSERR = 0.16
local MAX_ROTERR = 0.16
local MAX_POS_MISS = 4
-- Initialize as skill module
skillenv.skill_module(_M)

local tf_mod = require 'tf_module'


function pose_ok(self)
   return (math.abs(self.fsm.vars.goto_x - pose:translation(0)) <= MAX_TRANSERR
    and math.abs(self.fsm.vars.goto_y - pose:translation(1)) <= MAX_TRANSERR
    and math.abs(self.fsm.vars.goto_ori - 2*math.acos(pose:rotation(3))) <= MAX_ROTERR)
end

function missed_too_often(self)
   return (self.fsm.vars.num_pos_missed >= MAX_POS_MISS)
end

fsm:define_states{ export_to=_M,
   closure={missed_too_often = missed_too_often,
      pose_ok = pose_ok},
   {"CHECK_PARAMS", JumpState},
   {"DO_RELGOTO", SkillJumpState, skills={{relgoto}}, final_to="WAIT_POSE", fail_to="FAILED"},
   {"DO_PPGOTO", SkillJumpState, skills={{ppgoto}}, final_to="WAIT_POSE", fail_to="FAILED"},
   {"WAIT_POSE", JumpState},
   {"CHECK_POSE", JumpState},
   {"MISSED", JumpState}
}


fsm:add_transitions{
   {"CHECK_PARAMS", "DO_RELGOTO", cond="not vars.place"},
   {"CHECK_PARAMS", "DO_PPGOTO", cond="fsm.vars.place"},
   {"CHECK_PARAMS", "FAILED", cond="fsm.vars.goto_name", desc="goto_name param is deprecated!"},
   {"WAIT_POSE", "CHECK_POSE", timeout=0.5},
   {"CHECK_POSE", "FINAL", cond=pose_ok, desc="Pose reached" },
   {"CHECK_POSE", "MISSED", cond="not pose_ok()"},
   {"MISSED", "DO_RELGOTO", cond="not missed_too_often()"},
   {"MISSED", "FAILED", cond=missed_too_often, desc="Posed missed"}
}

function CHECK_PARAMS:init()
   self.fsm.vars.num_pos_missed=0

   if self.fsm.vars.place then
      return
   else
      self.fsm.vars.x   = self.fsm.vars.x
                       or self.fsm.vars.goto_x
                       or pose:translation(0)
      self.fsm.vars.y   = self.fsm.vars.y
                       or self.fsm.vars.goto_y
                       or pose:translation(1)
      self.fsm.vars.ori = self.fsm.vars.ori
                       or self.fsm.vars.goto_ori
                       or 2*math.acos(pose:translation(3))
   end
end

function MISSED:init()
   self.fsm.vars.num_pos_missed = self.fsm.vars.num_pos_missed + 1
end

function DO_RELGOTO:init()
   local rel_pos = tf_mod.transform({
         x = self.fsm.vars.x,
         y = self.fsm.vars.y,
         ori = self.fsm.vars.ori},
      "/map", "/base_link")

   self.skills[1].x = rel_pos.x
   self.skills[1].y = rel_pos.y
   self.skills[1].ori = rel_pos.ori
end

function DO_PPGOTO:init()
   self.skills[1].place = self.fsm.vars.place
end
